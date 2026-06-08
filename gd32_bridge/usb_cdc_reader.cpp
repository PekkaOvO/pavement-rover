#include "usb_cdc_reader.h"

#include <cstring>
#include <cerrno>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

namespace gd32_bridge {

UsbCdcReader::UsbCdcReader(FrameQueue &queue,
                           VehicleCmdPublisher &cmd_pub,
                           const std::string &device)
    : queue_(queue), cmd_pub_(cmd_pub), device_(device) {}

UsbCdcReader::~UsbCdcReader() { stop(); }

bool UsbCdcReader::start() {
    if (running_.exchange(true))
        return false;
    thread_ = std::thread(&UsbCdcReader::readLoop, this);
    return true;
}

void UsbCdcReader::stop() {
    running_ = false;
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    if (thread_.joinable())
        thread_.join();
}

// ---------------------------------------------------------------------------
// Header parsing (new 16-byte header)
// ---------------------------------------------------------------------------
bool UsbCdcReader::parseHeader(const uint8_t *data, size_t len,
                               UsbPacketHeader &hdr) {
    if (len < sizeof(UsbPacketHeader))
        return false;

    hdr.magic        = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    hdr.type         = data[2];
    hdr.event        = data[3];
    hdr.width        = (uint16_t)data[4]  | ((uint16_t)data[5] << 8);
    hdr.height       = (uint16_t)data[6]  | ((uint16_t)data[7] << 8);
    hdr.packet_id    = (uint16_t)data[8]  | ((uint16_t)data[9] << 8);
    hdr.packet_total = (uint16_t)data[10] | ((uint16_t)data[11] << 8);
    hdr.payload_len  = (uint16_t)data[12] | ((uint16_t)data[13] << 8);
    hdr.reserved     = (uint16_t)data[14] | ((uint16_t)data[15] << 8);

    return hdr.magic == 0xAA55;
}

// ---------------------------------------------------------------------------
// Push raw RGB565 to queue
// Queue frame format: [QUEUE_FRAME_RGB565(1B) or QUEUE_FRAME_RGB565_ANOM(1B)]
//                     [WIDTH(2B,LE)][HEIGHT(2B,LE)]
//                     [optional UsbDetObject(28B)]
//                     [RGB565_DATA(W*H*2)]
// ---------------------------------------------------------------------------
void UsbCdcReader::pushRgb565Frame(uint16_t width, uint16_t height,
                                   const uint8_t *rgb565, size_t rgb565_len,
                                   const UsbDetObject *detection) {
    const size_t det_len = detection ? sizeof(UsbDetObject) : 0;
    std::vector<uint8_t> frame;
    frame.reserve(1 + 2 + 2 + det_len + rgb565_len);

    frame.push_back(detection ? QUEUE_FRAME_RGB565_ANOM : QUEUE_FRAME_RGB565);
    // width (little-endian)
    frame.push_back((uint8_t)(width));
    frame.push_back((uint8_t)(width >> 8));
    // height (little-endian)
    frame.push_back((uint8_t)(height));
    frame.push_back((uint8_t)(height >> 8));
    if (detection) {
        const uint8_t *det_bytes = reinterpret_cast<const uint8_t *>(detection);
        frame.insert(frame.end(), det_bytes, det_bytes + sizeof(UsbDetObject));
    }
    // RGB565 data
    frame.insert(frame.end(), rgb565, rgb565 + rgb565_len);

    if (!queue_.push(frame.data(), frame.size()))
        frames_dropped++;
}

// ---------------------------------------------------------------------------
// Main read loop
// ---------------------------------------------------------------------------
void UsbCdcReader::readLoop() {
    // Pre-allocate raw read buffer to max expected size to eliminate runtime realloc
    std::vector<uint8_t> buf;
    buf.reserve(2 * 1024 * 1024);  // 2 MB (matches overflow guard)

    // Track anomaly state: after STOP event, mark subsequent images as anomaly
    // until detection ends or an X_DELTA event is processed.
    bool anomaly_pending = false;

    while (running_) {
        try {
        // Open device if not open
        if (fd_ < 0) {
            fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd_ < 0) {
                std::cerr << "usb_cdc: cannot open " << device_
                          << ", retrying in 2s..." << std::endl;
                for (int i = 0; i < 20 && running_; ++i)
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            ioctl(fd_, TIOCEXCL);

            // Assert DTR (some GD32 CDC implementations need this to stream data)
            int dtr_flag = TIOCM_DTR;
            ioctl(fd_, TIOCMBIS, &dtr_flag);

            struct termios tio;
            if (tcgetattr(fd_, &tio) == 0) {
                cfmakeraw(&tio);
                tio.c_cflag |= CLOCAL | CREAD;
                tio.c_cc[VMIN]  = 1;
                tio.c_cc[VTIME] = 0;
                // Match Python script baudrate 115200
                cfsetispeed(&tio, B115200);
                cfsetospeed(&tio, B115200);
                tcsetattr(fd_, TCSANOW, &tio);
                tcflush(fd_, TCIOFLUSH);
            }

            std::cerr << "usb_cdc: opened " << device_ << std::endl;
            reassembly_.reset();
            anomaly_pending = false;
        }

        // Read raw bytes
        uint8_t raw[4096];
        ssize_t n = ::read(fd_, raw, sizeof(raw));
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            std::cerr << "usb_cdc: read error, reopening..." << std::endl;
            ::close(fd_);
            fd_ = -1;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }
        if (n == 0)
            continue;

        buf.insert(buf.end(), raw, raw + n);

        // Guard against runaway buffer (bad data with no valid sync)
        if (buf.size() > 2 * 1024 * 1024) {  // 2 MB sanity cap
            std::cerr << "usb_cdc: input buffer overflow (" << buf.size()
                      << "), first 64 bytes: ";
            for (size_t i = 0; i < 64 && i < buf.size(); ++i)
                std::cerr << std::hex << (int)buf[i] << " ";
            std::cerr << std::dec << "..." << std::endl;
            // Capacity stays allocated (pre-reserved), no realloc needed
            buf.clear();
            reassembly_.reset();
            continue;
        }

        // Process frames: search for 0xAA55 magic
        while (buf.size() >= sizeof(UsbPacketHeader)) {
            // Find magic
            auto magic_pos = buf.size();
            for (size_t i = 0; i <= buf.size() - 2; ++i) {
                if (buf[i] == 0x55 && buf[i + 1] == 0xAA) {
                    magic_pos = i;
                    break;
                }
            }

            if (magic_pos >= buf.size()) {
                buf.clear();
                break;
            }

            // Discard bytes before magic
            if (magic_pos > 0)
                buf.erase(buf.begin(), buf.begin() + magic_pos);

            if (buf.size() < sizeof(UsbPacketHeader))
                break;

            // Parse header
            UsbPacketHeader hdr;
            if (!parseHeader(buf.data(), buf.size(), hdr)) {
                buf.erase(buf.begin());
                continue;
            }

            // Sanity-check header to prevent bad_alloc from garbage data
            if (hdr.type == USB_PKT_TYPE_IMAGE) {
                if (hdr.width == 0 || hdr.height == 0 ||
                    hdr.width > 640 || hdr.height > 480 ||
                    hdr.packet_total == 0 ||
                    hdr.packet_total > 2048) {
                    buf.erase(buf.begin());
                    continue;
                }
            }

            // Diagnostic: print first 10 parsed headers
            static int diag_count = 0;
            if (diag_count++ < 10) {
                std::cerr << "usb_cdc: pkt type=" << (int)hdr.type
                          << " event=" << (int)hdr.event
                          << " w=" << hdr.width << " h=" << hdr.height
                          << " id=" << hdr.packet_id
                          << " total=" << hdr.packet_total
                          << " plen=" << hdr.payload_len
                          << std::endl;
            }

            size_t packet_size = sizeof(UsbPacketHeader) + hdr.payload_len;
            if (buf.size() < packet_size)
                break;  // wait for more data

            const uint8_t *payload = buf.data() + sizeof(UsbPacketHeader);

            if (hdr.type == USB_PKT_TYPE_DET) {
                // ── Detection / Event packet ──
                switch (hdr.event) {
                case USB_DET_EVENT_STOP:
                    cmd_pub_.publishStop();
                    anomaly_pending = true;
                    break;

                case USB_DET_EVENT_X_DELTA:
                    if (hdr.payload_len >= sizeof(UsbDetObject)) {
                        UsbDetObject det;
                        std::memcpy(&det, payload, sizeof(det));
                        cmd_pub_.publishAdjust(det.x_angle_delta_deg);
                    }
                    anomaly_pending = false;
                    break;

                case USB_DET_EVENT_NONE:
                    // Normal detection result (no target or target info)
                    // Optional: forward payload for CarView2 if needed
                    anomaly_pending = false;
                    break;

                case USB_DET_EVENT_FLOW_END:
                    // Laser flow complete → resume patrol
                    cmd_pub_.publishResume();
                    anomaly_pending = false;
                    break;

                default:
                    break;
                }

                buf.erase(buf.begin(), buf.begin() + packet_size);
                continue;

            } else if (hdr.type == USB_PKT_TYPE_IMAGE) {
                // ── Image packet (slot-based reassembly) ──
                if (hdr.event != USB_IMG_EVENT_RGB565) {
                    buf.erase(buf.begin(), buf.begin() + packet_size);
                    continue;
                }

                // Start new reassembly if frame parameters changed
                // (GD32 sends chunks out of order: last chunk first, then 0..N-2)
                if (!reassembly_.active ||
                    hdr.width != reassembly_.width ||
                    hdr.height != reassembly_.height ||
                    hdr.packet_total != reassembly_.packet_total) {

                    if (reassembly_.active)
                        reassembly_errors++;
                    reassembly_.startFrame(hdr.width, hdr.height,
                                           hdr.packet_total);
                }

                // Bounds check against packet_total
                if (hdr.packet_id >= reassembly_.packet_total) {
                    buf.erase(buf.begin(), buf.begin() + packet_size);
                    continue;
                }

                // Store chunk in its slot (handles any arrival order)
                if (reassembly_.slots[hdr.packet_id].empty())
                    reassembly_.chunks_received++;
                reassembly_.slots[hdr.packet_id].assign(
                    payload, payload + hdr.payload_len);
                buf.erase(buf.begin(), buf.begin() + packet_size);

                // Check if all chunks received → frame complete
                size_t expected_size = (size_t)hdr.width * hdr.height * 2;
                if (reassembly_.allReceived()) {
                    auto frame_data = reassembly_.assemble();
                    if (frame_data.size() >= expected_size) {
                        pushRgb565Frame(reassembly_.width, reassembly_.height,
                                        frame_data.data(), expected_size,
                                        anomaly_pending ? nullptr : nullptr);
                        frames_received++;
                    } else {
                        reassembly_errors++;
                    }
                    reassembly_.reset();
                }
            } else {
                // Unknown type → skip
                buf.erase(buf.begin(), buf.begin() + packet_size);
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "usb_cdc: exception: " << e.what()
                  << " buf=" << buf.size()
                  << std::endl;
        buf.clear();
        reassembly_.reset();
    }
}

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    std::cerr << "usb_cdc: stopped" << std::endl;
}

} // namespace gd32_bridge
