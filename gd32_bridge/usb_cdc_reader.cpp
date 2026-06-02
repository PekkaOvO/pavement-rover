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
    std::vector<uint8_t> buf;
    buf.reserve(512 * 1024);  // 512 KB

    // Track anomaly state: after STOP event, mark subsequent images as anomaly
    // until detection ends or an X_DELTA event is processed.
    bool anomaly_pending = false;

    while (running_) {
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

            struct termios tio;
            if (tcgetattr(fd_, &tio) == 0) {
                cfmakeraw(&tio);
                tio.c_cc[VMIN]  = 1;
                tio.c_cc[VTIME] = 0;
                tcsetattr(fd_, TCSANOW, &tio);
                tcflush(fd_, TCIOFLUSH);
            }

            std::cerr << "usb_cdc: opened " << device_ << std::endl;
            reassembly_.active = false;
            reassembly_.data.clear();
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
                // ── Image packet ──
                if (hdr.event != USB_IMG_EVENT_RGB565) {
                    buf.erase(buf.begin(), buf.begin() + packet_size);
                    continue;
                }

                // Start new reassembly if packet_id restarted
                if (!reassembly_.active ||
                    hdr.packet_id < reassembly_.last_packet_id) {
                    reassembly_.width        = hdr.width;
                    reassembly_.height       = hdr.height;
                    reassembly_.packet_total = hdr.packet_total;
                    reassembly_.data.clear();
                    reassembly_.active       = true;
                }
                reassembly_.last_packet_id = hdr.packet_id;

                reassembly_.data.insert(reassembly_.data.end(),
                                        payload, payload + hdr.payload_len);
                buf.erase(buf.begin(), buf.begin() + packet_size);

                // Check if frame is complete
                size_t expected_size = (size_t)hdr.width * hdr.height * 2;
                if (hdr.packet_id + 1 >= hdr.packet_total &&
                    reassembly_.data.size() >= expected_size) {

                    // If anomaly is pending, attach detection (without det obj
                    // we just mark it; if we had cached det data we'd attach it)
                    // For now, anomaly_pending just marks the frame type.
                    const UsbDetObject *det = nullptr;
                    // In future: look up cached detection if needed.

                    pushRgb565Frame(reassembly_.width, reassembly_.height,
                                    reassembly_.data.data(), expected_size,
                                    anomaly_pending ? det : nullptr);
                    frames_received++;

                    reassembly_.active = false;
                    reassembly_.data.clear();
                } else if (hdr.packet_id + 1 >= hdr.packet_total) {
                    // Last chunk but size mismatch → discard
                    reassembly_errors++;
                    reassembly_.active = false;
                    reassembly_.data.clear();
                }
            } else {
                // Unknown type → skip
                buf.erase(buf.begin(), buf.begin() + packet_size);
            }
        }
    }

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    std::cerr << "usb_cdc: stopped" << std::endl;
}

} // namespace gd32_bridge
