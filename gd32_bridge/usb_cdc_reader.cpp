#include "usb_cdc_reader.h"

#include <cstring>
#include <cerrno>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

namespace gd32_bridge {

UsbCdcReader::UsbCdcReader(FrameQueue &queue, const std::string &device)
    : queue_(queue), device_(device) {}

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
// Header parsing
// ---------------------------------------------------------------------------
bool UsbCdcReader::parseHeader(const uint8_t *data, size_t len,
                                CdcPacketHeader &hdr) {
    if (len < sizeof(CdcPacketHeader))
        return false;

    // All multi-byte fields are little-endian (GD32 native)
    hdr.magic       = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    hdr.type        = data[2];
    hdr.format      = data[3];
    hdr.frame_id    = (uint32_t)data[4]
                    | ((uint32_t)data[5] << 8)
                    | ((uint32_t)data[6] << 16)
                    | ((uint32_t)data[7] << 24);
    hdr.width       = (uint16_t)data[8]  | ((uint16_t)data[9] << 8);
    hdr.height      = (uint16_t)data[10] | ((uint16_t)data[11] << 8);
    hdr.chunk_id    = (uint16_t)data[12] | ((uint16_t)data[13] << 8);
    hdr.chunk_total = (uint16_t)data[14] | ((uint16_t)data[15] << 8);
    hdr.payload_len = (uint16_t)data[16] | ((uint16_t)data[17] << 8);
    hdr.reserved    = (uint16_t)data[18] | ((uint16_t)data[19] << 8);

    return hdr.magic == 0xAA55;
}

// ---------------------------------------------------------------------------
// Push raw RGB565 to queue
// Queue frame format: [QUEUE_FRAME_RGB565(1B)][FRAME_ID(4B,LE)]
//                     [WIDTH(2B,LE)][HEIGHT(2B,LE)][RGB565_DATA(W*H*2)]
// ---------------------------------------------------------------------------
void UsbCdcReader::pushRgb565Frame(uint32_t frame_id,
                                    uint16_t width, uint16_t height,
                                    const uint8_t *rgb565, size_t rgb565_len,
                                    const CdcDetObjectV2 *detection) {
    const size_t det_len = detection ? sizeof(CdcDetObjectV2) : 0;
    std::vector<uint8_t> frame;
    frame.reserve(1 + 4 + 2 + 2 + det_len + rgb565_len);

    frame.push_back(detection ? QUEUE_FRAME_RGB565_DET : QUEUE_FRAME_RGB565);
    // frame_id (little-endian)
    frame.push_back((uint8_t)(frame_id));
    frame.push_back((uint8_t)(frame_id >> 8));
    frame.push_back((uint8_t)(frame_id >> 16));
    frame.push_back((uint8_t)(frame_id >> 24));
    // width (little-endian)
    frame.push_back((uint8_t)(width));
    frame.push_back((uint8_t)(width >> 8));
    // height (little-endian)
    frame.push_back((uint8_t)(height));
    frame.push_back((uint8_t)(height >> 8));
    if (detection) {
        const uint8_t *det_bytes = reinterpret_cast<const uint8_t *>(detection);
        frame.insert(frame.end(), det_bytes, det_bytes + sizeof(CdcDetObjectV2));
    }
    // RGB565 data
    frame.insert(frame.end(), rgb565, rgb565 + rgb565_len);

    if (!queue_.push(frame.data(), frame.size()))
        frames_dropped++;
}

void UsbCdcReader::cacheDetection(uint32_t frame_id, const uint8_t *payload, size_t payload_len) {
    if (payload_len < sizeof(CdcDetObjectV2))
        return;

    CdcDetObjectV2 detection;
    std::memcpy(&detection, payload, sizeof(detection));
    detections_[frame_id] = detection;

    while (detections_.size() > 32) {
        detections_.erase(detections_.begin());
    }
}

// ---------------------------------------------------------------------------
// Main read loop
// ---------------------------------------------------------------------------
void UsbCdcReader::readLoop() {
    // Persistent buffer for raw bytes from the device
    std::vector<uint8_t> buf;
    buf.reserve(512 * 1024);  // 512 KB

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

            // Set exclusive lock
            ioctl(fd_, TIOCEXCL);

            // CDC ACM devices don't use real baud rate, but we need
            // raw mode to avoid TTY line-discipline processing.
            struct termios tio;
            if (tcgetattr(fd_, &tio) == 0) {
                cfmakeraw(&tio);
                tio.c_cc[VMIN]  = 1;
                tio.c_cc[VTIME] = 0;
                tcsetattr(fd_, TCSANOW, &tio);
                tcflush(fd_, TCIOFLUSH);
            }

            std::cerr << "usb_cdc: opened " << device_ << std::endl;
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
        while (buf.size() >= sizeof(CdcPacketHeader)) {
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

            if (buf.size() < sizeof(CdcPacketHeader))
                break;

            // Parse header
            CdcPacketHeader hdr;
            if (!parseHeader(buf.data(), buf.size(), hdr)) {
                buf.erase(buf.begin());
                continue;
            }

            size_t packet_size = sizeof(CdcPacketHeader) + hdr.payload_len;
            if (buf.size() < packet_size)
                break;  // wait for more data

            // Image packets: process below
            if (hdr.type == CDC_PKT_TYPE_IMAGE) {
                // fall through to image handling
            } else if (hdr.type == CDC_PKT_TYPE_DET && hdr.format == CDC_DET_FMT_V2) {
                cacheDetection(hdr.frame_id,
                               buf.data() + sizeof(CdcPacketHeader),
                               hdr.payload_len);
                buf.erase(buf.begin(), buf.begin() + packet_size);
                continue;
            } else if (hdr.type == 0x03 && hdr.format == 0x01) {
                // Status packet — log the text payload
                const uint8_t *text = buf.data() + sizeof(CdcPacketHeader);
                uint16_t text_len = hdr.payload_len;
                if (text_len > 0) {
                    std::string status(text, text + text_len);
                    std::cerr << "usb_cdc: STATUS: " << status << std::endl;
                }
                buf.erase(buf.begin(), buf.begin() + packet_size);
                continue;
            } else {
                // Detection (0x02) or unknown — skip
                buf.erase(buf.begin(), buf.begin() + packet_size);
                continue;
            }

            if (hdr.format != CDC_IMG_FMT_RGB565) {
                buf.erase(buf.begin(), buf.begin() + packet_size);
                continue;
            }

            // Chunk data
            const uint8_t *chunk_data = buf.data() + sizeof(CdcPacketHeader);

            // Start new reassembly if frame_id changed
            if (!reassembly_.active || hdr.frame_id != reassembly_.frame_id) {
                reassembly_.frame_id     = hdr.frame_id;
                reassembly_.width        = hdr.width;
                reassembly_.height       = hdr.height;
                reassembly_.chunk_total  = hdr.chunk_total;
                reassembly_.data.clear();
                reassembly_.active       = true;
            }

            // Append chunk data
            reassembly_.data.insert(reassembly_.data.end(),
                                     chunk_data, chunk_data + hdr.payload_len);

            buf.erase(buf.begin(), buf.begin() + packet_size);

            // Check if frame is complete
            size_t expected_size = (size_t)hdr.width * hdr.height * 2;
            if (hdr.chunk_id + 1 >= hdr.chunk_total &&
                reassembly_.data.size() >= expected_size) {
                const CdcDetObjectV2 *detection = nullptr;
                auto detectionIt = detections_.find(reassembly_.frame_id);
                if (detectionIt != detections_.end()) {
                    detection = &detectionIt->second;
                }

                // Complete frame — push raw RGB565 to queue
                pushRgb565Frame(reassembly_.frame_id,
                                reassembly_.width,
                                reassembly_.height,
                                reassembly_.data.data(),
                                expected_size,
                                detection);
                if (detectionIt != detections_.end()) {
                    detections_.erase(detectionIt);
                }
                frames_received++;

                reassembly_.active = false;
                reassembly_.data.clear();
            } else if (hdr.chunk_id + 1 >= hdr.chunk_total) {
                // Last chunk but data size mismatch — discard
                reassembly_errors++;
                reassembly_.active = false;
                reassembly_.data.clear();
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
