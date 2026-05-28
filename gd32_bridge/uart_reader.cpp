#include "uart_reader.h"

#include <chrono>
#include <cstring>
#include <cerrno>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

namespace gd32_bridge {

UartReader::UartReader(FrameQueue &queue, const std::string &device)
    : queue_(queue), device_(device) {}

UartReader::~UartReader() { stop(); }

bool UartReader::start() {
    if (running_.exchange(true))
        return false;
    thread_ = std::thread(&UartReader::readLoop, this);
    return true;
}

void UartReader::stop() {
    running_ = false;
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    if (thread_.joinable())
        thread_.join();
}

// CRC16-CCITT (poly 0x1021, init 0xFFFF)
uint16_t UartReader::crc16Ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

bool UartReader::configureTermios(int fd) {
    struct termios tio;
    if (tcgetattr(fd, &tio) < 0)
        return false;

    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cc[VMIN]  = 1;  // block until at least 1 byte
    tio.c_cc[VTIME] = 0;  // no timeout

    // 921600 baud
    cfsetispeed(&tio, B921600);
    cfsetospeed(&tio, B921600);

    if (tcsetattr(fd, TCSANOW, &tio) < 0)
        return false;

    // Flush stale data
    tcflush(fd, TCIOFLUSH);
    return true;
}

ssize_t UartReader::readRaw(int fd, uint8_t *buf, size_t count) {
    ssize_t n = ::read(fd, buf, count);
    if (n < 0 && (errno == EAGAIN || errno == EINTR))
        return 0;
    return n;
}

void UartReader::readLoop() {
    while (running_) {
        // Open device (retry every 2s if not available)
        if (fd_ < 0) {
            fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd_ < 0) {
                std::cerr << "uart_reader: cannot open " << device_
                          << ", retrying in 2s..." << std::endl;
                for (int i = 0; i < 20 && running_; ++i)
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            // Set exclusive lock
            if (ioctl(fd_, TIOCEXCL) < 0) {
                std::cerr << "uart_reader: TIOCEXCL failed" << std::endl;
                ::close(fd_);
                fd_ = -1;
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }

            if (!configureTermios(fd_)) {
                std::cerr << "uart_reader: termios config failed" << std::endl;
                ::close(fd_);
                fd_ = -1;
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }

            std::cerr << "uart_reader: opened " << device_ << std::endl;
        }

        // Read raw bytes into scratch buffer
        uint8_t raw[4096];
        ssize_t n = readRaw(fd_, raw, sizeof(raw));
        if (n <= 0) {
            if (n < 0 && errno != EAGAIN) {
                // Device error, close and retry
                std::cerr << "uart_reader: read error, reopening..." << std::endl;
                ::close(fd_);
                fd_ = -1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Process the raw bytes for frame synchronization
        // We accumulate in a persistent buffer to handle partial frames
        static std::vector<uint8_t> buf;
        buf.insert(buf.end(), raw, raw + n);

        // Try to extract frames from the buffer
        while (buf.size() >= Gd32Frame::HEADER_SIZE + Gd32Frame::CRC_SIZE) {
            // Search for MAGIC
            auto magic_pos = buf.size();
            for (size_t i = 0; i <= buf.size() - 2; ++i) {
                if (buf[i] == 0xAA && buf[i + 1] == 0x55) {
                    magic_pos = i;
                    break;
                }
            }

            if (magic_pos >= buf.size()) {
                // No MAGIC found, discard all
                buf.clear();
                break;
            }

            // Discard bytes before MAGIC
            if (magic_pos > 0) {
                buf.erase(buf.begin(), buf.begin() + magic_pos);
            }

            if (buf.size() < Gd32Frame::HEADER_SIZE + Gd32Frame::CRC_SIZE)
                break;

            // Parse length (big-endian uint32)
            uint32_t payload_len = ((uint32_t)buf[3] << 24)
                                 | ((uint32_t)buf[4] << 16)
                                 | ((uint32_t)buf[5] << 8)
                                 | (uint32_t)buf[6];

            if (payload_len > Gd32Frame::MAX_PAYLOAD) {
                // Invalid length, skip this byte and re-sync
                buf.erase(buf.begin());
                frames_dropped++;
                continue;
            }

            size_t frame_size = Gd32Frame::HEADER_SIZE + payload_len + Gd32Frame::CRC_SIZE;
            if (buf.size() < frame_size)
                break; // wait for more data

            // Validate CRC16 (covers MAGIC through PAYLOAD)
            uint16_t crc_received = (uint16_t)buf[frame_size - 2] << 8
                                  | (uint16_t)buf[frame_size - 1];
            uint16_t crc_calc = crc16Ccitt(buf.data(), frame_size - 2);

            if (crc_received != crc_calc) {
                crc_errors++;
                buf.erase(buf.begin()); // skip one byte and re-sync
                continue;
            }

            // Valid frame: push to queue
            std::vector<uint8_t> frame(buf.begin(), buf.begin() + frame_size);
            buf.erase(buf.begin(), buf.begin() + frame_size);

            if (!queue_.push(frame.data(), frame.size())) {
                frames_dropped++;
            } else {
                packets_received++;
            }
        }
    }

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    std::cerr << "uart_reader: stopped" << std::endl;
}

} // namespace gd32_bridge
