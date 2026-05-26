#pragma once

#include <thread>
#include <atomic>
#include <string>
#include <vector>

#include "protocol.h"
#include "frame_queue.h"

namespace gd32_bridge {

// Reads raw bytes from USB-TTL (/dev/ttyUSB0), performs frame
// synchronization and CRC validation, pushes complete frames
// into the shared FrameQueue.
class UartReader {
public:
    UartReader(FrameQueue &queue, const std::string &device = "/dev/ttyUSB0");
    ~UartReader();

    bool start();
    void stop();

    // Status counters (readable by main thread)
    std::atomic<uint64_t> packets_received{0};
    std::atomic<uint64_t> crc_errors{0};
    std::atomic<uint64_t> frames_dropped{0};

private:
    void readLoop();
    bool configureTermios(int fd);
    ssize_t readRaw(int fd, uint8_t *buf, size_t count);
    uint16_t crc16Ccitt(const uint8_t *data, size_t len);

    FrameQueue &queue_;
    std::string device_;
    int fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};
};

} // namespace gd32_bridge
