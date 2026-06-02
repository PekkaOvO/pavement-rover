#pragma once

#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <cstdint>

#include "frame_queue.h"
#include "protocol.h"
#include "vehicle_cmd_publisher.h"

namespace gd32_bridge {

// Reads image data and detection events from GD32 via USB CDC ACM (/dev/ttyACM0).
//
// New protocol (GD32 firmware V2.2.0+, 16-byte header):
//   Image packet:  [UsbPacketHeader type=0x01] [RGB565 chunk]
//   Event packet:  [UsbPacketHeader type=0x02 event=STOP/XDETAIL] [optional payload]
//
// On STOP event:     publishes CMD_STOP via VehicleCmdPublisher
// On X_DELTA event:  publishes CMD_ADJUST with angle via VehicleCmdPublisher
class UsbCdcReader {
public:
    explicit UsbCdcReader(FrameQueue &queue,
                          VehicleCmdPublisher &cmd_pub,
                          const std::string &device = "/dev/ttyACM0");
    ~UsbCdcReader();

    bool start();
    void stop();

    // Statistics
    std::atomic<uint64_t> frames_received{0};
    std::atomic<uint64_t> frames_dropped{0};
    std::atomic<uint64_t> reassembly_errors{0};

private:
    void readLoop();

    // Parse a 16-byte USB packet header from raw bytes.
    static bool parseHeader(const uint8_t *data, size_t len,
                            UsbPacketHeader &hdr);

    // Push a raw RGB565 frame to the queue.
    void pushRgb565Frame(uint16_t width, uint16_t height,
                         const uint8_t *rgb565, size_t rgb565_len,
                         const UsbDetObject *detection = nullptr);

    // Per-frame reassembly state
    struct ReassemblyBuffer {
        uint16_t width = 0;
        uint16_t height = 0;
        uint16_t packet_total = 0;
        uint16_t last_packet_id = 0;
        std::vector<uint8_t> data;
        bool active = false;
    };

    FrameQueue &queue_;
    VehicleCmdPublisher &cmd_pub_;
    std::string device_;
    int fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};
    ReassemblyBuffer reassembly_;
};

} // namespace gd32_bridge
