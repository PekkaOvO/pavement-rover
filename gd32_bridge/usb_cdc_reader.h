#pragma once

#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <cstdint>
#include <map>

#include "frame_queue.h"
#include "protocol.h"

namespace gd32_bridge {

// Reads RGB565 image data from GD32 via USB CDC ACM (/dev/ttyACM0).
//
// Protocol format (from GD32):
//   [CdcPacketHeader (20B)] [chunk payload (payload_len bytes)]
//
// Reassembles chunks into a complete RGB565 frame, wraps with metadata,
// and pushes to FrameQueue. TcpSender detects QUEUE_FRAME_RGB565
// and sends raw RGB565 over TCP with TYPE_RGB565.
class UsbCdcReader {
public:
    explicit UsbCdcReader(FrameQueue &queue,
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

    // Parse a CDC packet header from raw bytes. Returns true on success.
    static bool parseHeader(const uint8_t *data, size_t len,
                            CdcPacketHeader &hdr);

    // Push a raw RGB565 frame to the queue (with metadata header).
    void pushRgb565Frame(uint32_t frame_id,
                         uint16_t width, uint16_t height,
                         const uint8_t *rgb565, size_t rgb565_len,
                         const CdcDetObjectV2 *detection = nullptr);

    void cacheDetection(uint32_t frame_id, const uint8_t *payload, size_t payload_len);

    // Per-frame reassembly state
    struct ReassemblyBuffer {
        uint32_t frame_id = 0;
        uint16_t width = 0;
        uint16_t height = 0;
        uint16_t chunk_total = 0;
        std::vector<uint8_t> data;
        bool active = false;
    };

    FrameQueue &queue_;
    std::string device_;
    int fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};
    ReassemblyBuffer reassembly_;
    std::map<uint32_t, CdcDetObjectV2> detections_;
};

} // namespace gd32_bridge
