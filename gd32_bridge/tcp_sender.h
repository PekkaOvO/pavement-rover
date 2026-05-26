#pragma once

#include <thread>
#include <atomic>
#include <string>

#include "protocol.h"
#include "frame_queue.h"
#include "gps_receiver.h"

namespace gd32_bridge {

// Reads frames from the shared queue, packs them with GPS state
// into the TCP binary protocol, and sends to CarView2.
// Auto-reconnects with exponential backoff on disconnect.
class TcpSender {
public:
    TcpSender(FrameQueue &queue, GpsReceiver &gps,
              const std::string &host = "192.168.1.100",
              uint16_t port = 8766);
    ~TcpSender();

    bool start();
    void stop();

private:
    void sendLoop();
    bool connectSocket();
    void packFrame(const std::vector<uint8_t> &gd32_frame,
                   uint32_t seq, const GpsState &gps,
                   std::vector<uint8_t> &tcp_pkt);
    uint16_t crc16Ccitt(const uint8_t *data, size_t len);

    FrameQueue &queue_;
    GpsReceiver &gps_;
    std::string host_;
    uint16_t port_;
    int fd_ = -1;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<uint32_t> seq_{0};
};

} // namespace gd32_bridge
