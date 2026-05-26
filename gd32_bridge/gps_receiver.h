#pragma once

#include <thread>
#include <atomic>
#include <string>

#include "protocol.h"

namespace gd32_bridge {

// Receives latest GPS state from gnss_path_control via Unix domain socket.
// Stores the latest state in an atomic-copy-friendly buffer (readers
// copy the GpsState under a spinlock).
class GpsReceiver {
public:
    GpsReceiver();
    ~GpsReceiver();

    bool start(const std::string &socket_path = "/tmp/gd32_gps.sock");
    void stop();

    // Returns the latest GPS state (thread-safe).
    GpsState latest() const;

private:
    void runLoop();

    std::string socket_path_;
    int listen_fd_ = -1;
    std::thread thread_;
    std::atomic<bool> running_{false};

    // Cached latest state
    mutable std::mutex mutex_;
    GpsState state_;
    bool has_state_ = false;
};

} // namespace gd32_bridge
