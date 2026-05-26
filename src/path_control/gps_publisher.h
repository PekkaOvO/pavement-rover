#pragma once

#include <string>
#include <atomic>

#include "path_controller.h"

// Publishes GnssPosition + RobotState to gd32_bridge via Unix domain socket.
// Connects to /tmp/gd32_gps.sock and sends a GpsState struct on each call.
class GpsPublisher {
public:
    GpsPublisher();
    ~GpsPublisher();

    // Open connection (retries silently if bridge not ready).
    bool open(const std::string &socket_path = "/tmp/gd32_gps.sock");

    // Publish latest state. Returns false if not connected.
    bool publish(const path_control::GnssPosition &fix,
                 const path_control::RobotState &state,
                 double timestamp);

    // Close connection.
    void close();

private:
    int fd_ = -1;
    std::string socket_path_;
};
