#pragma once

#include <cstdint>
#include <string>

namespace path_control {

// Command types (mirrors gd32_bridge::VehicleCmdType)
enum class VehicleCmdType : uint8_t {
    NONE   = 0,
    STOP   = 1,
    ADJUST = 2,
    RESUME = 3,
};

// Message structure (must match gd32_bridge::VehicleCmdMessage)
#pragma pack(push, 1)
struct VehicleCmdMessage {
    VehicleCmdType cmd;
    float          angle_delta_deg;
    uint32_t       timestamp_ms;
};
#pragma pack(pop)

// Listens for vehicle control commands from gd32_bridge via Unix DGRAM socket.
// Non-blocking receive; call tryRecv() periodically from the main loop.
class VehicleCmdListener {
public:
    VehicleCmdListener();
    ~VehicleCmdListener();

    bool start();
    void stop();

    // Non-blocking: returns true if a command was received.
    // Call repeatedly until it returns false to drain the socket.
    bool tryRecv(VehicleCmdMessage &msg);

    // File descriptor for use with poll()/select().
    int fd() const { return fd_; }

private:
    int  fd_ = -1;
    bool createSocket();
};

} // namespace path_control
