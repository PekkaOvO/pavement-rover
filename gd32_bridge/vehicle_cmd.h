#pragma once

#include <cstdint>

// ---------------------------------------------------------------------------
// Command protocol: gd32_bridge ──Unix DGRAM──► gnss_path_control
//
// gd32_bridge detects events from GD32 USB CDC and forwards them as
// structured commands to gnss_path_control via a Unix domain socket.
// ---------------------------------------------------------------------------

namespace gd32_bridge {

constexpr char VEHICLE_CMD_SOCK_PATH[] = "/tmp/gd32_vehicle_cmd.sock";

// Command types
enum VehicleCmdType : uint8_t {
    CMD_NONE   = 0,
    CMD_STOP   = 1,  // GD32 detected crack → stop vehicle immediately
    CMD_ADJUST = 2,  // GD32 gimbal centered → angle available
    CMD_RESUME = 3,  // Laser/crack processing done → resume patrol
};

// Message structure (Unix DGRAM, 9 bytes total)
#pragma pack(push, 1)
struct VehicleCmdMessage {
    VehicleCmdType cmd;           // 1B: command type
    float          angle_delta_deg; // 4B: gimbal angle delta (for CMD_ADJUST)
    uint32_t       timestamp_ms;    // 4B: local timestamp, for freshness check
};
#pragma pack(pop)

static_assert(sizeof(VehicleCmdMessage) == 9, "VehicleCmdMessage size unexpected");

} // namespace gd32_bridge
