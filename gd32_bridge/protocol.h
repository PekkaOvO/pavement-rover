#pragma once

#include <cstdint>
#include <cstddef>

namespace gd32_bridge {

// GD32 USB-TTL frame (GD32 -> i.MX6ULL)
struct Gd32Frame {
    static constexpr uint16_t MAGIC = 0xAA55;
    static constexpr uint8_t TYPE_IMAGE_ANGLE = 0x01;
    static constexpr uint8_t TYPE_ANGLE_ONLY  = 0x02;

    static constexpr size_t MAGIC_SIZE  = 2;
    static constexpr size_t TYPE_SIZE   = 1;
    static constexpr size_t LEN_SIZE    = 4;
    static constexpr size_t CRC_SIZE    = 2;
    static constexpr size_t HEADER_SIZE = MAGIC_SIZE + TYPE_SIZE + LEN_SIZE; // 7

    // JPEG max 128KB + trailing 4-byte float angle
    static constexpr size_t MAX_PAYLOAD     = 128U * 1024U + 4U;
    static constexpr size_t MAX_FRAME_SIZE  = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE;

    static constexpr size_t ANGLE_ONLY_PAYLOAD = 4; // 4-byte float
};

// TCP binary protocol (i.MX6ULL -> CarView2)
// Fixed 50-byte header + optional image payload + CRC16(2)
//
// | MAGIC(2) | VERSION(1) | TYPE(1) | SEQ(4) | TS_MS(8) |
// | LAT(8)   | LON(8)     | COURSE(4) | SPEED(4) | HEIGHT(4) |
// | CAM_ANGLE(4) | IMAGE_LEN(4) | [IMAGE_DATA(IMAGE_LEN)] | CRC16(2) |
struct TcpProtocol {
    static constexpr uint16_t MAGIC    = 0xAA55;
    static constexpr uint8_t  VERSION  = 0x01;
    static constexpr uint8_t  TYPE_WITH_IMAGE = 0x01;
    static constexpr uint8_t  TYPE_NO_IMAGE   = 0x02;

    static constexpr size_t HEADER_SIZE = 50;
    static constexpr size_t CRC_SIZE    =  2;

    static constexpr size_t MAGIC_OFF     = 0;  // 2B
    static constexpr size_t VERSION_OFF   = 2;  // 1B
    static constexpr size_t TYPE_OFF      = 3;  // 1B
    static constexpr size_t SEQ_OFF       = 4;  // 4B
    static constexpr size_t TS_MS_OFF     = 8;  // 8B
    static constexpr size_t LAT_OFF       = 16; // 8B
    static constexpr size_t LON_OFF       = 24; // 8B
    static constexpr size_t COURSE_OFF    = 32; // 4B
    static constexpr size_t SPEED_OFF     = 36; // 4B
    static constexpr size_t HEIGHT_OFF    = 40; // 4B
    static constexpr size_t CAM_ANGLE_OFF = 44; // 4B
    static constexpr size_t IMAGE_LEN_OFF = 48; // 4B
    // IMAGE_DATA starts at offset 50
    // CRC16 at end of frame
};

// GPS state published via Unix domain socket (gnss_path_control -> gd32_bridge)
#pragma pack(push, 1)
struct GpsState {
    double lat_deg;
    double lon_deg;
    double height_m;
    double course_deg;
    double speed_mps;
    double yaw_rad;
    double forward_speed_mps;
    double timestamp;
    uint8_t has_fix; // bool
};
#pragma pack(pop)

static_assert(sizeof(GpsState) == 65, "GpsState size unexpected");

} // namespace gd32_bridge
