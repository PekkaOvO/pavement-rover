#pragma once

#include <cstdint>
#include <cstddef>

namespace gd32_bridge {

// ============================================================================
// USB CDC protocol — GD32 firmware → i.MX6ULL
// ============================================================================

// USB packet header, 16 bytes
//   All multi-byte fields are little-endian (GD32 native).
struct UsbPacketHeader {
    uint16_t magic;        // 0xAA55 — sync magic
    uint8_t  type;         // 0x01=image, 0x02=detection/event
    uint8_t  event;        // image: format; detection: event type
    uint16_t width;        // image width (pixels)
    uint16_t height;       // image height (pixels)
    uint16_t packet_id;    // chunk index (0-based)
    uint16_t packet_total; // total chunk count
    uint16_t payload_len;  // bytes in payload
    uint16_t reserved;
} __attribute__((packed));

static_assert(sizeof(UsbPacketHeader) == 16, "UsbPacketHeader size unexpected");

// USB packet types
constexpr uint8_t USB_PKT_TYPE_IMAGE = 0x01;
constexpr uint8_t USB_PKT_TYPE_DET   = 0x02;

// Image event / format
constexpr uint8_t USB_IMG_EVENT_RGB565 = 0x01;

// Detection / flow event types
constexpr uint8_t USB_DET_EVENT_NONE     = 0x00; // empty / no target
constexpr uint8_t USB_DET_EVENT_STOP     = 0x01; // stop vehicle
constexpr uint8_t USB_DET_EVENT_X_DELTA  = 0x02; // gimbal X angle delta valid
constexpr uint8_t USB_DET_EVENT_FLOW_END = 0x03; // single crack flow end

// Detection / flow object payload, 28 bytes
//   Used when: type=0x02, event=USB_DET_EVENT_X_DELTA
//   x_angle_delta_deg = current X servo angle - GIMBAL_X_INIT_ANGLE (80.0°)
//       negative → servo is left of center
//       0        → servo at center
//       positive → servo is right of center
#pragma pack(push, 1)
struct UsbDetObject {
    uint16_t cls_index;
    uint16_t object_id;
    uint16_t conf_q10000;
    uint16_t reserved;
    char     name[16];
    float    x_angle_delta_deg;
};
#pragma pack(pop)

static_assert(sizeof(UsbDetObject) == 28, "UsbDetObject size unexpected");

// ============================================================================
// Frame queue markers (first byte of queued frame discriminator)
// ============================================================================

constexpr uint8_t QUEUE_FRAME_RGB565      = 0x01; // raw RGB565 image
constexpr uint8_t QUEUE_FRAME_RGB565_DET = 0x03; // RGB565 + embedded UsbDetObject(28B)
constexpr uint8_t QUEUE_FRAME_RGB565_ANOM = 0x04; // anomaly RGB565 + det object

// ============================================================================
// TCP protocol — i.MX6ULL → CarView2 (unchanged, kept compatible)
// ============================================================================

// Fixed 52-byte header + optional payload + CRC16(2)
// | MAGIC(2) | VERSION(1) | TYPE(1) | SEQ(4) | TS_MS(8) |
// | LAT(8)   | LON(8)     | COURSE(4) | SPEED(4) | HEIGHT(4) |
// | CAM_ANGLE(4) | IMAGE_LEN(4) |
// Normal RGB565:  [IMAGE_DATA(IMAGE_LEN)] | CRC16(2)
// Anomaly RGB565: [UsbDetObject(28)] [IMAGE_DATA(IMAGE_LEN)] | CRC16(2)
struct TcpProtocol {
    static constexpr uint16_t MAGIC    = 0xAA55;
    static constexpr uint8_t  VERSION  = 0x01;
    static constexpr uint8_t  TYPE_WITH_IMAGE     = 0x01;
    static constexpr uint8_t  TYPE_NO_IMAGE       = 0x02;
    static constexpr uint8_t  TYPE_RGB565         = 0x03;
    static constexpr uint8_t  TYPE_RGB565_ANOMALY = 0x04;

    static constexpr size_t HEADER_SIZE = 52;
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
    // IMAGE_DATA starts at offset 52
    // CRC16 at end of frame
};

// ============================================================================
// GPS state — published via Unix domain socket
// ============================================================================

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
    uint8_t has_fix;
};
#pragma pack(pop)

static_assert(sizeof(GpsState) == 65, "GpsState size unexpected");

// ============================================================================
// Legacy aliases (kept for backward compatibility during transition)
// ============================================================================

// Old 20-byte header — no longer used by GD32 firmware V2.2.0+
// Keep the define available so old code still compiles.
struct [[deprecated("Use UsbPacketHeader instead")]] CdcPacketHeader {
    uint16_t magic;
    uint8_t  type;
    uint8_t  format;
    uint32_t frame_id;
    uint16_t width;
    uint16_t height;
    uint16_t chunk_id;
    uint16_t chunk_total;
    uint16_t payload_len;
    uint16_t reserved;
} __attribute__((packed));

using CdcDetObjectV2 [[deprecated("Use UsbDetObject instead")]] = UsbDetObject;

// ============================================================================
// Gd32Frame — USB-TTL UART frame format (legacy path, kept for completeness)
// ============================================================================
//
// Frame format:
//   [MAGIC(2)][TYPE(1)][PAYLOAD_LEN(4)][PAYLOAD(VAR)][CRC16(2)]
//
//   MAGIC:       0xAA55 (big-endian for sync search)
//   TYPE:        0x01 = image + angle (last 4 bytes of payload)
//                0x02 = angle only
//   PAYLOAD_LEN: big-endian uint32
//   CRC16:       CCITT poly 0x1021, covers header + payload
struct Gd32Frame {
    static constexpr size_t HEADER_SIZE = 7;    // MAGIC(2) + TYPE(1) + LEN(4)
    static constexpr size_t CRC_SIZE    = 2;

    static constexpr uint8_t TYPE_IMAGE_ANGLE = 0x01; // payload = image + angle(4B)
    static constexpr uint8_t TYPE_ANGLE_ONLY  = 0x02; // payload = angle(4B)

    // Max payload: 256x256 RGB565 (131072) + angle(4) + margin
    static constexpr uint32_t MAX_PAYLOAD = 200000;
};

} // namespace gd32_bridge
