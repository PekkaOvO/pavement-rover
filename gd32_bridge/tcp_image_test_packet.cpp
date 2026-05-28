#include "tcp_image_test_packet.h"

#include <chrono>
#include <cstring>

namespace gd32_bridge {
namespace {

uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void writeBeU32(std::vector<uint8_t> &bytes, size_t offset, uint32_t value) {
    bytes[offset] = (uint8_t)(value >> 24);
    bytes[offset + 1] = (uint8_t)(value >> 16);
    bytes[offset + 2] = (uint8_t)(value >> 8);
    bytes[offset + 3] = (uint8_t)value;
}

void writeBeU64(std::vector<uint8_t> &bytes, size_t offset, uint64_t value) {
    for (int i = 0; i < 8; ++i)
        bytes[offset + i] = (uint8_t)(value >> (56 - i * 8));
}

void writeBeFloat(std::vector<uint8_t> &bytes, size_t offset, float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    writeBeU32(bytes, offset, bits);
}

void writeBeDouble(std::vector<uint8_t> &bytes, size_t offset, double value) {
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    writeBeU64(bytes, offset, bits);
}

uint16_t makeRgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

} // namespace

std::vector<uint8_t> buildTcpImageTestPacket(uint32_t sequence,
                                             uint16_t width,
                                             uint16_t height,
                                             const CdcDetObjectV2 *detection) {
    const uint32_t imageLen = (uint32_t)width * (uint32_t)height * 2U;
    const size_t detectionLen = detection ? sizeof(CdcDetObjectV2) : 0U;
    std::vector<uint8_t> packet(TcpProtocol::HEADER_SIZE + detectionLen + imageLen + TcpProtocol::CRC_SIZE, 0);

    packet[TcpProtocol::MAGIC_OFF] = 0xAA;
    packet[TcpProtocol::MAGIC_OFF + 1] = 0x55;
    packet[TcpProtocol::VERSION_OFF] = TcpProtocol::VERSION;
    packet[TcpProtocol::TYPE_OFF] = detection ? TcpProtocol::TYPE_RGB565_ANOMALY : TcpProtocol::TYPE_RGB565;
    writeBeU32(packet, TcpProtocol::SEQ_OFF, sequence);

    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const uint64_t tsMs = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    writeBeU64(packet, TcpProtocol::TS_MS_OFF, tsMs);

    writeBeDouble(packet, TcpProtocol::LAT_OFF, 39.9042);
    writeBeDouble(packet, TcpProtocol::LON_OFF, 116.4074);
    writeBeFloat(packet, TcpProtocol::COURSE_OFF, (float)((sequence * 7U) % 360U));
    writeBeFloat(packet, TcpProtocol::SPEED_OFF, 0.0f);
    writeBeFloat(packet, TcpProtocol::HEIGHT_OFF, 50.0f);
    writeBeFloat(packet, TcpProtocol::CAM_ANGLE_OFF, (float)((sequence * 3U) % 90U));
    writeBeU32(packet, TcpProtocol::IMAGE_LEN_OFF, imageLen);

    uint8_t *image = packet.data() + TcpProtocol::HEADER_SIZE;
    if (detection) {
        std::memcpy(image, detection, sizeof(CdcDetObjectV2));
        image += sizeof(CdcDetObjectV2);
    }

    for (uint16_t y = 0; y < height; ++y) {
        for (uint16_t x = 0; x < width; ++x) {
            const uint8_t r = (uint8_t)((x + sequence * 3U) & 0xFFU);
            const uint8_t g = (uint8_t)((y + sequence * 5U) & 0xFFU);
            const uint8_t b = (uint8_t)(((x ^ y) + sequence * 11U) & 0xFFU);
            const uint16_t rgb565 = makeRgb565(r, g, b);
            const size_t pos = ((size_t)y * width + x) * 2U;
            image[pos] = (uint8_t)(rgb565 & 0xFFU);
            image[pos + 1] = (uint8_t)(rgb565 >> 8);
        }
    }

    const uint16_t crc = crc16Ccitt(packet.data() + TcpProtocol::VERSION_OFF,
                                    TcpProtocol::HEADER_SIZE - TcpProtocol::VERSION_OFF + detectionLen + imageLen);
    packet[packet.size() - 2] = (uint8_t)(crc >> 8);
    packet[packet.size() - 1] = (uint8_t)crc;
    return packet;
}

} // namespace gd32_bridge
