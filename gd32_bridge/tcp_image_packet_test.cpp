#include "tcp_image_test_packet.h"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>

namespace {

uint32_t readBeU32(const std::vector<uint8_t> &bytes, size_t offset) {
    return ((uint32_t)bytes[offset] << 24) |
           ((uint32_t)bytes[offset + 1] << 16) |
           ((uint32_t)bytes[offset + 2] << 8) |
           (uint32_t)bytes[offset + 3];
}

uint16_t readLeU16(const std::vector<uint8_t> &bytes, size_t offset) {
    return (uint16_t)bytes[offset] | ((uint16_t)bytes[offset + 1] << 8);
}

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

bool require(bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << std::endl;
        return false;
    }
    return true;
}

gd32_bridge::CdcDetObjectV2 makeDetection() {
    gd32_bridge::CdcDetObjectV2 detection;
    std::memset(&detection, 0, sizeof(detection));
    detection.cls_index = 3;
    detection.object_id = 1;
    detection.conf_q10000 = 9123;
    std::strncpy(detection.name, "road_crack", sizeof(detection.name) - 1);
    detection.x_angle_delta_deg = 15.5f;
    return detection;
}

} // namespace

int main() {
    const uint16_t width = 256;
    const uint16_t height = 256;
    const std::vector<uint8_t> packet =
        gd32_bridge::buildTcpImageTestPacket(7, width, height);

    bool ok = true;
    ok &= require(packet.size() == gd32_bridge::TcpProtocol::HEADER_SIZE + width * height * 2 + gd32_bridge::TcpProtocol::CRC_SIZE,
                  "packet size should be header + RGB565 image + CRC");
    ok &= require(packet[0] == 0xAA && packet[1] == 0x55, "magic should be AA55");
    ok &= require(packet[gd32_bridge::TcpProtocol::VERSION_OFF] == gd32_bridge::TcpProtocol::VERSION,
                  "protocol version should match");
    ok &= require(packet[gd32_bridge::TcpProtocol::TYPE_OFF] == gd32_bridge::TcpProtocol::TYPE_RGB565,
                  "packet type should be RGB565");
    ok &= require(readBeU32(packet, gd32_bridge::TcpProtocol::SEQ_OFF) == 7,
                  "sequence should be encoded big-endian");
    ok &= require(readBeU32(packet, gd32_bridge::TcpProtocol::IMAGE_LEN_OFF) == width * height * 2,
                  "image length should be encoded big-endian");

    const uint16_t crcCalc = crc16Ccitt(packet.data() + gd32_bridge::TcpProtocol::VERSION_OFF,
                                        packet.size() - gd32_bridge::TcpProtocol::VERSION_OFF - gd32_bridge::TcpProtocol::CRC_SIZE);
    const uint16_t crcFrame = ((uint16_t)packet[packet.size() - 2] << 8) | packet[packet.size() - 1];
    ok &= require(crcCalc == crcFrame, "CRC should cover VERSION through image data");

    const gd32_bridge::CdcDetObjectV2 detection = makeDetection();
    const std::vector<uint8_t> anomalyPacket =
        gd32_bridge::buildTcpImageTestPacket(8, width, height, &detection);
    const size_t imageLen = (size_t)width * height * 2U;
    const size_t metaOff = gd32_bridge::TcpProtocol::HEADER_SIZE;
    const size_t imageOff = gd32_bridge::TcpProtocol::HEADER_SIZE + sizeof(gd32_bridge::CdcDetObjectV2);
    ok &= require(anomalyPacket.size() == imageOff + imageLen + gd32_bridge::TcpProtocol::CRC_SIZE,
                  "anomaly packet size should include DET_V2 metadata");
    ok &= require(anomalyPacket[gd32_bridge::TcpProtocol::TYPE_OFF] == gd32_bridge::TcpProtocol::TYPE_RGB565_ANOMALY,
                  "anomaly packet type should be TYPE_RGB565_ANOMALY");
    ok &= require(readBeU32(anomalyPacket, gd32_bridge::TcpProtocol::SEQ_OFF) == 8,
                  "anomaly sequence should be encoded big-endian");
    ok &= require(readBeU32(anomalyPacket, gd32_bridge::TcpProtocol::IMAGE_LEN_OFF) == imageLen,
                  "anomaly image length should still describe only image bytes");
    ok &= require(readLeU16(anomalyPacket, metaOff) == 3,
                  "anomaly metadata should carry class index");
    ok &= require(readLeU16(anomalyPacket, metaOff + 4) == 9123,
                  "anomaly metadata should carry confidence");
    ok &= require(std::strncmp((const char *)anomalyPacket.data() + metaOff + 8, "road_crack", 10) == 0,
                  "anomaly metadata should carry label name");

    const uint16_t anomalyCrcCalc = crc16Ccitt(
        anomalyPacket.data() + gd32_bridge::TcpProtocol::VERSION_OFF,
        anomalyPacket.size() - gd32_bridge::TcpProtocol::VERSION_OFF - gd32_bridge::TcpProtocol::CRC_SIZE);
    const uint16_t anomalyCrcFrame = ((uint16_t)anomalyPacket[anomalyPacket.size() - 2] << 8) |
                                     anomalyPacket[anomalyPacket.size() - 1];
    ok &= require(anomalyCrcCalc == anomalyCrcFrame,
                  "anomaly CRC should cover VERSION through metadata and image data");

    return ok ? 0 : 1;
}
