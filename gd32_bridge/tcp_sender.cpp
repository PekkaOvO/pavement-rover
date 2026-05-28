#include "tcp_sender.h"

#include <cstring>
#include <cerrno>
#include <chrono>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

namespace gd32_bridge {

TcpSender::TcpSender(FrameQueue &queue, GpsReceiver &gps,
                     const std::string &host, uint16_t port)
    : queue_(queue), gps_(gps), host_(host), port_(port) {}

TcpSender::~TcpSender() { stop(); }

bool TcpSender::start() {
    if (running_.exchange(true))
        return false;
    thread_ = std::thread(&TcpSender::sendLoop, this);
    return true;
}

void TcpSender::stop() {
    running_ = false;
    queue_.shutdown();
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    if (thread_.joinable())
        thread_.join();
}

uint16_t TcpSender::crc16Ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void TcpSender::packFrame(const std::vector<uint8_t> &gd32_frame,
                          uint32_t seq, const GpsState &gps,
                          std::vector<uint8_t> &pkt) {
    // Determine type and image data
    uint8_t type = TcpProtocol::TYPE_NO_IMAGE;
    const uint8_t *image_data = nullptr;
    uint32_t image_len = 0;
    const uint8_t *detection_data = nullptr;
    uint32_t detection_len = 0;
    float cam_angle = 0.0f;

    if (gd32_frame.empty())
        goto build;

    // Check for raw RGB565 frame (from USB CDC path)
    if (gd32_frame[0] == QUEUE_FRAME_RGB565 && gd32_frame.size() >= 9) {
        // Frame format: [TYPE(1)][FRAME_ID(4)][WIDTH(2)][HEIGHT(2)][RGB565_DATA]
        uint16_t width  = (uint16_t)gd32_frame[5] | ((uint16_t)gd32_frame[6] << 8);
        uint16_t height = (uint16_t)gd32_frame[7] | ((uint16_t)gd32_frame[8] << 8);
        image_len = (uint32_t)width * height * 2;

        if (gd32_frame.size() >= 9 + image_len) {
            type = TcpProtocol::TYPE_RGB565;
            image_data = gd32_frame.data() + 9;
        }
        goto build;
    }

    if (gd32_frame[0] == QUEUE_FRAME_RGB565_DET && gd32_frame.size() >= 9 + sizeof(CdcDetObjectV2)) {
        uint16_t width  = (uint16_t)gd32_frame[5] | ((uint16_t)gd32_frame[6] << 8);
        uint16_t height = (uint16_t)gd32_frame[7] | ((uint16_t)gd32_frame[8] << 8);
        image_len = (uint32_t)width * height * 2;

        if (gd32_frame.size() >= 9 + sizeof(CdcDetObjectV2) + image_len) {
            type = TcpProtocol::TYPE_RGB565_ANOMALY;
            detection_data = gd32_frame.data() + 9;
            detection_len = sizeof(CdcDetObjectV2);
            image_data = gd32_frame.data() + 9 + sizeof(CdcDetObjectV2);
        }
        goto build;
    }

    // Gd32Frame format (from USB-TTL UART path)
    if (gd32_frame.size() >= Gd32Frame::HEADER_SIZE + 2) {
        uint8_t frame_type = gd32_frame[2]; // TYPE byte
        // Payload length (big-endian)
        uint32_t payload_len = ((uint32_t)gd32_frame[3] << 24)
                             | ((uint32_t)gd32_frame[4] << 16)
                             | ((uint32_t)gd32_frame[5] << 8)
                             | (uint32_t)gd32_frame[6];

        if (frame_type == Gd32Frame::TYPE_IMAGE_ANGLE && payload_len >= 4) {
            type = TcpProtocol::TYPE_WITH_IMAGE;
            image_len = payload_len - 4; // last 4 bytes are angle
            image_data = gd32_frame.data() + Gd32Frame::HEADER_SIZE;

            // Extract camera angle (last 4 bytes of payload, little-endian float from GD32)
            const uint8_t *angle_bytes = gd32_frame.data() + Gd32Frame::HEADER_SIZE + image_len;
            memcpy(&cam_angle, angle_bytes, sizeof(cam_angle));
        } else if (frame_type == Gd32Frame::TYPE_ANGLE_ONLY && payload_len >= 4) {
            type = TcpProtocol::TYPE_NO_IMAGE;
            memcpy(&cam_angle, gd32_frame.data() + Gd32Frame::HEADER_SIZE, sizeof(cam_angle));
        }
    }

build:

    // Build TCP packet: header + [image] + crc16
    pkt.resize(TcpProtocol::HEADER_SIZE + detection_len + image_len + TcpProtocol::CRC_SIZE);
    uint8_t *hdr = pkt.data();

    // MAGIC
    hdr[TcpProtocol::MAGIC_OFF]     = 0xAA;
    hdr[TcpProtocol::MAGIC_OFF + 1] = 0x55;
    // VERSION
    hdr[TcpProtocol::VERSION_OFF] = TcpProtocol::VERSION;
    // TYPE
    hdr[TcpProtocol::TYPE_OFF] = type;
    // SEQ (big-endian)
    hdr[TcpProtocol::SEQ_OFF]     = (uint8_t)(seq >> 24);
    hdr[TcpProtocol::SEQ_OFF + 1] = (uint8_t)(seq >> 16);
    hdr[TcpProtocol::SEQ_OFF + 2] = (uint8_t)(seq >> 8);
    hdr[TcpProtocol::SEQ_OFF + 3] = (uint8_t)(seq);
    // TS_MS (big-endian)
    uint64_t ts_ms = (uint64_t)(gps.timestamp * 1000.0);
    for (int i = 0; i < 8; ++i)
        hdr[TcpProtocol::TS_MS_OFF + i] = (uint8_t)(ts_ms >> (56 - i * 8));
    // LAT (big-endian)
    uint64_t lat_bits;
    memcpy(&lat_bits, &gps.lat_deg, sizeof(lat_bits));
    for (int i = 0; i < 8; ++i)
        hdr[TcpProtocol::LAT_OFF + i] = (uint8_t)(lat_bits >> (56 - i * 8));
    // LON (big-endian)
    uint64_t lon_bits;
    memcpy(&lon_bits, &gps.lon_deg, sizeof(lon_bits));
    for (int i = 0; i < 8; ++i)
        hdr[TcpProtocol::LON_OFF + i] = (uint8_t)(lon_bits >> (56 - i * 8));
    // COURSE (big-endian float -> 4 bytes, reinterpret as uint32)
    float course_f = (float)gps.course_deg;
    uint32_t course_bits;
    memcpy(&course_bits, &course_f, sizeof(course_bits));
    hdr[TcpProtocol::COURSE_OFF]     = (uint8_t)(course_bits >> 24);
    hdr[TcpProtocol::COURSE_OFF + 1] = (uint8_t)(course_bits >> 16);
    hdr[TcpProtocol::COURSE_OFF + 2] = (uint8_t)(course_bits >> 8);
    hdr[TcpProtocol::COURSE_OFF + 3] = (uint8_t)(course_bits);
    // SPEED (big-endian float)
    float speed_f = (float)gps.speed_mps;
    uint32_t speed_bits;
    memcpy(&speed_bits, &speed_f, sizeof(speed_bits));
    hdr[TcpProtocol::SPEED_OFF]     = (uint8_t)(speed_bits >> 24);
    hdr[TcpProtocol::SPEED_OFF + 1] = (uint8_t)(speed_bits >> 16);
    hdr[TcpProtocol::SPEED_OFF + 2] = (uint8_t)(speed_bits >> 8);
    hdr[TcpProtocol::SPEED_OFF + 3] = (uint8_t)(speed_bits);
    // HEIGHT (big-endian float)
    float height_f = (float)gps.height_m;
    uint32_t height_bits;
    memcpy(&height_bits, &height_f, sizeof(height_bits));
    hdr[TcpProtocol::HEIGHT_OFF]     = (uint8_t)(height_bits >> 24);
    hdr[TcpProtocol::HEIGHT_OFF + 1] = (uint8_t)(height_bits >> 16);
    hdr[TcpProtocol::HEIGHT_OFF + 2] = (uint8_t)(height_bits >> 8);
    hdr[TcpProtocol::HEIGHT_OFF + 3] = (uint8_t)(height_bits);
    // CAM_ANGLE (big-endian float)
    uint32_t angle_bits;
    memcpy(&angle_bits, &cam_angle, sizeof(angle_bits));
    hdr[TcpProtocol::CAM_ANGLE_OFF]     = (uint8_t)(angle_bits >> 24);
    hdr[TcpProtocol::CAM_ANGLE_OFF + 1] = (uint8_t)(angle_bits >> 16);
    hdr[TcpProtocol::CAM_ANGLE_OFF + 2] = (uint8_t)(angle_bits >> 8);
    hdr[TcpProtocol::CAM_ANGLE_OFF + 3] = (uint8_t)(angle_bits);
    // IMAGE_LEN (big-endian)
    hdr[TcpProtocol::IMAGE_LEN_OFF]     = (uint8_t)(image_len >> 24);
    hdr[TcpProtocol::IMAGE_LEN_OFF + 1] = (uint8_t)(image_len >> 16);
    hdr[TcpProtocol::IMAGE_LEN_OFF + 2] = (uint8_t)(image_len >> 8);
    hdr[TcpProtocol::IMAGE_LEN_OFF + 3] = (uint8_t)(image_len);

    uint8_t *payload = hdr + TcpProtocol::HEADER_SIZE;
    if (detection_len > 0 && detection_data) {
        memcpy(payload, detection_data, detection_len);
        payload += detection_len;
    }

    // Image data (if any)
    if (image_len > 0 && image_data) {
        memcpy(payload, image_data, image_len);
    }

    // CRC16 (covers VERSION byte through image data)
    uint16_t crc = crc16Ccitt(hdr + TcpProtocol::VERSION_OFF,
                              TcpProtocol::HEADER_SIZE - TcpProtocol::VERSION_OFF + detection_len + image_len);
    pkt[pkt.size() - 2] = (uint8_t)(crc >> 8);
    pkt[pkt.size() - 1] = (uint8_t)(crc);
}

bool TcpSender::connectSocket() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }

    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ < 0)
        return false;

    // Enable TCP_NODELAY to reduce latency
    int flag = 1;
    ::setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);

    if (::connect(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

void TcpSender::sendLoop() {
    unsigned int retry_delay = 1; // seconds, exponential backoff

    while (running_) {
        if (fd_ < 0) {
            std::cerr << "tcp_sender: connecting to " << host_ << ":" << port_ << "..."
                      << std::endl;
            if (!connectSocket()) {
                std::cerr << "tcp_sender: connect failed, retry in " << retry_delay << "s"
                          << std::endl;
                for (unsigned int i = 0; i < retry_delay * 10 && running_; ++i)
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (retry_delay < 30)
                    retry_delay *= 2;
                continue;
            }
            retry_delay = 1;
            std::cerr << "tcp_sender: connected" << std::endl;
        }

        // Blocking pop from the frame queue
        std::vector<uint8_t> gd32_frame;
        if (!queue_.pop(gd32_frame)) {
            // Shutdown requested or queue closed
            break;
        }

        // Get latest GPS state
        GpsState gps = gps_.latest();

        // Pack into TCP protocol
        std::vector<uint8_t> tcp_pkt;
        uint32_t seq = seq_++;
        packFrame(gd32_frame, seq, gps, tcp_pkt);

        // Send
        ssize_t total_sent = 0;
        while (total_sent < (ssize_t)tcp_pkt.size()) {
            ssize_t n = ::write(fd_, tcp_pkt.data() + total_sent,
                                tcp_pkt.size() - total_sent);
            if (n > 0) {
                total_sent += n;
            } else if (n == 0 || (n < 0 && errno != EINTR)) {
                // Connection lost
                std::cerr << "tcp_sender: connection lost" << std::endl;
                ::close(fd_);
                fd_ = -1;
                break;
            }
        }
    }

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    std::cerr << "tcp_sender: stopped" << std::endl;
}

} // namespace gd32_bridge
