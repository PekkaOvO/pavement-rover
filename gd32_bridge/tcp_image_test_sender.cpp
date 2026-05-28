#include "tcp_image_test_packet.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

std::atomic<bool> g_stop{false};

void onSignal(int) {
    g_stop = true;
}

void printUsage(const char *prog) {
    std::cerr << "Usage: " << prog << " <server_host> [server_port] [frame_count] [interval_ms] [mode]\n"
              << "  server_host: Windows or WSL-reachable CarView2 IP address\n"
              << "  server_port: CarView2 TCP port, default 8766\n"
              << "  frame_count: 0 means send until Ctrl+C, default 0\n"
              << "  interval_ms: default 500\n"
              << "  mode: normal, anomaly, or mixed; default normal\n";
}

int connectToServer(const std::string &host, uint16_t port) {
    int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        std::cerr << "socket() failed: " << std::strerror(errno) << std::endl;
        return -1;
    }

    int flag = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        std::cerr << "invalid IPv4 address: " << host << std::endl;
        ::close(fd);
        return -1;
    }

    if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        std::cerr << "connect(" << host << ":" << port << ") failed: "
                  << std::strerror(errno) << std::endl;
        ::close(fd);
        return -1;
    }

    return fd;
}

bool writeAll(int fd, const std::vector<uint8_t> &packet) {
    size_t total = 0;
    while (total < packet.size()) {
        const ssize_t n = ::write(fd, packet.data() + total, packet.size() - total);
        if (n > 0) {
            total += (size_t)n;
            continue;
        }
        if (n < 0 && errno == EINTR)
            continue;
        std::cerr << "write failed: " << std::strerror(errno) << std::endl;
        return false;
    }
    return true;
}

gd32_bridge::CdcDetObjectV2 makeDetection(uint32_t sequence) {
    gd32_bridge::CdcDetObjectV2 detection;
    std::memset(&detection, 0, sizeof(detection));
    detection.cls_index = (uint16_t)(sequence % 4U);
    detection.object_id = 1U;
    detection.conf_q10000 = (uint16_t)(8500U + (sequence % 1000U));
    std::strncpy(detection.name, "road_crack", sizeof(detection.name) - 1);
    detection.x_angle_delta_deg = (float)((int)(sequence % 31U) - 15);
    return detection;
}

bool shouldSendDetection(const std::string &mode, uint32_t sequence) {
    if (mode == "anomaly")
        return true;
    if (mode == "mixed")
        return (sequence % 3U) == 0U;
    return false;
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    const std::string host = argv[1];
    const uint16_t port = (argc >= 3) ? (uint16_t)std::max(1, std::atoi(argv[2])) : 8766;
    const uint32_t frameCount = (argc >= 4) ? (uint32_t)std::max(0, std::atoi(argv[3])) : 0;
    const int intervalMs = (argc >= 5) ? std::max(1, std::atoi(argv[4])) : 500;
    const std::string mode = (argc >= 6) ? argv[5] : "normal";
    if (mode != "normal" && mode != "anomaly" && mode != "mixed") {
        std::cerr << "invalid mode: " << mode << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    std::cerr << "tcp_image_test_sender:\n"
              << "  server:   " << host << ":" << port << "\n"
              << "  image:    256x256 RGB565\n"
              << "  frames:   " << (frameCount == 0 ? std::string("until Ctrl+C") : std::to_string(frameCount)) << "\n"
              << "  interval: " << intervalMs << " ms\n"
              << "  mode:     " << mode << "\n";

    int fd = connectToServer(host, port);
    if (fd < 0)
        return 2;

    uint32_t seq = 0;
    while (!g_stop && (frameCount == 0 || seq < frameCount)) {
        gd32_bridge::CdcDetObjectV2 detection;
        const gd32_bridge::CdcDetObjectV2 *detectionPtr = nullptr;
        if (shouldSendDetection(mode, seq)) {
            detection = makeDetection(seq);
            detectionPtr = &detection;
        }
        const std::vector<uint8_t> packet = gd32_bridge::buildTcpImageTestPacket(seq, 256, 256, detectionPtr);
        if (!writeAll(fd, packet)) {
            ::close(fd);
            return 3;
        }

        std::cerr << "sent frame seq=" << seq
                  << " type=" << (detectionPtr ? "anomaly" : "normal")
                  << " bytes=" << packet.size() << std::endl;
        ++seq;
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
    }

    ::close(fd);
    std::cerr << "done" << std::endl;
    return 0;
}
