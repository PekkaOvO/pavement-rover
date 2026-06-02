#include "vehicle_cmd_publisher.h"

#include <cstring>
#include <cerrno>
#include <chrono>
#include <iostream>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

namespace gd32_bridge {

VehicleCmdPublisher::VehicleCmdPublisher() {}

VehicleCmdPublisher::~VehicleCmdPublisher() { stop(); }

bool VehicleCmdPublisher::start() {
    if (sock_fd_ >= 0)
        return true;
    return initSocket();
}

void VehicleCmdPublisher::stop() {
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
}

bool VehicleCmdPublisher::initSocket() {
    sock_fd_ = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sock_fd_ < 0) {
        std::cerr << "vehicle_cmd: socket() failed: " << std::strerror(errno)
                  << std::endl;
        return false;
    }
    return true;
}

void VehicleCmdPublisher::publish(const VehicleCmdMessage &msg) {
    if (sock_fd_ < 0 && !initSocket())
        return;

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, VEHICLE_CMD_SOCK_PATH,
            sizeof(addr.sun_path) - 1);

    ssize_t n = ::sendto(sock_fd_, &msg, sizeof(msg), MSG_DONTWAIT,
                         (struct sockaddr *)&addr, sizeof(addr));
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK
             && errno != ECONNREFUSED && errno != ENOENT) {
        std::cerr << "vehicle_cmd: sendto failed: " << std::strerror(errno)
                  << std::endl;
    }
}

void VehicleCmdPublisher::publishStop() {
    VehicleCmdMessage msg;
    msg.cmd = CMD_STOP;
    msg.angle_delta_deg = 0.0f;
    msg.timestamp_ms = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    publish(msg);
    std::cerr << "vehicle_cmd: CMD_STOP" << std::endl;
}

void VehicleCmdPublisher::publishAdjust(float angle_delta_deg) {
    VehicleCmdMessage msg;
    msg.cmd = CMD_ADJUST;
    msg.angle_delta_deg = angle_delta_deg;
    msg.timestamp_ms = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    publish(msg);
    std::cerr << "vehicle_cmd: CMD_ADJUST angle=" << angle_delta_deg << "°"
              << std::endl;
}

void VehicleCmdPublisher::publishResume() {
    VehicleCmdMessage msg;
    msg.cmd = CMD_RESUME;
    msg.angle_delta_deg = 0.0f;
    msg.timestamp_ms = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    publish(msg);
    std::cerr << "vehicle_cmd: CMD_RESUME" << std::endl;
}

} // namespace gd32_bridge
