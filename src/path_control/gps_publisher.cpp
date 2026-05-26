#include "gps_publisher.h"

#include <cstring>
#include <cerrno>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>

// Packed struct matching gd32_bridge::GpsState
#pragma pack(push, 1)
struct GpsStatePacked {
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

GpsPublisher::GpsPublisher() = default;
GpsPublisher::~GpsPublisher() { close(); }

bool GpsPublisher::open(const std::string &socket_path) {
    socket_path_ = socket_path;

    fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd_ < 0) {
        std::cerr << "gps_publisher: socket() failed: " << strerror(errno) << std::endl;
        return false;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

    if (::connect(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        // Bridge not ready yet
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Non-blocking
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    return true;
}

bool GpsPublisher::publish(const path_control::GnssPosition &fix,
                            const path_control::RobotState &state,
                            double timestamp) {
    // Try to (re)connect if not connected
    if (fd_ < 0) {
        open(socket_path_);
        if (fd_ < 0)
            return false;
    }

    GpsStatePacked pkt;
    pkt.lat_deg          = fix.position.lat_deg;
    pkt.lon_deg          = fix.position.lon_deg;
    pkt.height_m         = fix.has_height ? fix.height_m : 0.0;
    pkt.course_deg       = fix.has_course ? fix.course_deg : 0.0;
    pkt.speed_mps        = fix.has_speed ? fix.speed_mps : 0.0;
    pkt.yaw_rad          = state.yaw_rad;
    pkt.forward_speed_mps = state.forward_speed_mps;
    pkt.timestamp        = timestamp;
    pkt.has_fix          = 1;

    ssize_t n = ::write(fd_, &pkt, sizeof(pkt));
    if (n < 0) {
        if (errno == EAGAIN || errno == EINTR)
            return true; // will retry next cycle
        // Connection lost
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    return true;
}

void GpsPublisher::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}
