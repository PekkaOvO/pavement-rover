#include "vehicle_cmd_listener.h"

#include <cstring>
#include <cerrno>
#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>

namespace path_control {

constexpr const char *VEHICLE_CMD_SOCK_PATH = "/tmp/gd32_vehicle_cmd.sock";

VehicleCmdListener::VehicleCmdListener() {}

VehicleCmdListener::~VehicleCmdListener() { stop(); }

bool VehicleCmdListener::start() {
    if (fd_ >= 0)
        return true;
    return createSocket();
}

void VehicleCmdListener::stop() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    ::unlink(VEHICLE_CMD_SOCK_PATH);
}

bool VehicleCmdListener::createSocket() {
    // Remove any stale socket file
    ::unlink(VEHICLE_CMD_SOCK_PATH);

    fd_ = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        std::cerr << "vehicle_cmd_listener: socket() failed: "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    // Bind to the well-known socket path
    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, VEHICLE_CMD_SOCK_PATH,
            sizeof(addr.sun_path) - 1);

    if (::bind(fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "vehicle_cmd_listener: bind(" << VEHICLE_CMD_SOCK_PATH
                  << ") failed: " << std::strerror(errno) << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Set non-blocking
    int flags = ::fcntl(fd_, F_GETFL, 0);
    if (flags >= 0)
        ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

    std::cerr << "vehicle_cmd_listener: listening on "
              << VEHICLE_CMD_SOCK_PATH << std::endl;
    return true;
}

bool VehicleCmdListener::tryRecv(VehicleCmdMessage &msg) {
    if (fd_ < 0)
        return false;

    ssize_t n = ::recv(fd_, &msg, sizeof(msg), MSG_DONTWAIT);
    if (n == sizeof(msg))
        return true;

    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        // Socket error — recreate
        std::cerr << "vehicle_cmd_listener: recv error: "
                  << std::strerror(errno) << ", recreating socket..."
                  << std::endl;
        stop();
    }
    return false;
}

} // namespace path_control
