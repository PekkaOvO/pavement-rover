#include "gps_receiver.h"

#include <cstring>
#include <cerrno>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>

namespace gd32_bridge {

GpsReceiver::GpsReceiver() {
    memset(&state_, 0, sizeof(state_));
}

GpsReceiver::~GpsReceiver() { stop(); }

bool GpsReceiver::start(const std::string &socket_path) {
    if (running_.exchange(true))
        return false;
    socket_path_ = socket_path;
    thread_ = std::thread(&GpsReceiver::runLoop, this);
    return true;
}

void GpsReceiver::stop() {
    running_ = false;
    if (listen_fd_ >= 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    if (thread_.joinable())
        thread_.join();
}

GpsState GpsReceiver::latest() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void GpsReceiver::runLoop() {
    // Remove any stale socket file
    ::unlink(socket_path_.c_str());

    listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        std::cerr << "gps_receiver: socket() failed" << std::endl;
        return;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);

    if (::bind(listen_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "gps_receiver: bind() failed: " << strerror(errno) << std::endl;
        ::close(listen_fd_);
        listen_fd_ = -1;
        return;
    }

    // Allow gnss_path_control (running as same user) to connect
    ::chmod(socket_path_.c_str(), 0666);

    if (::listen(listen_fd_, 1) < 0) {
        std::cerr << "gps_receiver: listen() failed" << std::endl;
        ::close(listen_fd_);
        listen_fd_ = -1;
        return;
    }

    std::cerr << "gps_receiver: listening on " << socket_path_ << std::endl;

    int conn_fd = -1;
    while (running_) {
        if (conn_fd < 0) {
            struct sockaddr_un client_addr;
            socklen_t client_len = sizeof(client_addr);
            conn_fd = ::accept(listen_fd_, (struct sockaddr *)&client_addr, &client_len);
            if (conn_fd < 0) {
                if (errno == EINTR) continue;
                std::cerr << "gps_receiver: accept() failed" << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            // Non-blocking for the connection
            int flags = fcntl(conn_fd, F_GETFL, 0);
            fcntl(conn_fd, F_SETFL, flags | O_NONBLOCK);
            std::cerr << "gps_receiver: gps publisher connected" << std::endl;
        }

        GpsState state;
        ssize_t n = ::read(conn_fd, &state, sizeof(state));
        if (n > 0) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = state;
            has_state_ = true;
        } else if (n == 0) {
            // Client disconnected
            std::cerr << "gps_receiver: gps publisher disconnected" << std::endl;
            ::close(conn_fd);
            conn_fd = -1;
        } else if (errno != EAGAIN && errno != EINTR) {
            ::close(conn_fd);
            conn_fd = -1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (conn_fd >= 0) {
        ::close(conn_fd);
    }
    if (listen_fd_ >= 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
    ::unlink(socket_path_.c_str());
    std::cerr << "gps_receiver: stopped" << std::endl;
}

} // namespace gd32_bridge
