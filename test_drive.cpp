// test_drive.cpp - 直线行驶测试, 不需要 GPS
// 编译: arm-linux-gnueabihf-g++ -std=c++11 -O2 test_drive.cpp -o bin/test_drive
// 运行: ./test_drive [--speed 40] [--duration 3] [--right-scale 0.48]
//
// Ctrl+C 安全停车

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <csignal>

#include <fcntl.h>
#include <unistd.h>

static int motor_fd = -1;

static void onSignal(int) {
    if (motor_fd >= 0) {
        ::write(motor_fd, "stop\n", 5);
        ::close(motor_fd);
        motor_fd = -1;
    }
    std::cerr << "\ntest_drive: interrupted, stopped" << std::endl;
    _exit(1);
}

static void usage(const char *prog) {
    std::cerr << "Usage: " << prog << " [options]\n"
              << "  --speed PCT      Motor speed percent, 0-100 (default 40)\n"
              << "  --duration SEC   Drive duration in seconds (default 3)\n"
              << "  --dev DEVICE     Device path (default /dev/tb6612)\n"
              << "  --right-scale S  Right motor scale (default 0.48)\n"
              << "  --dry-run        Print only, don't drive\n";
}

int main(int argc, char **argv) {
    int speed = 40;
    int duration_s = 3;
    std::string dev = "/dev/tb6612";
    double right_scale = 0.48;
    bool dry_run = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") { usage(argv[0]); return 0; }
        else if (arg == "--speed" && i+1 < argc) speed = std::atoi(argv[++i]);
        else if (arg == "--duration" && i+1 < argc) duration_s = std::atoi(argv[++i]);
        else if (arg == "--dev" && i+1 < argc) dev = argv[++i];
        else if (arg == "--right-scale" && i+1 < argc) right_scale = std::atof(argv[++i]);
        else if (arg == "--dry-run") dry_run = true;
        else { std::cerr << "Unknown: " << arg << std::endl; return 1; }
    }

    int left = speed;
    int right = static_cast<int>(speed * right_scale);

    std::cerr << "test_drive: " << speed << "% left, " << right << "% right"
              << " for " << duration_s << "s"
              << (dry_run ? " (dry-run)" : "")
              << " (Ctrl+C to stop)"
              << std::endl;

    if (!dry_run) {
        motor_fd = ::open(dev.c_str(), O_WRONLY);
        if (motor_fd < 0) {
            std::cerr << "Cannot open " << dev << ": " << std::strerror(errno) << std::endl;
            return 1;
        }

        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);

        char buf[64];
        int n = std::snprintf(buf, sizeof(buf), "set %d %d\n", left, right);
        ::write(motor_fd, buf, static_cast<size_t>(n));
        std::cerr << "Driving forward..." << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(duration_s));

        ::write(motor_fd, "stop\n", 5);
        ::close(motor_fd);
        motor_fd = -1;
    }

    std::cerr << "test_drive: finished" << std::endl;
    return 0;
}
