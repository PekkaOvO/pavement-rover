#include <csignal>
#include <atomic>
#include <iostream>
#include <thread>

#include "protocol.h"
#include "frame_queue.h"
#include "uart_reader.h"
#include "gps_receiver.h"
#include "tcp_sender.h"

namespace {

std::atomic<bool> g_stop{false};

void onSignal(int) {
    g_stop = true;
}

void printUsage(const char *prog) {
    std::cerr << "Usage: " << prog
              << " <server_host> [server_port]\n"
              << "  server_host: CarView2 IP address\n"
              << "  server_port: CarView2 TCP port (default 8766)\n"
              << "  UART device: /dev/ttyUSB0 (fixed)\n"
              << "  GPS socket:  /tmp/gd32_gps.sock (fixed)\n";
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::string host = argv[1];
    uint16_t port = 8766;
    if (argc >= 3)
        port = (uint16_t)std::max(1, std::atoi(argv[2]));

    std::cerr << "gd32_bridge starting:\n"
              << "  server: " << host << ":" << port << "\n"
              << "  uart:   /dev/ttyUSB0 @ 921600\n"
              << "  gps:    /tmp/gd32_gps.sock\n";

    gd32_bridge::FrameQueue queue(32);
    gd32_bridge::UartReader uart(queue, "/dev/ttyUSB0");
    gd32_bridge::GpsReceiver gps;
    gd32_bridge::TcpSender tcp(queue, gps, host, port);

    if (!gps.start()) {
        std::cerr << "Failed to start GPS receiver" << std::endl;
        return 1;
    }

    if (!uart.start()) {
        std::cerr << "Failed to start UART reader" << std::endl;
        gps.stop();
        return 1;
    }

    if (!tcp.start()) {
        std::cerr << "Failed to start TCP sender" << std::endl;
        uart.stop();
        gps.stop();
        return 1;
    }

    // Main thread: print stats periodically until stop
    while (!g_stop) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cerr << "stats: packets=" << uart.packets_received.load()
                  << " crc_err=" << uart.crc_errors.load()
                  << " dropped=" << uart.frames_dropped.load()
                  << " queued=" << queue.size()
                  << std::endl;
    }

    std::cerr << "Shutting down..." << std::endl;
    tcp.stop();
    uart.stop();
    gps.stop();
    std::cerr << "gd32_bridge stopped" << std::endl;
    return 0;
}
