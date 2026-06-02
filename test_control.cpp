// =============================================================================
// test_control – Full-flow test for crack handling:
//   normal driving → STOP → ADJUST (in-place rotate) → RESUME → continue
//
// Usage:
//   Interactive mode (type commands):
//     sudo ./KF-GINS-TestControl | sudo ./KF-GINS-GnssPathControl ./config.yaml
//
//   Auto mode (executes full sequence automatically):
//     sudo ./KF-GINS-TestControl --auto | sudo ./KF-GINS-GnssPathControl ./config.yaml
//
//   The program simulates GPS at lat=30.5 lon=114 heading east (90°),
//   matching the default config goal [30.5, 114.00005]. The car drives
//   forward automatically — just type commands to test crack handling.
//
// Commands (type in terminal while test_control runs):
//   stop              — send CMD_STOP   → car brakes
//   adjust <deg>      — send CMD_ADJUST → car rotates in place
//   resume            — send CMD_RESUME → car resumes path tracking
//   origin <lat> <lon>— reset GPS simulation position
//   heading <deg>     — set GPS travel direction (0=north, 90=east)
//   speed <m/s>       — set step distance
//   pause             — pause GPS output
//   cont              — continue GPS output
//   status            — print current GPS state
//   help              — show this help
//   quit              — exit
//
// Architecture:
//   ┌───────────────┐   pipe(stdout)   ┌────────────────────┐
//   │ test_control   │ ───────────────→│ gnss_path_control   │
//   │  GPS模拟线程   │    "lat lon\n"   │  stdin ← GPS Coords  │
//   │  auto测试线程  │                  │  /dev/tb6612 ← 电机 │
//   │  命令 ← stdin  │                  └─────────┬──────────┘
//   │   (键盘输入)   │                            ▲
//   └───────┬───────┘     Unix DGRAM socket       │
//           └─────────────────────────────────────┘
//                  /tmp/gd32_vehicle_cmd.sock
// =============================================================================

#include <cerrno>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/time.h>

// ---------------------------------------------------------------------------
// Vehicle command protocol (mirrors gd32_bridge/vehicle_cmd.h)
// ---------------------------------------------------------------------------
static constexpr const char *VEHICLE_CMD_SOCK_PATH = "/tmp/gd32_vehicle_cmd.sock";

enum class TestVehicleCmd : uint8_t {
    NONE   = 0,
    STOP   = 1,
    ADJUST = 2,
    RESUME = 3,
};

#pragma pack(push, 1)
struct TestVehicleCmdMsg {
    TestVehicleCmd cmd;
    float          angle_delta_deg;
    uint32_t       timestamp_ms;
};
#pragma pack(pop)

// ---------------------------------------------------------------------------
// GPS simulation state — defaults match config/goal (lat=30.5, lon=114, east)
// ---------------------------------------------------------------------------
static std::atomic<bool>  g_running{true};
static std::atomic<bool>  g_paused{false};
static double g_lat         = 30.5;
static double g_lon         = 114.0;
static double g_heading_deg = 90.0;          // 90° = east (toward config goal)
static double g_step_mps    = 0.3;            // metres per step
static int    g_interval_ms = 200;             // ms between GPS lines (5 Hz)

// ---------------------------------------------------------------------------
// Auto-test sequence — runs STOP → ADJUST 90° → RESUME on a timer
// ---------------------------------------------------------------------------
static std::atomic<bool> g_auto_mode{false};

// Forward declarations used by autoTestSequence
static bool sendVehicleCmd(TestVehicleCmd cmd, float angle = 0.0f);

static void autoTestSequence() {
    std::cerr << "\n[AUTO] Sequence: drive(3s) → STOP → wait(3s) → ADJUST 90°"
              << " → wait(5s) → RESUME\n" << std::endl;

    // Step 1: drive forward for 3 seconds
    std::this_thread::sleep_for(std::chrono::seconds(3));
    if (!g_running) return;

    // Step 2: STOP
    std::cerr << "\n[AUTO] +++ STOP +++\n" << std::endl;
    sendVehicleCmd(TestVehicleCmd::STOP);

    // Step 3: wait 3 seconds, then ADJUST 90
    std::this_thread::sleep_for(std::chrono::seconds(3));
    if (!g_running) return;

    std::cerr << "\n[AUTO] +++ ADJUST 90° +++\n" << std::endl;
    sendVehicleCmd(TestVehicleCmd::ADJUST, 90.0f);

    // Step 4: wait for rotation to complete (~2s) + margin, then RESUME
    std::this_thread::sleep_for(std::chrono::seconds(5));
    if (!g_running) return;

    std::cerr << "\n[AUTO] +++ RESUME +++\n" << std::endl;
    sendVehicleCmd(TestVehicleCmd::RESUME);

    std::cerr << "\n[AUTO] Sequence complete. Car continues driving."
              << "\n[AUTO] Type 'stop', 'adjust', 'resume' for more testing."
              << "\n[AUTO] Type 'quit' to exit.\n" << std::endl;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static uint32_t nowMs() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return static_cast<uint32_t>(tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

static bool sendVehicleCmd(TestVehicleCmd cmd, float angle) {
    int fd = ::socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "test_control: socket: " << std::strerror(errno) << std::endl;
        return false;
    }

    struct sockaddr_un addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, VEHICLE_CMD_SOCK_PATH, sizeof(addr.sun_path) - 1);

    TestVehicleCmdMsg msg;
    msg.cmd            = cmd;
    msg.angle_delta_deg = angle;
    msg.timestamp_ms   = nowMs();

    ssize_t n = ::sendto(fd, &msg, sizeof(msg), 0,
                         reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));

    ::close(fd);

    if (n != static_cast<ssize_t>(sizeof(msg))) {
        std::cerr << "test_control: sendto " << VEHICLE_CMD_SOCK_PATH << ": "
                  << std::strerror(errno) << std::endl;
        return false;
    }
    return true;
}

static void printHelp() {
    std::cerr <<
        "\n"
        "Commands:\n"
        "  stop               send CMD_STOP    → car brakes\n"
        "  adjust <deg>       send CMD_ADJUST  → car rotates in place\n"
        "  resume             send CMD_RESUME  → car resumes\n"
        "  origin <lat> <lon> reset GPS position\n"
        "  heading <deg>      set GPS travel direction (0=north, 90=east)\n"
        "  speed <m/s>        set GPS speed per step (default 0.5)\n"
        "  pause              pause GPS output\n"
        "  cont               continue GPS output\n"
        "  status             show current state\n"
        "  help               this help\n"
        "  quit               exit\n"
        "\n"
        "Auto mode (no typing needed):\n"
        "  sudo ./KF-GINS-TestControl --auto | sudo ./KF-GINS-GnssPathControl ...\n"
        "  Sequence: drive(3s) → STOP(3s) → ADJUST 90°(5s) → RESUME\n"
        << std::endl;
}

// ---------------------------------------------------------------------------
// GPS simulation thread — outputs "lat lon" lines to stdout at fixed rate
// ---------------------------------------------------------------------------
static void gpsThread() {
    while (g_running) {
        if (!g_paused) {
            std::cout << std::fixed << std::setprecision(8)
                      << g_lat << " " << g_lon << std::endl;

            // Advance GPS position along current heading
            //   1° lat ≈ 111111 m
            //   1° lon ≈ 111111 × cos(lat) m
            const double rad   = g_heading_deg * M_PI / 180.0;
            const double dlat  = g_step_mps * std::cos(rad) / 111111.0;
            const double dlon  = g_step_mps * std::sin(rad)
                               / (111111.0 * std::cos(g_lat * M_PI / 180.0));
            g_lat += dlat;
            g_lon += dlon;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(g_interval_ms));
    }
}

// ---------------------------------------------------------------------------
// Main — reads user commands from stdin, sends Unix socket messages
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
    // Parse args: <step_mps> or --auto
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--auto" || a == "-a") {
            g_auto_mode = true;
        } else {
            g_step_mps = std::max(0.01, std::atof(a.c_str()));
        }
    }

    std::cerr << "\n"
              << "=== Test Control for GNSS Path Control ===\n"
              << "  GPS: " << std::fixed << std::setprecision(6)
              << g_lat << ", " << g_lon
              << " heading " << g_heading_deg << "° (east)\n"
              << "  step: " << g_step_mps << " m  @ "
              << (1000 / g_interval_ms) << " Hz"
              << "  →  " << (g_step_mps * 1000.0 / g_interval_ms) << " m/s\n"
              << "  mode: " << (g_auto_mode ? "AUTO (no typing needed)" : "interactive")
              << "\n"
              << "Pipe to: sudo ./KF-GINS-GnssPathControl ./config.yaml\n"
              << "  sudo ./KF-GINS-TestControl" << (g_auto_mode ? " --auto" : "")
              << " | sudo ./KF-GINS-GnssPathControl ./config.yaml\n"
              << "---" << std::endl;
    printHelp();

    // Start background GPS simulator
    std::thread gps(gpsThread);

    // Auto mode: start sequence after a brief delay
    std::thread auto_seq;
    if (g_auto_mode) {
        auto_seq = std::thread(autoTestSequence);
    }

    // Read user commands from stdin (the terminal, even when stdout is piped)
    std::string line;
    while (g_running && std::getline(std::cin, line)) {
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;

        if (cmd == "quit" || cmd == "q" || cmd == "exit") {
            g_running = false;
            break;
        } else if (cmd == "stop") {
            std::cerr << "→ sending CMD_STOP ... ";
            if (sendVehicleCmd(TestVehicleCmd::STOP))
                std::cerr << "OK" << std::endl;
        } else if (cmd == "adjust") {
            float angle = 0.0f;
            iss >> angle;
            std::cerr << "→ sending CMD_ADJUST angle=" << angle << "° ... ";
            if (sendVehicleCmd(TestVehicleCmd::ADJUST, angle))
                std::cerr << "OK" << std::endl;
        } else if (cmd == "resume") {
            std::cerr << "→ sending CMD_RESUME ... ";
            if (sendVehicleCmd(TestVehicleCmd::RESUME))
                std::cerr << "OK" << std::endl;
        } else if (cmd == "origin" || cmd == "set") {
            double lat, lon;
            if (iss >> lat >> lon) {
                g_lat = lat;
                g_lon = lon;
            }
            std::cerr << "→ GPS origin set to "
                      << std::fixed << std::setprecision(6)
                      << g_lat << ", " << g_lon << std::endl;
        } else if (cmd == "heading") {
            iss >> g_heading_deg;
            std::cerr << "→ GPS heading set to " << g_heading_deg << "°" << std::endl;
        } else if (cmd == "speed") {
            iss >> g_step_mps;
            g_step_mps = std::max(0.01, g_step_mps);
            std::cerr << "→ GPS step set to " << g_step_mps << " m" << std::endl;
        } else if (cmd == "pause") {
            g_paused = true;
            std::cerr << "→ GPS output paused" << std::endl;
        } else if (cmd == "cont" || cmd == "continue") {
            g_paused = false;
            std::cerr << "→ GPS output continued" << std::endl;
        } else if (cmd == "status") {
            std::cerr << "Status:\n"
                      << "  position: " << std::fixed << std::setprecision(6)
                      << g_lat << ", " << g_lon << "\n"
                      << "  heading:  " << g_heading_deg << "°\n"
                      << "  step:     " << g_step_mps << " m\n"
                      << "  rate:     " << (1000 / g_interval_ms) << " Hz\n"
                      << "  paused:   " << (g_paused ? "yes" : "no") << "\n"
                      << "  socket:   " << VEHICLE_CMD_SOCK_PATH << std::endl;
        } else if (cmd == "help" || cmd == "h") {
            printHelp();
        } else {
            std::cerr << "Unknown command: " << cmd << std::endl;
        }
    }

    g_running = false;
    if (auto_seq.joinable())
        auto_seq.join();
    gps.join();
    std::cerr << "test_control: exiting" << std::endl;
    return 0;
}
