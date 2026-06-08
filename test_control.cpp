// =============================================================================
// test_control – 模拟小车直线行驶，输出 GPS 坐标到 stdout
//
// 用法:
//   交互模式 (可输入命令):
//     sudo ./KF-GINS-TestControl 0.3 | sudo ./KF-GINS-GnssPathControl ./config.yaml
//
//   仅直线行驶 (无交互, 后台运行):
//     sudo ./KF-GINS-TestControl 0.5 --quiet | sudo ./KF-GINS-GnssPathControl ./config.yaml
//
//   速度参数 (m/s, 默认 0.3):
//     sudo ./KF-GINS-TestControl 0.2           # 0.2 m/s
//     sudo ./KF-GINS-TestControl 0.5 --auto    # 0.5 m/s + auto序列
//
//   默认速度: 0.3 m/s (约 1.08 km/h, 缓慢步行速度)
//
// Commands (交互模式下可用):
//   quit              — exit
//   pause             — pause GPS output
//   cont              — continue GPS output
//   speed <m/s>       — set speed (m/s)
//   heading <deg>     — set GPS travel direction (0=north, 90=east)
//   origin <lat> <lon>— reset GPS simulation position
//   status            — print current GPS state
//   help              — show this help
//
//   stop / adjust / resume  — 发送车辆控制命令 (需 gnss_path_control)
//
// Architecture:
//   ┌───────────────┐   pipe(stdout)   ┌────────────────────┐
//   │ test_control   │ ───────────────→│ gnss_path_control   │
//   │  GPS模拟线程   │    "lat lon\n"   │  stdin ← GPS Coords  │
//   │  命令 ← stdin  │                  │  /dev/tb6612 ← 电机 │
//   └───────┬───────┘     Unix DGRAM    └─────────┬──────────┘
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
// GPS simulation state — defaults: lat=30.5, lon=114, heading east (90°)
// ---------------------------------------------------------------------------
static std::atomic<bool>  g_running{true};
static std::atomic<bool>  g_paused{false};
static std::atomic<bool>  g_quiet{false};       // --quiet: suppress stderr spam
static double g_lat         = 30.5;
static double g_lon         = 114.0;
static double g_heading_deg = 90.0;              // 90° = east (toward config goal)
static double g_speed_mps   = 0.3;               // 实际地面速度 (m/s), 命令行可覆盖
static int    g_interval_ms = 200;               // ms between GPS lines (5 Hz)

// 默认速度
static constexpr double kDefaultSpeedMps = 0.3;  // 0.3 m/s ≈ 1.08 km/h

// ---------------------------------------------------------------------------
// Auto-test sequence (仅 --auto 模式)
// ---------------------------------------------------------------------------
static std::atomic<bool> g_auto_mode{false};

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
        "  speed <m/s>       设置行驶速度 (当前: " << std::fixed << std::setprecision(2)
        << g_speed_mps << " m/s)\n"
        "  heading <deg>     设置航向 (0=北, 90=东, 当前: "
        << g_heading_deg << "°)\n"
        "  origin <lat> <lon> 重置 GPS 起点\n"
        "  pause             暂停 GPS 输出\n"
        "  cont              继续 GPS 输出\n"
        "  status            显示当前状态\n"
        "  stop               发送 CMD_STOP   → 小车刹车\n"
        "  adjust <deg>       发送 CMD_ADJUST → 原地旋转\n"
        "  resume             发送 CMD_RESUME → 恢复行驶\n"
        "  help               显示此帮助\n"
        "  quit               退出\n"
        "\n"
        "Pipe用法:\n"
        "  sudo ./KF-GINS-TestControl [速度m/s] | sudo ./KF-GINS-GnssPathControl ./config.yaml\n"
        << std::endl;
}

// ---------------------------------------------------------------------------
// GPS simulation thread — 以 speed_mps 匀速沿 heading 方向行驶
// ---------------------------------------------------------------------------
static void gpsThread() {
    while (g_running) {
        if (!g_paused) {
            // 输出当前 GPS 坐标 (lat lon)
            std::cout << std::fixed << std::setprecision(8)
                      << g_lat << " " << g_lon << std::endl;

            // 每步移动距离 = 速度(m/s) × 间隔(ms) / 1000
            // 1° lat ≈ 111111 m
            // 1° lon ≈ 111111 × cos(lat) m
            double step_m = g_speed_mps * g_interval_ms / 1000.0;
            double rad   = g_heading_deg * M_PI / 180.0;
            double dlat  = step_m * std::cos(rad) / 111111.0;
            double dlon  = step_m * std::sin(rad)
                         / (111111.0 * std::cos(g_lat * M_PI / 180.0));
            g_lat += dlat;
            g_lon += dlon;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(g_interval_ms));
    }
}

// ---------------------------------------------------------------------------
// Main — 解析命令行速度参数, 启动 GPS 线程, 处理交互命令
// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
    bool got_speed = false;

    // 解析命令行参数: [速度m/s] [--auto] [--quiet]
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--auto" || a == "-a") {
            g_auto_mode = true;
        } else if (a == "--quiet" || a == "-q") {
            g_quiet = true;
        } else {
            // 第一个非选项参数 = 速度 (m/s)
            if (!got_speed) {
                double v = std::atof(a.c_str());
                if (v > 0.0) {
                    g_speed_mps = v;
                    got_speed = true;
                }
            }
        }
    }

    // 打印启动信息
    if (!g_quiet) {
        std::cerr << "\n"
                  << "==================================================\n"
                  << "  Test Control – 模拟直线行驶\n"
                  << "==================================================\n"
                  << "  GPS起点: " << std::fixed << std::setprecision(6)
                  << g_lat << ", " << g_lon << "\n"
                  << "  航向:    " << std::fixed << std::setprecision(1)
                  << g_heading_deg << "° (90°=东)\n"
                  << "  速度:    " << std::fixed << std::setprecision(2) << g_speed_mps
                  << " m/s" << "  (" << std::fixed << std::setprecision(1)
                  << (g_speed_mps * 3.6) << " km/h)\n"
                  << "  频率:    " << (1000 / g_interval_ms) << " Hz\n"
                  << "  每步:    " << std::fixed << std::setprecision(3)
                  << (g_speed_mps * g_interval_ms / 1000.0) << " m\n"
                  << "  " << (g_auto_mode ? "自动测试模式" : "直线行驶模式")
                  << "\n==================================================\n"
                  << std::endl;
        printHelp();
    }

    // 启动 GPS 模拟线程
    std::thread gps(gpsThread);

    // 自动测试序列 (仅 --auto)
    std::thread auto_seq;
    if (g_auto_mode) {
        auto_seq = std::thread(autoTestSequence);
    }

    // 读取命令 (来自 stdin, 即用户的终端输入)
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
            std::cerr << "→ GPS起点设置为 "
                      << std::fixed << std::setprecision(6)
                      << g_lat << ", " << g_lon << std::endl;
        } else if (cmd == "heading") {
            double h;
            if (iss >> h) {
                g_heading_deg = h;
                std::cerr << "→ 航向设置为 " << g_heading_deg << "°" << std::endl;
            }
        } else if (cmd == "speed") {
            double v;
            if (iss >> v && v > 0.0) {
                g_speed_mps = v;
                std::cerr << "→ 速度设置为 " << std::fixed << std::setprecision(2)
                          << g_speed_mps << " m/s  ("
                          << std::fixed << std::setprecision(2)
                          << (g_speed_mps * 3.6) << " km/h)" << std::endl;
            }
        } else if (cmd == "pause") {
            g_paused = true;
            std::cerr << "→ GPS 输出已暂停" << std::endl;
        } else if (cmd == "cont" || cmd == "continue") {
            g_paused = false;
            std::cerr << "→ GPS 输出已继续" << std::endl;
        } else if (cmd == "status") {
            double step = g_speed_mps * g_interval_ms / 1000.0;
            std::cerr << "状态:\n"
                      << "  坐标:   " << std::fixed << std::setprecision(6)
                      << g_lat << ", " << g_lon << "\n"
                      << "  航向:   " << g_heading_deg << "°\n"
                      << "  速度:   " << std::fixed << std::setprecision(2)
                      << g_speed_mps << " m/s  ("
                      << std::fixed << std::setprecision(1)
                      << (g_speed_mps * 3.6) << " km/h)\n"
                      << "  频率:   " << (1000 / g_interval_ms) << " Hz\n"
                      << "  每步:   " << std::fixed << std::setprecision(3)
                      << step << " m\n"
                      << "  暂停:   " << (g_paused ? "是" : "否") << "\n"
                      << "  模式:   " << (g_auto_mode ? "自动测试" : "直线行驶")
                      << std::endl;
        } else if (cmd == "help" || cmd == "h") {
            printHelp();
        } else {
            std::cerr << "未知命令: " << cmd << " (输入 'help' 查看帮助)" << std::endl;
        }
    }

    g_running = false;
    if (auto_seq.joinable())
        auto_seq.join();
    gps.join();
    if (!g_quiet) std::cerr << "test_control: 已退出" << std::endl;
    return 0;
}
