// =============================================================================
// Obstacle Avoidance Demo — 障碍物绕行演示程序
//
// 时序流程（7 个阶段 + 最终直行）：
//   Phase 1: 初始前进（后轮直行，等待第 1 次 CMD_STOP）
//   Phase 2: 停止 → 原地旋转（角度来自 GD32 CMD_ADJUST）
//   Phase 3: 旋转完成后继续前进（等待第 2 次 CMD_STOP）
//   Phase 4: 停止 stop_duration_s → 后退（舵机左转，后轮差速偏右）
//   Phase 5: 前进绕行（舵机右转，后轮差速左快右慢）
//   Phase 6: 轮回正，两后轮直行
//   Phase 7: 舵机左转 + 差速左转 → 回正直行
//   Final:   继续前进直到 Ctrl+C
//
// 所有参数通过 YAML 配置文件调节。
//
// 与 GD32 通信：监听 /tmp/gd32_vehicle_cmd.sock (Unix DGRAM)
//   CMD_STOP(1)  — 停止信号
//   CMD_ADJUST(2)— 携带旋转角度 angle_delta_deg
//   CMD_RESUME(3)— 流程结束标志（Phase 4 等待此信号再执行后退绕行）
// =============================================================================

#include "src/path_control/path_controller.h"
#include "src/path_control/vehicle_cmd_listener.h"

#include <csignal>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <yaml-cpp/yaml.h>

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------
namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void onSignal(int) {
    g_stop_requested = 1;
}

// -----------------------------------------------------------------------------
// Demo configuration (loaded from YAML)
// -----------------------------------------------------------------------------
struct DemoConfig {
    // Device
    std::string device = "/dev/tb6612";
    bool        dry_run = false;
    int         max_command_percent = 40;
    double      left_scale  = 1.0;   // 匹配 kf-gins.yaml
    double      right_scale = 1.0;   // 匹配 kf-gins.yaml

    // ---- Forward baseline (直行基准速度，调好的值不做改动) ----
    int forward_speed_percent = 20;

    // ---- Phase 2: in-place rotation ----
    int    rotation_percent        = 15;
    double rotation_calib_k        = 0.8;
    float  default_rotation_angle_deg = 90.0f;
    double rotation_wait_timeout_s = 10.0;

    // ---- Phase 4: stop delay then backup ----
    double stop_duration_s        = 2.0;
    double backup_duration_s      = 1.5;
    int    backup_servo_angle     = -30;    // 前轮左转
    double backup_left_coeff      = -0.75;  // 左后轮慢（负值=后退）
    double backup_right_coeff     = -1.25;  // 右后轮快

    // ---- Phase 5: evade forward (turn right) ----
    double evade_duration_s       = 2.0;
    int    evade_servo_angle      = 30;     // 前轮右转
    double evade_left_coeff       = 1.5;    // 左后轮快
    double evade_right_coeff      = 0.75;   // 右后轮慢

    // ---- Phase 6: go straight ----
    double straight_duration_s    = 3.0;
    double straight_coeff         = 1.0;    // 直行，与基准相同

    // ---- Phase 7: turn left ----
    double turn_left_duration_s   = 1.5;
    int    turn_left_servo_angle  = -30;    // 前轮左转
    double turn_left_left_coeff   = 0.75;   // 左后轮慢
    double turn_left_right_coeff  = 1.5;    // 右后轮快

    // Convenience: compute wheel percent from coefficients
    int calcLeft(double coeff) const {
        return static_cast<int>(forward_speed_percent * coeff);
    }
    int calcRight(double coeff) const {
        return static_cast<int>(forward_speed_percent * coeff);
    }
};

template <typename T>
void readOptional(const YAML::Node &node, const char *key, T &value) {
    if (node && node[key]) {
        value = node[key].as<T>();
    }
}

bool loadDemoConfig(const std::string &yaml_path,
                    DemoConfig &config,
                    std::string &error) {
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception &e) {
        error = std::string("failed to read config: ") + e.what();
        return false;
    }

    const YAML::Node demo = yaml["obstacle_avoid_demo"];
    if (!demo) {
        error = "missing 'obstacle_avoid_demo' section in " + yaml_path;
        return false;
    }

    readOptional(demo, "device",               config.device);
    readOptional(demo, "dry_run",              config.dry_run);
    readOptional(demo, "max_command_percent",  config.max_command_percent);
    readOptional(demo, "left_scale",           config.left_scale);
    readOptional(demo, "right_scale",          config.right_scale);

    readOptional(demo, "forward_speed_percent",  config.forward_speed_percent);

    readOptional(demo, "rotation_percent",           config.rotation_percent);
    readOptional(demo, "rotation_calib_k",           config.rotation_calib_k);
    readOptional(demo, "default_rotation_angle_deg", config.default_rotation_angle_deg);
    readOptional(demo, "rotation_wait_timeout_s",    config.rotation_wait_timeout_s);

    readOptional(demo, "stop_duration_s",       config.stop_duration_s);
    readOptional(demo, "backup_duration_s",     config.backup_duration_s);
    readOptional(demo, "backup_servo_angle",    config.backup_servo_angle);
    readOptional(demo, "backup_left_coeff",     config.backup_left_coeff);
    readOptional(demo, "backup_right_coeff",    config.backup_right_coeff);

    readOptional(demo, "evade_duration_s",      config.evade_duration_s);
    readOptional(demo, "evade_servo_angle",     config.evade_servo_angle);
    readOptional(demo, "evade_left_coeff",      config.evade_left_coeff);
    readOptional(demo, "evade_right_coeff",     config.evade_right_coeff);

    readOptional(demo, "straight_duration_s",   config.straight_duration_s);
    readOptional(demo, "straight_coeff",        config.straight_coeff);

    readOptional(demo, "turn_left_duration_s",  config.turn_left_duration_s);
    readOptional(demo, "turn_left_servo_angle", config.turn_left_servo_angle);
    readOptional(demo, "turn_left_left_coeff",  config.turn_left_left_coeff);
    readOptional(demo, "turn_left_right_coeff", config.turn_left_right_coeff);

    // Final phase uses forward_speed_percent directly (no separate field)

    return true;
}

// -----------------------------------------------------------------------------
// Helper: execute a timed motor + servo action
// Returns false if interrupted by SIGINT/SIGTERM.
// -----------------------------------------------------------------------------
bool timedAction(path_control::Tb6612Driver &driver,
                 double duration_s,
                 int left_pct,
                 int right_pct,
                 int servo_angle,
                 bool dry_run) {
    std::string error;

    if (!driver.servo(static_cast<float>(servo_angle), error)) {
        std::cerr << "servo(" << servo_angle << "°) failed: " << error << std::endl;
        return false;
    }
    if (!driver.set(left_pct, right_pct, error)) {
        std::cerr << "set(" << left_pct << ", " << right_pct << ") failed: "
                  << error << std::endl;
        return false;
    }

    std::cerr << "  → servo=" << servo_angle << "°"
              << "  L=" << left_pct << "%  R=" << right_pct << "%"
              << "  for " << duration_s << "s" << std::endl;

    // Sleep in small chunks so we can catch Ctrl+C
    const int total_ms = static_cast<int>(duration_s * 1000.0);
    constexpr int SLEEP_MS = 50;
    for (int elapsed = 0; elapsed < total_ms; elapsed += SLEEP_MS) {
        if (g_stop_requested) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
    }
    return true;
}

// -----------------------------------------------------------------------------
// Helper: in-place rotation (with configurable parameters)
// -----------------------------------------------------------------------------
bool doInPlaceRotation(path_control::Tb6612Driver &driver,
                       float angle_deg,
                       int rot_percent,
                       double calib_k,
                       bool dry_run) {
    if (angle_deg < -180.0f) angle_deg = -180.0f;
    if (angle_deg > 180.0f)  angle_deg = 180.0f;

    const double abs_angle = std::abs(static_cast<double>(angle_deg));
    if (abs_angle < 0.5) {
        std::cerr << "rotate: angle too small (" << angle_deg << "°), skip"
                  << std::endl;
        return true;
    }

    // t = θ / (P × K)
    const double duration_s = abs_angle / (rot_percent * calib_k);
    const int motor_val = (angle_deg > 0) ? rot_percent : -rot_percent;

    std::cerr << "rotate: angle=" << angle_deg << "°"
              << " P=" << rot_percent
              << " K=" << calib_k
              << " t=" << duration_s << "s"
              << " L=" << -motor_val << " R=" << motor_val
              << std::endl;

    if (dry_run) {
        std::cerr << "rotate: (dry-run, skipped)" << std::endl;
        return true;
    }

    std::string error;
    if (!driver.servo(0.0f, error)) {
        std::cerr << "rotate: servo center failed: " << error << std::endl;
        return false;
    }

    if (!driver.set(-motor_val, motor_val, error)) {
        std::cerr << "rotate: set failed: " << error << std::endl;
        return false;
    }

    // Sleep with interrupt check
    const int total_ms = static_cast<int>(duration_s * 1000.0);
    constexpr int SLEEP_MS = 50;
    for (int elapsed = 0; elapsed < total_ms; elapsed += SLEEP_MS) {
        if (g_stop_requested) {
            driver.stop(error);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
    }

    driver.stop(error);
    std::cerr << "rotate: done" << std::endl;
    return true;
}

// -----------------------------------------------------------------------------
// Helper: drain the command socket, return true if CMD_STOP found.
// Saves latest ADJUST angle into pending_angle / has_angle.
// -----------------------------------------------------------------------------
bool drainForStop(path_control::VehicleCmdListener &listener,
                  float &pending_angle, bool &has_angle) {
    path_control::VehicleCmdMessage cmd;
    bool found_stop = false;
    while (listener.tryRecv(cmd)) {
        switch (cmd.cmd) {
        case path_control::VehicleCmdType::STOP:
            found_stop = true;
            break;
        case path_control::VehicleCmdType::ADJUST:
            pending_angle = cmd.angle_delta_deg;
            has_angle = true;
            break;
        default:
            break;
        }
    }
    return found_stop;
}

// -----------------------------------------------------------------------------
// Helper: wait (non-blocking poll) until CMD_STOP arrives or signal received.
// During wait, keep draining socket to also capture ADJUST angles.
// Returns the angle from ADJUST if one was received, else default_angle.
// -----------------------------------------------------------------------------
float waitForStopAndGetAngle(path_control::VehicleCmdListener &listener,
                             float default_angle) {
    float pending_angle = 0.0f;
    bool has_angle = false;

    while (!g_stop_requested) {
        if (drainForStop(listener, pending_angle, has_angle)) {
            std::cerr << "CMD_STOP received" << std::endl;
            return has_angle ? pending_angle : default_angle;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return default_angle;
}

// -----------------------------------------------------------------------------
// Helper: wait a duration, checking g_stop_requested periodically.
// Returns false if interrupted.
// -----------------------------------------------------------------------------
bool waitDuration(double duration_s) {
    const int total_ms = static_cast<int>(duration_s * 1000.0);
    constexpr int SLEEP_MS = 50;
    for (int elapsed = 0; elapsed < total_ms; elapsed += SLEEP_MS) {
        if (g_stop_requested) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(SLEEP_MS));
    }
    return true;
}

// -----------------------------------------------------------------------------
// Helper: drain the command socket until CMD_RESUME (flow end) arrives.
// Any STOP / ADJUST commands received in between are consumed and ignored.
// Returns false if interrupted by signal.
// -----------------------------------------------------------------------------
bool waitForResume(path_control::VehicleCmdListener &listener) {
    path_control::VehicleCmdMessage cmd;
    while (!g_stop_requested) {
        while (listener.tryRecv(cmd)) {
            if (cmd.cmd == path_control::VehicleCmdType::RESUME) {
                std::cerr << "CMD_RESUME (flow end) received from GD32" << std::endl;
                return true;
            }
            // STOP, ADJUST and others are silently consumed
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}

void printUsage(const char *program) {
    std::cout << "Usage:\n"
              << "  " << program << " config.yaml [--dry-run]\n\n"
              << "Listens on /tmp/gd32_vehicle_cmd.sock for GD32 commands.\n"
              << "  CMD_STOP(1)   — stop the vehicle\n"
              << "  CMD_ADJUST(2) — rotation angle delta (float)\n"
              << "  CMD_RESUME(3) — flow end flag (Phase 4 waits for this)\n\n"
              << "Obstacle avoidance sequence:\n"
              << "  Phase 1: Forward → wait STOP → rotate → Phase 3\n"
              << "  Phase 3: Forward → wait STOP → obstacle avoidance\n"
              << "  Phase 4-7: backup → evade → straight → turn-left\n"
              << "  Final:   Continue forward until Ctrl+C\n";
}

} // anonymous namespace

// =============================================================================
int main(int argc, char **argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    // ---- Load configuration ----
    DemoConfig config;
    std::string error;
    if (!loadDemoConfig(argv[1], config, error)) {
        std::cerr << error << std::endl;
        return 1;
    }

    // Override dry_run via CLI
    for (int i = 2; i < argc; ++i) {
        if (std::string(argv[i]) == "--dry-run") {
            config.dry_run = true;
        }
    }

    // ---- Open motor driver ----
    path_control::RuntimeOptions opts;
    opts.device             = config.device;
    opts.dry_run            = config.dry_run;
    opts.max_command_percent = config.max_command_percent;
    opts.left_scale         = config.left_scale;
    opts.right_scale        = config.right_scale;

    path_control::Tb6612Driver driver(opts);
    if (!driver.open(error)) {
        std::cerr << error << std::endl;
        return 1;
    }

    // ---- Start command listener ----
    path_control::VehicleCmdListener cmd_listener;
    if (!cmd_listener.start()) {
        std::cerr << "Warning: failed to start vehicle command listener"
                  << std::endl;
    }

    std::cerr << "\n=== Obstacle Avoidance Demo ===\n"
              << "Device: " << config.device
              << (config.dry_run ? " (dry-run)" : "") << "\n"
              << "Forward: cmd L=R=" << config.forward_speed_percent
              << "% → actual L=" << static_cast<int>(config.forward_speed_percent * config.left_scale)
              << "% R=" << static_cast<int>(config.forward_speed_percent * config.right_scale)
              << "%  (scale " << config.left_scale << "/" << config.right_scale << ")\n"
              << "Other phases = coeff × " << config.forward_speed_percent << "\n"
              << "Rotation: P=" << config.rotation_percent
              << "% K=" << config.rotation_calib_k << "\n"
              << "===============================\n"
              << std::endl;

    // =========================================================================
    // Phase 1: Initial forward — wait for first CMD_STOP from GD32
    // =========================================================================
    {
        std::string err;
        driver.servo(0.0f, err);  // center steering

        if (!driver.set(config.forward_speed_percent,
                        config.forward_speed_percent, err)) {
            std::cerr << "Phase 1 set failed: " << err << std::endl;
            driver.close();
            return 1;
        }
    }
    std::cerr << "\n[Phase 1] Driving forward. Waiting for CMD_STOP..." << std::endl;

    float rotate_angle = waitForStopAndGetAngle(cmd_listener,
                                                config.default_rotation_angle_deg);
    if (g_stop_requested) goto shutdown;

    // Stop immediately
    {
        std::string err;
        driver.stop(err);
    }
    std::cerr << "[Phase 1] Stopped. Angle for rotation: "
              << rotate_angle << "°" << std::endl;

    // =========================================================================
    // Phase 2: In-place rotation
    // =========================================================================
    std::cerr << "\n[Phase 2] Rotating in place..." << std::endl;

    if (!doInPlaceRotation(driver, rotate_angle,
                           config.rotation_percent,
                           config.rotation_calib_k,
                           config.dry_run)) {
        goto shutdown;
    }

    // Drain any stale commands that arrived during rotation
    {
        float tmp_angle;
        bool tmp_has;
        while (drainForStop(cmd_listener, tmp_angle, tmp_has))
            ;  // discard
    }

    // =========================================================================
    // Phase 3: Forward after rotation — wait for second STOP (obstacle detected)
    // =========================================================================
    {
        std::string err;
        driver.servo(0.0f, err);

        if (!driver.set(config.forward_speed_percent,
                        config.forward_speed_percent, err)) {
            std::cerr << "Phase 3 set failed: " << err << std::endl;
            goto shutdown;
        }
    }
    std::cerr << "\n[Phase 3] Driving forward after rotation."
              << " Waiting for CMD_STOP (obstacle)..." << std::endl;

    waitForStopAndGetAngle(cmd_listener, 0.0f);  // angle not needed for phase 4
    if (g_stop_requested) goto shutdown;

    {
        std::string err;
        driver.stop(err);
    }
    std::cerr << "[Phase 3] CMD_STOP received (obstacle detected). Stopped."
              << std::endl;

    // =========================================================================
    // Phase 4: Wait for GD32 flow-end → backup
    // =========================================================================
    std::cerr << "\n[Phase 4] Stopped. Waiting for CMD_RESUME (flow end) from GD32..."
              << std::endl;
    if (!waitForResume(cmd_listener)) goto shutdown;

    std::cerr << "[Phase 4] Backing up (servo left, rear right faster)..."
              << std::endl;
    if (!timedAction(driver,
                     config.backup_duration_s,
                     config.calcLeft(config.backup_left_coeff),
                     config.calcRight(config.backup_right_coeff),
                     config.backup_servo_angle,
                     config.dry_run)) {
        goto shutdown;
    }

    // =========================================================================
    // Phase 5: Evade forward (turn right around obstacle)
    // =========================================================================
    std::cerr << "\n[Phase 5] Evading forward (servo right, rear left faster)..."
              << std::endl;
    if (!timedAction(driver,
                     config.evade_duration_s,
                     config.calcLeft(config.evade_left_coeff),
                     config.calcRight(config.evade_right_coeff),
                     config.evade_servo_angle,
                     config.dry_run)) {
        goto shutdown;
    }

    // =========================================================================
    // Phase 6: Go straight
    // =========================================================================
    std::cerr << "\n[Phase 6] Going straight..." << std::endl;
    if (!timedAction(driver,
                     config.straight_duration_s,
                     config.calcLeft(config.straight_coeff),
                     config.calcRight(config.straight_coeff),
                     0,   // servo center
                     config.dry_run)) {
        goto shutdown;
    }

    // =========================================================================
    // Phase 7: Turn left → straighten
    // =========================================================================
    std::cerr << "\n[Phase 7] Turning left..." << std::endl;
    if (!timedAction(driver,
                     config.turn_left_duration_s,
                     config.calcLeft(config.turn_left_left_coeff),
                     config.calcRight(config.turn_left_right_coeff),
                     config.turn_left_servo_angle,
                     config.dry_run)) {
        goto shutdown;
    }

    // =========================================================================
    // Final: Continue forward indefinitely
    // =========================================================================
    {
        std::string err;
        driver.servo(0.0f, err);
        if (!driver.set(config.forward_speed_percent,
                        config.forward_speed_percent, err)) {
            std::cerr << "Final set failed: " << err << std::endl;
            goto shutdown;
        }
    }
    std::cerr << "\n[Final] Obstacle avoidance complete."
              << " Continuing forward. Press Ctrl+C to stop."
              << std::endl;

    while (!g_stop_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // =========================================================================
    // Clean shutdown
    // =========================================================================
shutdown:
    {
        std::string err;
        driver.stop(err);
        driver.close();
    }
    if (g_stop_requested) {
        std::cerr << "\nInterrupted. Motor stopped." << std::endl;
    }
    std::cerr << "Demo finished." << std::endl;
    return 0;
}
