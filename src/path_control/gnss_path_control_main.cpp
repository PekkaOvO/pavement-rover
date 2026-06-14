#include "path_controller.h"
#include "gps_publisher.h"
#include "vehicle_cmd_listener.h"

#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <yaml-cpp/yaml.h>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

// ── Robot phase ──────────────────────────────────────────────────────────────
// Tracks the high-level state of the crack-detection → avoidance flow.
//   NORMAL         — normal GNSS path control (driving toward waypoints)
//   STOPPED_FIRST  — CMD_STOP received (AI found target), waiting for ADJUST
//   CRACK_STOPPED  — CMD_CRACK_STOP received (laser detected crack), waiting
//                    for RESUME (FLOW_END from GD32) → then obstacle avoidance
enum class RobotPhase : uint8_t {
    NORMAL,
    STOPPED_FIRST,
    CRACK_STOPPED,
};
RobotPhase g_phase = RobotPhase::NORMAL;

// ── Obstacle avoidance configuration ────────────────────────────────────────
struct AvoidConfig {
    int    forward_speed_percent = 20;

    // Phase 4: backup (servo left, rear differential)
    double backup_duration_s     = 1.5;
    int    backup_servo_angle    = -30;
    double backup_left_coeff     = -0.75;
    double backup_right_coeff    = -1.25;

    // Phase 5: evade forward (servo right, rear left faster)
    double evade_duration_s      = 2.0;
    int    evade_servo_angle     = 30;
    double evade_left_coeff      = 1.5;
    double evade_right_coeff     = 0.75;

    // Phase 6: go straight
    double straight_duration_s   = 3.0;
    double straight_coeff        = 1.0;

    // Phase 7: turn left
    double turn_left_duration_s  = 1.5;
    int    turn_left_servo_angle = -30;
    double turn_left_left_coeff  = 0.75;
    double turn_left_right_coeff = 1.5;

    int calcLeft(double coeff) const {
        return static_cast<int>(forward_speed_percent * coeff);
    }
    int calcRight(double coeff) const {
        return static_cast<int>(forward_speed_percent * coeff);
    }
};

void onSignal(int) {
    g_stop_requested = 1;
}

void printUsage(const char *program) {
    std::cout << "Usage:\n"
              << "  " << program << " kf-gins.yaml [--dry-run] [--dev /dev/tb6612]\n\n"
              << "Input lines accepted on stdin:\n"
              << "  lat lon\n"
              << "  time lat lon [height]\n"
              << "  week sow lat lon [height]\n"
              << "  GNSS,time,lat,lon[,height[,course_deg,speed_kph]]\n"
              << "  $GNGGA,... plus $GNVTG,... raw NMEA lines\n\n"
              << "Crack handling (two-stop flow):\n"
              << "  CMD_STOP(1)       — AI found target → stop for centering\n"
              << "  CMD_ADJUST(2)     — gimbal angle delta → rotate, then resume forward\n"
              << "  CMD_CRACK_STOP(4) — laser detected crack → stop for photo\n"
              << "  CMD_RESUME(3)     — flow end → execute obstacle avoidance, resume patrol\n\n"
              << "Obstacle avoidance (after RESUME):\n"
              << "  backup → evade(right) → straight → turn-left → GNSS path control\n\n"
              << "Rotation calibration (one-time):\n"
              << "  Set ROTATION_PERCENT and CALIB_K after experiment.\n";
}

template <typename T>
void readOptional(const YAML::Node &node, const char *key, T &value) {
    if (node && node[key]) {
        value = node[key].as<T>();
    }
}

bool loadAvoidConfig(const YAML::Node &node, AvoidConfig &config) {
    if (!node) {
        return false;
    }
    readOptional(node, "forward_speed_percent",  config.forward_speed_percent);
    readOptional(node, "backup_duration_s",      config.backup_duration_s);
    readOptional(node, "backup_servo_angle",     config.backup_servo_angle);
    readOptional(node, "backup_left_coeff",      config.backup_left_coeff);
    readOptional(node, "backup_right_coeff",     config.backup_right_coeff);
    readOptional(node, "evade_duration_s",       config.evade_duration_s);
    readOptional(node, "evade_servo_angle",      config.evade_servo_angle);
    readOptional(node, "evade_left_coeff",       config.evade_left_coeff);
    readOptional(node, "evade_right_coeff",      config.evade_right_coeff);
    readOptional(node, "straight_duration_s",    config.straight_duration_s);
    readOptional(node, "straight_coeff",         config.straight_coeff);
    readOptional(node, "turn_left_duration_s",   config.turn_left_duration_s);
    readOptional(node, "turn_left_servo_angle",  config.turn_left_servo_angle);
    readOptional(node, "turn_left_left_coeff",   config.turn_left_left_coeff);
    readOptional(node, "turn_left_right_coeff",  config.turn_left_right_coeff);
    return true;
}

// ── Timed motor + servo action (from obstacle_avoid_demo) ───────────────────
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

// ── Rotation constants ──────────────────────────────────────────────────────
constexpr int    ROTATION_PERCENT = 15;
constexpr double CALIB_K = 0.8;

void doInPlaceRotation(path_control::Tb6612Driver &driver,
                       float angle_deg,
                       bool dry_run) {
    if (angle_deg < -180.0f) angle_deg = -180.0f;
    if (angle_deg > 180.0f)  angle_deg = 180.0f;

    const double abs_angle = std::abs(static_cast<double>(angle_deg));
    if (abs_angle < 0.5) {
        std::cerr << "rotate: angle too small (" << angle_deg << "°), skipping"
                  << std::endl;
        return;
    }

    const double duration_s = abs_angle / (ROTATION_PERCENT * CALIB_K);
    const int motor_val = (angle_deg > 0) ? ROTATION_PERCENT : -ROTATION_PERCENT;

    std::cerr << "rotate: angle=" << angle_deg << "°"
              << " percent=" << ROTATION_PERCENT
              << " duration=" << duration_s << "s"
              << " left=" << -motor_val << " right=" << motor_val
              << std::endl;

    if (dry_run) {
        std::cerr << "rotate: (dry-run, not executed)" << std::endl;
        return;
    }

    std::string error;
    driver.servo(0.0f, error);
    if (!driver.set(-motor_val, motor_val, error)) {
        std::cerr << "rotate: set failed: " << error << std::endl;
        return;
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0)));

    driver.stop(error);
    std::cerr << "rotate: complete" << std::endl;
}

// ── Obstacle avoidance sequence (Phases 4-7 from demo) ──────────────────────
bool executeObstacleAvoidance(path_control::Tb6612Driver &driver,
                              const AvoidConfig &config,
                              bool dry_run) {
    std::cerr << "\n=== Obstacle avoidance start ===" << std::endl;

    // Phase 4: backup (servo left, rear right faster)
    std::cerr << "[Avoid 1/4] Backing up..." << std::endl;
    if (!timedAction(driver, config.backup_duration_s,
                     config.calcLeft(config.backup_left_coeff),
                     config.calcRight(config.backup_right_coeff),
                     config.backup_servo_angle, dry_run))
        return false;

    // Phase 5: evade forward (servo right, rear left faster → turn right)
    std::cerr << "[Avoid 2/4] Evading forward (turn right)..." << std::endl;
    if (!timedAction(driver, config.evade_duration_s,
                     config.calcLeft(config.evade_left_coeff),
                     config.calcRight(config.evade_right_coeff),
                     config.evade_servo_angle, dry_run))
        return false;

    // Phase 6: go straight
    std::cerr << "[Avoid 3/4] Going straight..." << std::endl;
    if (!timedAction(driver, config.straight_duration_s,
                     config.calcLeft(config.straight_coeff),
                     config.calcRight(config.straight_coeff),
                     0, dry_run))
        return false;

    // Phase 7: turn left then straighten
    std::cerr << "[Avoid 4/4] Turning left..." << std::endl;
    if (!timedAction(driver, config.turn_left_duration_s,
                     config.calcLeft(config.turn_left_left_coeff),
                     config.calcRight(config.turn_left_right_coeff),
                     config.turn_left_servo_angle, dry_run))
        return false;

    // Straighten steering
    {
        std::string err;
        driver.servo(0.0f, err);
    }

    std::cerr << "=== Obstacle avoidance complete. Resuming GNSS path control ==="
              << std::endl;
    return true;
}

// ── Status printing ─────────────────────────────────────────────────────────
void printStatus(int seq,
                 const path_control::GnssPosition &fix,
                 const path_control::RobotState &state,
                 const path_control::ControlOutput &out) {
    std::cout << "GNSS_CTRL"
              << ",sample=" << seq
              << ",lat=" << fix.position.lat_deg
              << ",lon=" << fix.position.lon_deg
              << ",course_deg=" << (fix.has_course ? fix.course_deg : -1.0)
              << ",speed_kph=" << (fix.has_speed ? fix.speed_kph : -1.0)
              << ",speed_mps=" << (fix.has_speed ? fix.speed_mps : -1.0)
              << ",x=" << state.x
              << ",y=" << state.y
              << ",yaw_rad=" << state.yaw_rad
              << ",fwd_v=" << state.forward_speed_mps
              << ",target=" << out.target_index << "/" << std::max(0, out.path_size - 1)
              << ",dist=" << out.distance_to_goal_m
              << ",lookahead=" << out.lookahead_m
              << ",v_cmd=" << out.v_cmd_mps
              << ",omega=" << out.omega_cmd_radps
              << ",left_mps=" << out.left_wheel_mps
              << ",right_mps=" << out.right_wheel_mps
              << ",left_pct=" << out.left_percent
              << ",right_pct=" << out.right_percent
              << std::endl;
}

void printGnssData(int seq, const path_control::GnssPosition &fix) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(10)
        << "GNSS"
        << ",sample=" << seq;
    if (fix.has_time) {
        oss << std::setprecision(3) << ",time=" << fix.time;
    } else {
        oss << ",time=nan";
    }
    oss << std::setprecision(10)
        << ",lat=" << fix.position.lat_deg
        << ",lon=" << fix.position.lon_deg;
    if (fix.has_height) {
        oss << std::setprecision(4) << ",height=" << fix.height_m;
    } else {
        oss << ",height=nan";
    }
    if (fix.has_course) {
        oss << std::setprecision(3) << ",course_deg=" << fix.course_deg;
    } else {
        oss << ",course_deg=nan";
    }
    if (fix.has_speed) {
        oss << std::setprecision(3)
            << ",speed_kph=" << fix.speed_kph
            << ",speed_mps=" << fix.speed_mps;
    } else {
        oss << ",speed_kph=nan,speed_mps=nan";
    }
    std::cerr << oss.str() << std::endl;
}

// ── Command-line overrides ──────────────────────────────────────────────────
bool applyCommandLineOverrides(int argc,
                               char **argv,
                               path_control::PathControlConfig &config,
                               std::string &error) {
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--dry-run") {
            config.runtime.dry_run = true;
        } else if (arg == "--dev") {
            if (i + 1 >= argc) {
                error = "missing value for --dev";
                return false;
            }
            config.runtime.device = argv[++i];
        } else if (arg == "--print-every") {
            if (i + 1 >= argc) {
                error = "missing value for --print-every";
                return false;
            }
            config.runtime.print_every = std::max(1, std::atoi(argv[++i]));
        } else if (arg == "--heading-min-distance") {
            if (i + 1 >= argc) {
                error = "missing value for --heading-min-distance";
                return false;
            }
            config.gnss_only.heading_min_distance_m = std::atof(argv[++i]);
        } else if (arg == "--max-speed") {
            if (i + 1 >= argc) {
                error = "missing value for --max-speed";
                return false;
            }
            config.gnss_only.max_speed_mps = std::atof(argv[++i]);
        } else if (arg == "--vtg-min-speed") {
            if (i + 1 >= argc) {
                error = "missing value for --vtg-min-speed";
                return false;
            }
            config.gnss_only.vtg_min_speed_mps = std::atof(argv[++i]);
        } else {
            error = "unknown option: " + arg;
            return false;
        }
    }
    config.runtime.print_every = std::max(1, config.runtime.print_every);
    config.gnss_only.heading_min_distance_m =
        std::max(0.02, config.gnss_only.heading_min_distance_m);
    config.gnss_only.max_speed_mps = std::max(0.0, config.gnss_only.max_speed_mps);
    config.gnss_only.vtg_min_speed_mps = std::max(0.0, config.gnss_only.vtg_min_speed_mps);
    return true;
}

} // anonymous namespace

// =============================================================================
int main(int argc, char **argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    std::string error;
    path_control::PathControlConfig config;
    if (!path_control::loadPathControlConfig(argv[1], config, error)) {
        std::cerr << error << std::endl;
        return 1;
    }
    if (!applyCommandLineOverrides(argc, argv, config, error)) {
        std::cerr << error << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    // Load obstacle avoidance config from YAML
    AvoidConfig avoid_config;
    {
        YAML::Node yaml;
        try {
            yaml = YAML::LoadFile(argv[1]);
        } catch (...) {
            // ignore — use defaults
        }
        const YAML::Node pc = yaml["path_control"];
        loadAvoidConfig(pc["obstacle_avoid"], avoid_config);
    }

    const std::vector<path_control::GeoPoint> goals =
        config.has_goals ? config.goals : std::vector<path_control::GeoPoint>{config.goal};
    path_control::GnssPathController controller(goals,
                                                config.control,
                                                config.gnss_only);
    path_control::Tb6612Driver driver(config.runtime);
    GpsPublisher gps_pub;

    if (!driver.open(error)) {
        std::cerr << error << std::endl;
        return 1;
    }

    // Start vehicle command listener
    path_control::VehicleCmdListener cmd_listener;
    if (!cmd_listener.start()) {
        std::cerr << "Warning: failed to start vehicle command listener"
                  << std::endl;
    }

    std::cerr << "KF-GINS GNSS path control started (two-stop flow)"
              << std::endl;
    std::cerr << "TB6612 device: " << config.runtime.device
              << (config.runtime.dry_run ? " (dry-run)" : "") << std::endl;
    std::cerr << "Goal: " << config.goal.lat_deg << ", " << config.goal.lon_deg
              << " (" << goals.size() << " waypoints)" << std::endl;
    std::cerr << "Rotation: P=" << ROTATION_PERCENT
              << "%, K=" << CALIB_K
              << " (calibrate in field)" << std::endl;
    std::cerr << "Obstacle avoidance: backup=" << avoid_config.backup_duration_s << "s"
              << " evade=" << avoid_config.evade_duration_s << "s"
              << " straight=" << avoid_config.straight_duration_s << "s"
              << " turn_left=" << avoid_config.turn_left_duration_s << "s" << std::endl;
    std::cerr << "Waiting for GNSS position lines on stdin" << std::endl;

    std::string line;
    int seq = 0;
    bool origin_announced = false;
    bool has_cached_vtg = false;
    path_control::GnssVtg cached_vtg;

    while (!g_stop_requested && std::getline(std::cin, line)) {
        // =====================================================================
        // 1. Drain vehicle command socket (non-blocking)
        // =====================================================================
        {
            path_control::VehicleCmdMessage cmd;
            while (cmd_listener.tryRecv(cmd)) {
                switch (cmd.cmd) {
                case path_control::VehicleCmdType::STOP:
                    // ── First stop: AI found target ──
                    if (g_phase == RobotPhase::NORMAL) {
                        std::cerr << "CMD: STOP (AI target found)" << std::endl;
                        g_phase = RobotPhase::STOPPED_FIRST;
                        driver.stop(error);
                    } else {
                        std::cerr << "CMD: STOP ignored (phase="
                                  << static_cast<int>(g_phase) << ")" << std::endl;
                    }
                    break;

                case path_control::VehicleCmdType::ADJUST:
                    // ── Rotate after first stop ──
                    if (g_phase == RobotPhase::STOPPED_FIRST) {
                        std::cerr << "CMD: ADJUST angle="
                                  << cmd.angle_delta_deg << "°" << std::endl;
                        doInPlaceRotation(driver, cmd.angle_delta_deg,
                                          config.runtime.dry_run);
                        g_phase = RobotPhase::NORMAL;
                        std::cerr << "Rotation done, resuming forward path control"
                                  << std::endl;
                    } else {
                        std::cerr << "CMD: ADJUST ignored (not in STOPPED_FIRST)"
                                  << std::endl;
                    }
                    break;

                case path_control::VehicleCmdType::CRACK_STOP:
                    // ── Second stop: laser detected crack ──
                    std::cerr << "CMD: CRACK_STOP (laser detected crack)"
                              << std::endl;
                    g_phase = RobotPhase::CRACK_STOPPED;
                    driver.stop(error);
                    break;

                case path_control::VehicleCmdType::RESUME:
                    // ── FLOW_END: obstacle avoidance → resume ──
                    if (g_phase == RobotPhase::CRACK_STOPPED) {
                        std::cerr << "CMD: RESUME (flow end) — executing avoidance"
                                  << std::endl;
                        if (!executeObstacleAvoidance(driver, avoid_config,
                                                      config.runtime.dry_run)) {
                            if (g_stop_requested) {
                                std::cerr << "Avoidance interrupted" << std::endl;
                            }
                        }
                        g_phase = RobotPhase::NORMAL;
                    } else {
                        std::cerr << "CMD: RESUME (no crack-stop pending, ignoring)"
                                  << std::endl;
                    }
                    break;

                default:
                    break;
                }
            }
        }

        // =====================================================================
        // 2. Brake state: keep publishing GPS, keep brakes on
        // =====================================================================
        if (g_phase == RobotPhase::STOPPED_FIRST ||
            g_phase == RobotPhase::CRACK_STOPPED) {
            path_control::GnssVtg vtg;
            if (path_control::parseGnssVtgLine(line, vtg)) {
                cached_vtg = vtg;
                has_cached_vtg = true;
                continue;
            }

            path_control::GnssPosition fix;
            if (path_control::parseGnssPositionLine(line, fix)) {
                const int sample = seq + 1;
                printGnssData(sample, fix);

                path_control::RobotState state;
                state.x = 0; state.y = 0;
                gps_pub.publish(fix, state, fix.has_time ? fix.time : 0.0);
                seq = sample;
            }

            driver.stop(error);
            continue;
        }

        // =====================================================================
        // 3. Normal GNSS path control
        // =====================================================================
        path_control::GnssVtg vtg;
        if (path_control::parseGnssVtgLine(line, vtg)) {
            cached_vtg = vtg;
            has_cached_vtg = true;
            continue;
        }

        path_control::GnssPosition fix;
        if (!path_control::parseGnssPositionLine(line, fix)) {
            continue;
        }
        if (has_cached_vtg) {
            if (!fix.has_course && cached_vtg.has_course) {
                fix.course_deg = cached_vtg.course_deg;
                fix.has_course = true;
            }
            if (!fix.has_speed && cached_vtg.has_speed) {
                fix.speed_mps = cached_vtg.speed_mps;
                fix.speed_kph = cached_vtg.speed_kph;
                fix.has_speed = true;
            }
        }

        const int sample = seq + 1;
        printGnssData(sample, fix);

        path_control::RobotState state;
        path_control::ControlOutput out;
        if (!controller.update(fix, state, out, error)) {
            std::cerr << error << std::endl;
            driver.stop(error);
            driver.close();
            return 1;
        }

        gps_pub.publish(fix, state, fix.has_time ? fix.time : 0.0);

        if (!origin_announced) {
            const path_control::GeoPoint origin = controller.origin();
            const path_control::Point2d goal_local = controller.goalLocal();
            std::cerr << "Origin: " << origin.lat_deg << ", " << origin.lon_deg
                      << " goal_local_m=(" << goal_local.x << ", " << goal_local.y << ")"
                      << " path_points=" << controller.pathSize()
                      << " waypoints=" << (config.has_goals ? config.goals.size() : 1U) << std::endl;
            origin_announced = true;
        }

        seq = sample;
        if (seq % config.runtime.print_every == 0) {
            printStatus(seq, fix, state, out);
        }

        if (out.goal_reached) {
            if (!driver.stop(error)) {
                std::cerr << error << std::endl;
                driver.close();
                return 1;
            }
            std::cerr << "Goal reached, distance=" << out.distance_to_goal_m << " m" << std::endl;
            driver.close();
            return 0;
        }

        if (!driver.set(out.left_percent, out.right_percent, error)) {
            std::cerr << error << std::endl;
            driver.stop(error);
            driver.close();
            return 1;
        }
    }

    if (!driver.stop(error)) {
        std::cerr << error << std::endl;
        driver.close();
        return 1;
    }
    driver.close();

    if (g_stop_requested) {
        std::cerr << "Stop requested, motor stopped" << std::endl;
    } else {
        std::cerr << "Input ended, motor stopped" << std::endl;
    }
    return 0;
}
