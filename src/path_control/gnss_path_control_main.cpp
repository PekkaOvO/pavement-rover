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

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

// Crack handling state
bool g_crack_stop = false;

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
              << "Crack handling:\n"
              << "  gd32_bridge sends STOP/ADJUST/RESUME via Unix socket.\n"
              << "  ADJUST carries gimbal angle delta for in-place rotation.\n"
              << "\n"
              << "Rotation calibration (one-time):\n"
              << "  Set ROTATION_PERCENT and CALIB_K after experiment.\n"
              << "\n"
              << "Example:\n"
              << "  cat processed_gnss.txt | " << program
              << " ./dataset/kf-gins.yaml --dry-run\n";
}

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

// ---------------------------------------------------------------------------
// Calibration parameters for open-loop in-place rotation
// ---------------------------------------------------------------------------

// Wheel speed percent used during rotation (both wheels, opposite directions).
// Must be calibrated together with CALIB_K.
constexpr int ROTATION_PERCENT = 15;

// Calibration factor K:
//   measured_angle_deg = ROTATION_PERCENT * K * time_seconds
//   K = measured_angle_deg / (ROTATION_PERCENT * time_seconds)
//
// Calibration procedure:
//   1. run: servo 0; set +P -P for exactly 5 seconds; stop
//   2. measure how many degrees the vehicle rotated
//   3. K = measured_deg / (P * 5)
//
// Example: rotated 60° in 5s at 15% → K = 60 / (15 * 5) = 0.8
// Current: P=15, K=0.8  →  90° rotation takes 90/(15*0.8) = 7.5s
constexpr double CALIB_K = 0.8;

void doInPlaceRotation(path_control::Tb6612Driver &driver,
                       float angle_deg,
                       bool dry_run) {
    // Clamp angle to reasonable range
    if (angle_deg < -180.0f) angle_deg = -180.0f;
    if (angle_deg > 180.0f)  angle_deg = 180.0f;

    const double abs_angle = std::abs(static_cast<double>(angle_deg));
    if (abs_angle < 0.5) {
        std::cerr << "rotate: angle too small (" << angle_deg << "°), skipping"
                  << std::endl;
        return;
    }

    // Calculate rotation duration
    // t = θ / (P × K)
    const double duration_s = abs_angle / (ROTATION_PERCENT * CALIB_K);

    // Sign determines direction: positive → turn right, negative → turn left
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

    // Step 1: center steering servo
    std::string error;
    driver.servo(0.0f, error);

    // Step 2: start counter-rotating (pure in-place rotation)
    //   Left wheel: -motor_val, Right wheel: +motor_val
    //   (adjust signs based on your vehicle's wheel mapping)
    if (!driver.set(-motor_val, motor_val, error)) {
        std::cerr << "rotate: set failed: " << error << std::endl;
        return;
    }

    // Step 3: wait for calculated duration
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0)));

    // Step 4: stop
    driver.stop(error);

    std::cerr << "rotate: complete" << std::endl;
}

} // namespace

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

    path_control::GnssPathController controller(config.goal,
                                                config.control,
                                                config.gnss_only);
    path_control::Tb6612Driver driver(config.runtime);
    GpsPublisher gps_pub;

    if (!driver.open(error)) {
        std::cerr << error << std::endl;
        return 1;
    }

    // -----------------------------------------------------------------------
    // Start vehicle command listener (receives STOP/ADJUST/RESUME from gd32)
    // -----------------------------------------------------------------------
    path_control::VehicleCmdListener cmd_listener;
    if (!cmd_listener.start()) {
        std::cerr << "Warning: failed to start vehicle command listener"
                  << std::endl;
    }

    std::cerr << "KF-GINS GNSS-only path control started" << std::endl;
    std::cerr << "TB6612 device: " << config.runtime.device
              << (config.runtime.dry_run ? " (dry-run)" : "") << std::endl;
    std::cerr << "Goal: " << config.goal.lat_deg << ", " << config.goal.lon_deg << std::endl;
    std::cerr << "Rotation: P=" << ROTATION_PERCENT
              << "%, K=" << CALIB_K
              << " (calibrate in field)" << std::endl;
    std::cerr << "Waiting for GNSS position lines on stdin" << std::endl;

    std::string line;
    int seq = 0;
    bool origin_announced = false;
    bool has_cached_vtg = false;
    path_control::GnssVtg cached_vtg;

    while (!g_stop_requested && std::getline(std::cin, line)) {
        // -------------------------------------------------------------------
        // 1. Drain vehicle command socket (non-blocking)
        // -------------------------------------------------------------------
        {
            path_control::VehicleCmdMessage cmd;
            while (cmd_listener.tryRecv(cmd)) {
                switch (cmd.cmd) {
                case path_control::VehicleCmdType::STOP:
                    std::cerr << "CMD: STOP" << std::endl;
                    g_crack_stop = true;
                    if (!driver.stop(error)) {
                        std::cerr << "stop failed: " << error << std::endl;
                    }
                    break;

                case path_control::VehicleCmdType::ADJUST:
                    std::cerr << "CMD: ADJUST angle="
                              << cmd.angle_delta_deg << "°" << std::endl;
                    g_crack_stop = true;
                    // Execute in-place rotation immediately
                    doInPlaceRotation(driver, cmd.angle_delta_deg,
                                      config.runtime.dry_run);
                    break;

                case path_control::VehicleCmdType::RESUME:
                    std::cerr << "CMD: RESUME" << std::endl;
                    g_crack_stop = false;
                    break;

                default:
                    break;
                }
            }
        }

        // -------------------------------------------------------------------
        // 2. If crack-stopped, keep brakes on and re-check commands next cycle
        // -------------------------------------------------------------------
        if (g_crack_stop) {
            // 继续读取并发布GPS（保持地图更新），但不驱动电机
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

                // Still publish GPS so CarView2 map keeps updating
                path_control::RobotState state;
                state.x = 0; state.y = 0;
                gps_pub.publish(fix, state, fix.has_time ? fix.time : 0.0);
                seq = sample;
            }

            // Keep brakes on
            driver.stop(error);
            continue;
        }

        // -------------------------------------------------------------------
        // 3. Normal GNSS path control (existing logic)
        // -------------------------------------------------------------------
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

        // Publish GPS+state to gd32_bridge via Unix socket
        gps_pub.publish(fix, state, fix.has_time ? fix.time : 0.0);

        if (!origin_announced) {
            const path_control::GeoPoint origin = controller.origin();
            const path_control::Point2d goal_local = controller.goalLocal();
            std::cerr << "Origin: " << origin.lat_deg << ", " << origin.lon_deg
                      << " goal_local_m=(" << goal_local.x << ", " << goal_local.y << ")"
                      << " path_points=" << controller.pathSize() << std::endl;
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
