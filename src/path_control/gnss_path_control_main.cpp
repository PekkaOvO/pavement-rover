#include "path_controller.h"

#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

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

    if (!driver.open(error)) {
        std::cerr << error << std::endl;
        return 1;
    }

    std::cerr << "KF-GINS GNSS-only path control started" << std::endl;
    std::cerr << "TB6612 device: " << config.runtime.device
              << (config.runtime.dry_run ? " (dry-run)" : "") << std::endl;
    std::cerr << "Goal: " << config.goal.lat_deg << ", " << config.goal.lon_deg << std::endl;
    std::cerr << "Waiting for GNSS position lines on stdin" << std::endl;

    std::string line;
    int seq = 0;
    bool origin_announced = false;
    bool has_cached_vtg = false;
    path_control::GnssVtg cached_vtg;

    while (!g_stop_requested && std::getline(std::cin, line)) {
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
