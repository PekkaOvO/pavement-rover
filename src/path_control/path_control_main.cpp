#include "path_controller.h"

#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

void onSignal(int) {
    g_stop_requested = 1;
}
// 打印程序帮助信息
void printUsage(const char *program) {
    std::cout << "Usage:\n"
              << "  " << program << " kf-gins.yaml [--dry-run] [--dev /dev/tb6612]\n\n"
              << "Pipeline example:\n"
              << "  ./bin/KF-GINS ./dataset/kf-gins.yaml | "
              << "./bin/KF-GINS-PathControl ./dataset/kf-gins.yaml\n";
}
// 解析命令行参数
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
        } else {
            error = "unknown option: " + arg;
            return false;
        }
    }
    return true;
}
// 打印当前控制周期的一条状态记录
void printStatus(int seq,
                 const path_control::RobotState &state,
                 const path_control::ControlOutput &out) {
    std::cout << "CTRL"
              << ",sample=" << seq
              << ",x=" << state.x
              << ",y=" << state.y
              << ",yaw_rad=" << state.yaw_rad
              << ",fwd_v=" << state.forward_speed_mps
              << ",target=" << out.target_index << "/" << (out.path_size - 1)
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

} // namespace

int main(int argc, char **argv) {
    // 参数检查
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    // 信号注册
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // 加载配置
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

    // 初始化控制器与驱动
    path_control::PathController controller(config.goal, config.control);
    path_control::Tb6612Driver driver(config.runtime);

    if (!driver.open(error)) {
        std::cerr << error << std::endl;
        return 1;
    }

    // 打印警告/提示信息
    std::cerr << "KF-GINS path control started" << std::endl;
    std::cerr << "TB6612 device: " << config.runtime.device
              << (config.runtime.dry_run ? " (dry-run)" : "") << std::endl;
    std::cerr << "Goal: " << config.goal.lat_deg << ", " << config.goal.lon_deg << std::endl;
    std::cerr << "Waiting for NAV lines on stdin" << std::endl;

    std::string line;
    int seq = 0;
    bool origin_announced = false;

    // 主循环
    while (!g_stop_requested && std::getline(std::cin, line)) {
        // 循环成立条件
        path_control::NavOutput nav;
        if (!path_control::parseNavOutputLine(line, nav)) {
            continue;                                                                                          
        }

        path_control::RobotState state;
        path_control::ControlOutput out;

        // 更新小车状态并计算控制指令
        if (!controller.update(nav, state, out, error)) {
            std::cerr << error << std::endl;
            driver.stop(error);
            driver.close();
            return 1;
        }
        // 获取原点和局部目标点输出
        if (!origin_announced) {
            const path_control::GeoPoint origin = controller.origin();
            const path_control::Point2d goal_local = controller.goalLocal();
            std::cerr << "Origin: " << origin.lat_deg << ", " << origin.lon_deg
                      << " goal_local_m=(" << goal_local.x << ", " << goal_local.y << ")"
                      << " path_points=" << controller.pathSize() << std::endl;
            origin_announced = true;
        }
        // 判断是否调用函数输出状态。
        ++seq;
        if (seq % config.runtime.print_every == 0) {
            printStatus(seq, state, out);
        }
        // 若为真，输出目标到达信息，关闭驱动
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
        // 否则调用函数设置左右轮PWM。
        if (!driver.set(out.left_percent, out.right_percent, error)) {
            std::cerr << error << std::endl;
            driver.stop(error);
            driver.close();
            return 1;
        }
    }
    // 退出清理
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
