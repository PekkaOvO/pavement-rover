// rc_receiver_main.cpp
// i.MX6ULL 端 TCP 遥控接收程序
// 编译: arm-linux-gnueabihf-g++ -std=c++11 -O2 rc_receiver_main.cpp -pthread -o rc_receiver
// 运行: ./rc_receiver [--port 9876] [--dev /dev/tb6612] [--dry-run]
//                     [--turn-ratio 0.0] [--timeout 500] [--left-scale 1.0] [--right-scale 0.8]
//
// 默认: 小车静止启动, 舵机朝正前方
// 安全: 500ms 无指令自动停车

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <string>
#include <csignal>
#include <atomic>
#include <chrono>

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <poll.h>

// ============================================================================
// RC 协议: PC → i.MX6ULL (TCP, 16 bytes)
// ============================================================================
#pragma pack(push, 1)
struct RcCommand {
    uint32_t magic;       // 0x52430001 ("RC\0\1")
    int32_t  speed_pct;   // -100~100, 0=stop, +=前进, -=后退
    int32_t  servo_deg;   // -90~90, 0=中心, -=左转, +=右转
    int32_t  diff_pct;    // -100~100, 差速量: 左轮+=diff, 右轮-=diff (方向键)
    uint32_t seq;         // 序列号 (检测过期指令)
};
#pragma pack(pop)

static_assert(sizeof(RcCommand) == 20, "RcCommand must be 20 bytes");

constexpr uint32_t RC_MAGIC = 0x52430001;

// ============================================================================
// 简易 TB6612 电机驱动 (直接写设备文件)
// ============================================================================
class MotorDriver {
public:
    MotorDriver() = default;
    ~MotorDriver() { close(); }

    bool open(const std::string &dev, std::string &error) {
        close();
        fd_ = ::open(dev.c_str(), O_WRONLY | O_CLOEXEC);
        if (fd_ < 0) {
            error = "cannot open " + dev + ": " + std::strerror(errno);
            return false;
        }
        return true;
    }

    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    bool stop(std::string &error) {
        return writeCmd("stop\n", 5, error);
    }

    // left_pct, right_pct: -100..100
    bool set(int left_pct, int right_pct, std::string &error) {
        char buf[64];
        int n = std::snprintf(buf, sizeof(buf), "set %d %d\n", left_pct, right_pct);
        if (n < 0) { error = "snprintf failed"; return false; }
        return writeCmd(buf, static_cast<size_t>(n), error);
    }

    // deg: -90..90
    bool servo(int deg, std::string &error) {
        if (deg < -90) deg = -90;
        if (deg > 90)  deg = 90;
        char buf[32];
        int n = std::snprintf(buf, sizeof(buf), "servo %d\n", deg);
        if (n < 0) { error = "snprintf failed"; return false; }
        return writeCmd(buf, static_cast<size_t>(n), error);
    }

private:
    bool writeCmd(const char *buf, size_t len, std::string &error) {
        if (fd_ < 0) { error = "device not open"; return false; }
        while (len > 0) {
            ssize_t n = ::write(fd_, buf, len);
            if (n < 0) {
                if (errno == EINTR) continue;
                error = std::string("write error: ") + std::strerror(errno);
                return false;
            }
            buf += n;
            len -= static_cast<size_t>(n);
        }
        return true;
    }

    int fd_ = -1;
};

// ============================================================================
// 配置 (命令行参数)
// ============================================================================
struct Config {
    int  port             = 9876;
    std::string device    = "/dev/tb6612";
    bool dry_run          = false;
    double turn_ratio     = 0.0;    // 转弯时内侧轮速度比 (0=原地旋转)
    int  stop_timeout_ms  = 500;    // 无指令安全超时
    double left_scale     = 1.0;    // 左电机修正系数
    double right_scale    = 0.48;   // 右电机修正系数
    bool swap_ab          = false;
    bool invert_left      = false;
    bool invert_right     = false;
    int  max_pct          = 100;
};

static void printUsage(const char *prog) {
    std::cerr << "Usage: " << prog << " [options]\n"
              << "  --port PORT         TCP listen port (default 9876)\n"
              << "  --dev DEVICE        TB6612 device (default /dev/tb6612)\n"
              << "  --dry-run           Don't touch hardware\n"
              << "  --turn-ratio RATIO  Inner wheel speed ratio during turn (default 0.0)\n"
              << "  --timeout MS        Stop after MS no command (default 500)\n"
              << "  --left-scale SCALE  Left motor correction (default 1.0)\n"
              << "  --right-scale SCALE Right motor correction (default 0.8)\n"
              << "  --max-pct PCT       Max command percent (default 100)\n"
              << "  --swap-ab           Swap motor A/B\n"
              << "  --invert-left       Invert left motor direction\n"
              << "  --invert-right      Invert right motor direction\n";
}

static bool parseArgs(int argc, char **argv, Config &cfg, std::string &error) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return false;
        } else if (arg == "--port" && i+1 < argc) {
            cfg.port = std::atoi(argv[++i]);
        } else if (arg == "--dev" && i+1 < argc) {
            cfg.device = argv[++i];
        } else if (arg == "--dry-run") {
            cfg.dry_run = true;
        } else if (arg == "--turn-ratio" && i+1 < argc) {
            cfg.turn_ratio = std::atof(argv[++i]);
        } else if (arg == "--timeout" && i+1 < argc) {
            cfg.stop_timeout_ms = std::atoi(argv[++i]);
        } else if (arg == "--left-scale" && i+1 < argc) {
            cfg.left_scale = std::atof(argv[++i]);
        } else if (arg == "--right-scale" && i+1 < argc) {
            cfg.right_scale = std::atof(argv[++i]);
        } else if (arg == "--max-pct" && i+1 < argc) {
            cfg.max_pct = std::atoi(argv[++i]);
        } else if (arg == "--swap-ab") {
            cfg.swap_ab = true;
        } else if (arg == "--invert-left") {
            cfg.invert_left = true;
        } else if (arg == "--invert-right") {
            cfg.invert_right = true;
        } else {
            error = "unknown option: " + arg;
            return false;
        }
    }
    return true;
}

// ============================================================================
// 应用 scale/invert/swap/clamp 修正
// ============================================================================
static void applyMotorCorrections(int &left, int &right, const Config &cfg) {
    left  = static_cast<int>(static_cast<double>(left) * cfg.left_scale);
    right = static_cast<int>(static_cast<double>(right) * cfg.right_scale);
    if (cfg.invert_left)  left  = -left;
    if (cfg.invert_right) right = -right;
    int limit = cfg.max_pct;
    if (limit > 100) limit = 100;
    if (limit < 0)   limit = 0;
    if (left  > limit)  left  = limit;
    if (left  < -limit) left  = -limit;
    if (right > limit)  right = limit;
    if (right < -limit) right = -limit;
    if (cfg.swap_ab) {
        int tmp = left;
        left = right;
        right = tmp;
    }
}

// ============================================================================
// TCP 服务器 + 主循环
// ============================================================================
static std::atomic<bool> g_stop{false};

static void onSignal(int) {
    g_stop = true;
}

int main(int argc, char **argv) {
    Config cfg;
    std::string _err;
    {
        std::string error;
        if (!parseArgs(argc, argv, cfg, error)) {
            if (!error.empty())
                std::cerr << "Error: " << error << std::endl;
            return 1;
        }
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    // 打开电机驱动设备
    MotorDriver driver;
    if (!cfg.dry_run) {
        std::string error;
        if (!driver.open(cfg.device, error)) {
            std::cerr << "rc_receiver: " << error << std::endl;
            return 1;
        }
    }
    { std::string ignore; driver.stop(ignore); driver.servo(0, ignore); } // 初始停车+舵机居中

    // 创建 TCP server socket
    int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        std::cerr << "rc_receiver: socket() failed: " << std::strerror(errno) << std::endl;
        return 1;
    }
    {
        int reuse = 1;
        ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    }

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(cfg.port));

    if (::bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "rc_receiver: bind port " << cfg.port
                  << " failed: " << std::strerror(errno) << std::endl;
        ::close(listen_fd);
        return 1;
    }
    ::listen(listen_fd, 1);

    std::cerr << "rc_receiver: listening on port " << cfg.port
              << (cfg.dry_run ? " (dry-run)" : "")
              << ", device=" << cfg.device
              << ", turn_ratio=" << cfg.turn_ratio
              << std::endl;

    // 主循环
    int client_fd = -1;
    uint32_t last_seq = 0;
    auto last_cmd_time = std::chrono::steady_clock::now();
    bool had_command = false;

    while (!g_stop) {
        // 如果没有客户端连接, 等待接受
        if (client_fd < 0) {
            struct pollfd accept_pfd;
            accept_pfd.fd = listen_fd;
            accept_pfd.events = POLLIN;
            int ret = ::poll(&accept_pfd, 1, 500);  // 500ms 超时 (可响应退出信号)
            if (ret <= 0) continue;

            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            client_fd = ::accept(listen_fd, (struct sockaddr *)&client_addr, &client_len);
            if (client_fd < 0) continue;

            char ip_str[INET_ADDRSTRLEN];
            ::inet_ntop(AF_INET, &client_addr.sin_addr, ip_str, sizeof(ip_str));
            std::cerr << "rc_receiver: connected from " << ip_str << ":"
                      << ntohs(client_addr.sin_port) << std::endl;

            // TCP_NODELAY 降低延迟
            int flag = 1;
            ::setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

            // 连接建立 → 初始停车+舵机居中 (安全)
            if (!cfg.dry_run) {
                { std::string _e; driver.stop(_e); driver.servo(0, _e); }
            }
            last_seq = 0;
            had_command = false;
            last_cmd_time = std::chrono::steady_clock::now();
            continue;
        }

        // 轮询客户端数据 (50ms 超时)
        struct pollfd pfd;
        pfd.fd = client_fd;
        pfd.events = POLLIN;
        int ret = ::poll(&pfd, 1, 50);

        if (ret < 0) {
            if (errno == EINTR) continue;
            // poll 出错, 断开客户端
            ::close(client_fd);
            client_fd = -1;
            std::cerr << "rc_receiver: poll error, disconnected" << std::endl;
            if (!cfg.dry_run) driver.stop(_err);
            continue;
        }

        if (ret > 0 && (pfd.revents & POLLIN)) {
            // 读取指令
            RcCommand cmd;
            ssize_t n = ::read(client_fd, &cmd, sizeof(cmd));

            if (n <= 0) {
                // 连接断开
                ::close(client_fd);
                client_fd = -1;
                std::cerr << "rc_receiver: client disconnected" << std::endl;
                if (!cfg.dry_run) driver.stop(_err);
                continue;
            }

            if (static_cast<size_t>(n) != sizeof(cmd)) {
                // 部分读取 → 丢弃 (不正常)
                continue;
            }

            // 验证 magic
            if (cmd.magic != RC_MAGIC) continue;

            // 序列号检查: 收到的 seq 应 >= last_seq (排除重放过期指令)
            if (cmd.seq < last_seq && had_command) continue;
            last_seq = cmd.seq;
            had_command = true;
            last_cmd_time = std::chrono::steady_clock::now();

            // 限幅 speed_pct
            int speed = cmd.speed_pct;
            if (speed > 100) speed = 100;
            if (speed < -100) speed = -100;
            int servo_deg = cmd.servo_deg;
            if (servo_deg > 90) servo_deg = 90;
            if (servo_deg < -90) servo_deg = -90;
            int diff = cmd.diff_pct;
            if (diff > 100) diff = 100;
            if (diff < -100) diff = -100;

            if (cfg.dry_run) {
                std::cout << "rc_cmd: speed=" << speed << "% servo=" << servo_deg
                          << "° diff=" << diff << " seq=" << cmd.seq << std::endl;
                continue;
            }

            // ---- 计算左右轮速度 ----
            int left, right;

            if (speed == 0 && diff == 0) {
                // 停车
                left = right = 0;
            } else if (diff == 0) {
                // 无差速: 两轮同速 (舵机或直行)
                left = right = speed;
            } else {
                // 差速转向: 左轮 += diff, 右轮 -= diff
                // 舵机不动, 仅靠后轮速差转弯
                left  = speed + diff;
                right = speed - diff;
            }

            // 应用电机修正系数 (含限幅)
            applyMotorCorrections(left, right, cfg);

            // 驱动电机 + 舵机 (舵机不受差速影响)
            std::string error;
            if (!driver.set(left, right, error)) {
                std::cerr << "rc_receiver: set failed: " << error << std::endl;
            }
            if (!driver.servo(servo_deg, error)) {
                std::cerr << "rc_receiver: servo failed: " << error << std::endl;
            }
        }

        // 安全超时检查: 超过 stop_timeout_ms 没有收到指令 → 停车
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           now - last_cmd_time).count();
        if (had_command && elapsed > cfg.stop_timeout_ms) {
            if (!cfg.dry_run) {
                driver.stop(_err);
                driver.servo(0, _err);
            }
            had_command = false;
            std::cerr << "rc_receiver: command timeout (" << elapsed << "ms), stopped"
                      << std::endl;
        }
    }

    // 退出: 停车 + 清理
    if (!cfg.dry_run) {
        driver.stop(_err);
        driver.servo(0, _err);
    }
    driver.close();
    if (client_fd >= 0) ::close(client_fd);
    ::close(listen_fd);
    std::cerr << "rc_receiver: stopped" << std::endl;
    return 0;
}
