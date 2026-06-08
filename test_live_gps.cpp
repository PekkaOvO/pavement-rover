/**
 * test_live_gps.cpp — 实时 GPS 数据发送程序
 *
 * 从串口 /dev/ttyUSB0 读取 GNSS NMEA 数据，解析后通过 TCP 发送到上位机 CarView2。
 * 支持静态坐标回退模式（无 GPS 硬件时使用固定坐标）。
 *
 * 用法:
 *   ./test_live_gps [server_ip] [server_port]
 *   默认: 172.20.10.3:8766
 *
 * 编译:
 *   g++ -std=c++11 -O2 -Wall -pthread -o test_live_gps test_live_gps.cpp
 *   交叉编译:
 *   arm-linux-gnueabihf-g++ -std=c++11 -O2 -Wall -pthread -o test_live_gps test_live_gps.cpp
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <csignal>
#include <algorithm>
#include <mutex>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

// ============================================================================
// 可配置参数
// ============================================================================

// GPS 串口配置
static const char *g_serial_dev   = "/dev/ttyUSB0";
static const int   g_serial_baud  = 115200;

// 回退静态坐标 (无 GPS 时使用)
static double g_fallback_lat_deg  = 38.8692;
static double g_fallback_lon_deg  = 121.5221;
static double g_fallback_height_m = 26.8;

// 发送间隔 (毫秒)，默认 200ms = 5Hz
static int g_interval_ms = 200;

// ============================================================================
// TCP 协议常量 (与 gd32_bridge::TcpProtocol 保持一致)
// ============================================================================

static constexpr uint16_t kMagic       = 0xAA55;
static constexpr uint8_t  kVersion     = 0x01;
static constexpr uint8_t  kTypeNoImage = 0x02;   // 无图像帧
static constexpr size_t   kHeaderSize  = 52;
static constexpr size_t   kCrcSize     = 2;

// 头部各字段偏移
static constexpr size_t OFF_MAGIC     = 0;
static constexpr size_t OFF_VERSION   = 2;
static constexpr size_t OFF_TYPE      = 3;
static constexpr size_t OFF_SEQ       = 4;
static constexpr size_t OFF_TS_MS     = 8;
static constexpr size_t OFF_LAT       = 16;
static constexpr size_t OFF_LON       = 24;
static constexpr size_t OFF_COURSE    = 32;
static constexpr size_t OFF_SPEED     = 36;
static constexpr size_t OFF_HEIGHT    = 40;
static constexpr size_t OFF_CAM_ANGLE = 44;
static constexpr size_t OFF_IMAGE_LEN = 48;

// ============================================================================
// GPS 状态 (线程安全)
// ============================================================================

struct GpsState {
    double   lat_deg;
    double   lon_deg;
    double   height_m;
    double   course_deg;
    double   speed_mps;
    int      fix_quality;    // 0=无效, 1=GPS, 2=DGPS
    int      satellites;
    bool     has_fix;

    GpsState() : lat_deg(0), lon_deg(0), height_m(0),
                 course_deg(0), speed_mps(0),
                 fix_quality(0), satellites(0), has_fix(false) {}
};

static GpsState g_gps;
static std::mutex g_gps_mutex;

// ============================================================================
// CRC16-CCITT (与 gd32_bridge 一致)
// ============================================================================

static uint16_t crc16Ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ============================================================================
// 大端序写入
// ============================================================================

static void writeBe16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)(v);
}

static void writeBe32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)(v);
}

static void writeBe64(uint8_t *p, uint64_t v) {
    for (int i = 0; i < 8; ++i)
        p[i] = (uint8_t)(v >> (56 - i * 8));
}

static void writeBeFloat(uint8_t *p, float v) {
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    writeBe32(p, bits);
}

static void writeBeDouble(uint8_t *p, double v) {
    uint64_t bits;
    memcpy(&bits, &v, sizeof(bits));
    writeBe64(p, bits);
}

// ============================================================================
// 获取当前时间 (ms since epoch)
// ============================================================================

static uint64_t getTimeMs() {
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
}

// ============================================================================
// 构建一个完整的数据包 (使用当前 GPS 状态)
// ============================================================================

static size_t buildPacket(uint32_t seq, uint8_t *pkt, size_t pkt_size, const GpsState &gps) {
    const size_t total = kHeaderSize + kCrcSize;
    if (pkt_size < total) return 0;

    memset(pkt, 0, total);

    uint64_t ts_ms = getTimeMs();

    writeBe16(pkt + OFF_MAGIC, kMagic);
    pkt[OFF_VERSION] = kVersion;
    pkt[OFF_TYPE] = kTypeNoImage;
    writeBe32(pkt + OFF_SEQ, seq);
    writeBe64(pkt + OFF_TS_MS, ts_ms);
    writeBeDouble(pkt + OFF_LAT, gps.lat_deg);
    writeBeDouble(pkt + OFF_LON, gps.lon_deg);
    writeBeFloat(pkt + OFF_COURSE, (float)gps.course_deg);
    writeBeFloat(pkt + OFF_SPEED, (float)gps.speed_mps);
    writeBeFloat(pkt + OFF_HEIGHT, (float)gps.height_m);
    writeBeFloat(pkt + OFF_CAM_ANGLE, 0.0f);  // 无摄像头
    writeBe32(pkt + OFF_IMAGE_LEN, 0);

    // CRC16 (覆盖 VERSION 到 IMAGE_LEN)
    uint16_t crc = crc16Ccitt(pkt + OFF_VERSION, kHeaderSize - OFF_VERSION);
    writeBe16(pkt + kHeaderSize, crc);

    return total;
}

// ============================================================================
// NMEA 解析
// ============================================================================

// 转换 NMEA 坐标 (DDMM.MMMMM 或 DDDMM.MMMMM) 为十进制度数
static double nmeaToDeg(double raw, char hemisphere) {
    // raw = DDMM.MMMMM, 提取度和分
    int deg = (int)(raw / 100.0);
    double minutes = raw - deg * 100.0;
    double result = deg + minutes / 60.0;
    if (hemisphere == 'S' || hemisphere == 'W')
        result = -result;
    return result;
}

// 解析 $xxRMC 语句
// $GNRMC,time,status,lat,N/S,lon,E/W,speed,course,date,magvar,magdir,mode*cs
static bool parseRMC(const char *sentence, GpsState &gps) {
    // 跳过 $xxRMC, 前缀 (7 chars: $ + 2 talker + RMC + ,)
    const char *p = sentence;
    // 找到第一个逗号后面
    p = strchr(p, ',');
    if (!p) return false;
    p++; // 跳过逗号，现在在 time 字段

    // 用 strtok_r 风格解析，但用简单的逗号遍历更安全
    // field 0: time
    // field 1: status
    // field 2: lat
    // field 3: N/S
    // field 4: lon
    // field 5: E/W
    // field 6: speed (knots)
    // field 7: course (degrees)
    // ...

    const char *fields[12];
    int n = 0;
    const char *start = p;
    while (n < 12) {
        const char *end = strchr(start, ',');
        fields[n++] = start;
        if (!end) break;
        start = end + 1;
        if (*start == '*') break; // checksum
    }

    if (n < 8) return false;

    // Status: 'A'=valid, 'V'=invalid
    if (fields[0][0] != 'A') return false;

    // Parse lat/lon
    double lat_raw = atof(fields[1]);
    char ns = fields[2][0];
    double lon_raw = atof(fields[3]);
    char ew = fields[4][0];

    if (ns == '\0' || ew == '\0') return false;
    if (lat_raw == 0.0 && lon_raw == 0.0) return false;

    gps.lat_deg = nmeaToDeg(lat_raw, ns);
    gps.lon_deg = nmeaToDeg(lon_raw, ew);

    // Speed (knots → m/s)
    double speed_kn = atof(fields[5]);
    gps.speed_mps = speed_kn * 0.514444;

    // Course (degrees)
    gps.course_deg = atof(fields[6]);

    gps.has_fix = true;
    return true;
}

// 解析 $xxGGA 语句
// $GNGGA,time,lat,N/S,lon,E/W,quality,num_sats,hdop,alt,M,geoid,M,age,refid*cs
static bool parseGGA(const char *sentence, GpsState &gps) {
    const char *p = strchr(sentence, ',');
    if (!p) return false;
    p++; // 现在在 time 字段

    const char *fields[14];
    int n = 0;
    const char *start = p;
    while (n < 14) {
        const char *end = strchr(start, ',');
        fields[n++] = start;
        if (!end) break;
        start = end + 1;
        if (*start == '*') break;
    }

    if (n < 9) return false;

    // Quality: 0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
    int quality = atoi(fields[5]);
    if (quality == 0) return false;

    gps.fix_quality = quality;

    // Satellites
    gps.satellites = atoi(fields[6]);

    // Altitude (meters)
    gps.height_m = atof(fields[7]);

    // Also update lat/lon from GGA if RMC hasn't already
    if (fields[1][0] != '\0' && fields[3][0] != '\0') {
        double lat_raw = atof(fields[1]);
        char ns = fields[2][0];
        double lon_raw = atof(fields[3]);
        char ew = fields[4][0];
        if (ns != '\0' && ew != '\0') {
            gps.lat_deg = nmeaToDeg(lat_raw, ns);
            gps.lon_deg = nmeaToDeg(lon_raw, ew);
            gps.has_fix = true;
        }
    }

    return true;
}

// 解析一行 NMEA 数据，更新 gps 状态
static void parseNmeaLine(const char *line, int len) {
    if (len < 10) return;

    // 检查是否以 $xxRMC 或 $xxGGA 开头
    const char *talker_end = strchr(line + 1, ','); // 第一个逗号在 talker 后面
    if (!talker_end) return;

    // 句子类型在 talker 的倒数3个字符 (e.g., "$GNRMC" → "RMC")
    int sentence_start = (int)(talker_end - line);
    if (sentence_start < 5) return;

    const char *sentence_type = line + sentence_start - 3;
    // sentence_type 指向类似 "RMC," 的开头

    GpsState gps;
    bool updated = false;

    if (strncmp(sentence_type, "RMC", 3) == 0) {
        updated = parseRMC(line, gps);
    } else if (strncmp(sentence_type, "GGA", 3) == 0) {
        updated = parseGGA(line, gps);
    }

    if (updated) {
        std::lock_guard<std::mutex> lock(g_gps_mutex);
        g_gps = gps;
    }
}

// ============================================================================
// 串口初始化
// ============================================================================

static int openSerial(const char *dev, int baud) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "WARNING: 无法打开串口 %s: %s\n", dev, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) < 0) {
        fprintf(stderr, "WARNING: tcgetattr() 失败: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    // 波特率
    speed_t speed = B115200;
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:     speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1, 无流控, 原始模式
    tty.c_cflag &= ~PARENB;        // 无校验
    tty.c_cflag &= ~CSTOPB;        // 1 停止位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 数据位
    tty.c_cflag &= ~CRTSCTS;       // 无硬件流控
    tty.c_cflag |= CREAD | CLOCAL; // 启用接收，忽略调制解调器控制线

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG); // 原始模式
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);                   // 无软件流控
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;                                    // 原始输出

    // 读取超时: 0.1s
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) < 0) {
        fprintf(stderr, "WARNING: tcsetattr() 失败: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    // 清空缓冲区
    tcflush(fd, TCIOFLUSH);

    return fd;
}

// ============================================================================
// 全局状态
// ============================================================================

static volatile bool g_stop = false;

static void onSignal(int) {
    g_stop = true;
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char **argv) {
    const char *server_ip   = (argc >= 2) ? argv[1] : "172.20.10.3";
    uint16_t    server_port = (argc >= 3) ? (uint16_t)std::max(1, std::atoi(argv[2])) : 8766;

    printf("=============================================\n");
    printf("  实时 GPS → CarView2 数据发送程序\n");
    printf("=============================================\n");
    printf("  GPS 串口:  %s (%d baud)\n", g_serial_dev, g_serial_baud);
    printf("  上位机:    %s:%u\n", server_ip, server_port);
    printf("  发送间隔:  %d ms (%.1f Hz)\n", g_interval_ms, 1000.0 / g_interval_ms);
    printf("  回退坐标:  %.6f°, %.6f°\n", g_fallback_lat_deg, g_fallback_lon_deg);
    printf("  包类型:    TYPE_NO_IMAGE (0x02)\n");
    printf("=============================================\n\n");

    // 注册信号处理
    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);

    // 打开 GPS 串口
    int serial_fd = openSerial(g_serial_dev, g_serial_baud);
    if (serial_fd < 0) {
        printf("⚠ 未检测到 GPS 硬件，使用回退静态坐标\n\n");
    } else {
        printf("✓ GPS 串口 %s 已打开 (%d baud)\n", g_serial_dev, g_serial_baud);
    }

    // 创建 TCP socket
    int tcp_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_fd < 0) {
        fprintf(stderr, "ERROR: socket() failed: %s\n", strerror(errno));
        if (serial_fd >= 0) close(serial_fd);
        return 1;
    }

    int flag = 1;
    setsockopt(tcp_fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(server_port);
    if (inet_pton(AF_INET, server_ip, &addr.sin_addr) != 1) {
        fprintf(stderr, "ERROR: invalid IP address: %s\n", server_ip);
        close(tcp_fd);
        if (serial_fd >= 0) close(serial_fd);
        return 1;
    }

    printf("正在连接 %s:%u ...\n", server_ip, server_port);
    if (connect(tcp_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "ERROR: connect() failed: %s\n", strerror(errno));
        close(tcp_fd);
        if (serial_fd >= 0) close(serial_fd);
        return 1;
    }
    printf("✓ TCP 已连接到上位机\n\n");

    // 主循环: 读串口 → 解析 NMEA → 发送 TCP
    uint32_t seq = 0;
    uint64_t total_sent = 0;
    uint64_t total_bytes = 0;
    uint64_t last_print_ts = getTimeMs();
    int nmea_updates = 0;

    char nmea_buf[512];
    int  nmea_pos = 0;

    uint8_t pkt[256];

    // 用回退坐标初始化 GPS 状态
    {
        std::lock_guard<std::mutex> lock(g_gps_mutex);
        g_gps.lat_deg  = g_fallback_lat_deg;
        g_gps.lon_deg  = g_fallback_lon_deg;
        g_gps.height_m = g_fallback_height_m;
        g_gps.course_deg = 0;
        g_gps.speed_mps  = 0;
        g_gps.has_fix    = false;
    }

    printf("开始发送数据 (Ctrl+C 停止)...\n\n");

    while (!g_stop) {
        // ---- 读取串口数据 ----
        if (serial_fd >= 0) {
            uint8_t buf[256];
            ssize_t n = read(serial_fd, buf, sizeof(buf));
            if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) {
                    char c = (char)buf[i];
                    if (c == '\n' || c == '\r') {
                        if (nmea_pos > 0) {
                            nmea_buf[nmea_pos] = '\0';
                            parseNmeaLine(nmea_buf, nmea_pos);
                            nmea_updates++;
                            nmea_pos = 0;
                        }
                    } else if (nmea_pos < (int)sizeof(nmea_buf) - 1) {
                        nmea_buf[nmea_pos++] = c;
                    }
                }
            }
        }

        // ---- 获取最新 GPS 状态并发送 ----
        GpsState current;
        {
            std::lock_guard<std::mutex> lock(g_gps_mutex);
            current = g_gps;
        }

        size_t len = buildPacket(seq, pkt, sizeof(pkt), current);

        ssize_t sent = write(tcp_fd, pkt, len);
        if (sent < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "ERROR: write() failed: %s\n", strerror(errno));
            break;
        }

        seq++;
        total_sent++;
        total_bytes += len;

        // 每秒打印一次状态
        uint64_t now = getTimeMs();
        if (now - last_print_ts >= 1000) {
            printf("\r[#%u] 发送 %lu 包 | 坐标: %.6f,%.6f | 高度 %.1fm | "
                   "航向 %.1f° | 速度 %.2fm/s | 定位: %s | 卫星: %d | NMEA: %d行/s  ",
                   seq,
                   (unsigned long)total_sent,
                   current.lat_deg, current.lon_deg,
                   current.height_m,
                   current.course_deg,
                   current.speed_mps,
                   current.has_fix ? "✓有效" : "△回退",
                   current.satellites,
                   nmea_updates);
            fflush(stdout);
            last_print_ts = now;
            nmea_updates = 0;
        }

        usleep((useconds_t)g_interval_ms * 1000);
    }

    printf("\n\n正在关闭...\n");
    close(tcp_fd);
    if (serial_fd >= 0) close(serial_fd);
    printf("已停止。共发送 %lu 包 (%lu KB)\n",
           (unsigned long)total_sent, (unsigned long)(total_bytes / 1024));
    return 0;
}
