/**
 * test_static_gps.cpp — 小车静止测试程序
 *
 * 模拟静止小车的 GPS 位置，直接通过 TCP 发送到上位机 CarView2。
 * 不依赖 KF-GINS、GNSS、IMU、GD32 MCU 等任何硬件。
 *
 * 用法:
 *   ./test_static_gps [server_ip] [server_port]
 *   默认: 192.168.1.100:8766
 *
 * 编译 (在IMX6ULL上):
 *   g++ -std=c++11 -O2 -Wall -o test_static_gps test_static_gps.cpp
 *
 * 也可以用交叉编译:
 *   arm-linux-gnueabihf-g++ -std=c++11 -O2 -Wall -o test_static_gps test_static_gps.cpp
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <csignal>
#include <algorithm>

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

// ============================================================================
// 可配置参数
// ============================================================================

// 静止小车位置 (WGS84 经纬度)
// 默认: 上海市某处 (可根据实际测试位置修改)
static double g_lat_deg  = 31.2304;
static double g_lon_deg  = 121.4737;
static double g_height_m = 10.0;
static double g_course_deg = 0.0;    // 航向角 (0=正北)
static double g_speed_mps  = 0.0;    // 速度 (静止=0)
static double g_cam_angle_deg = 0.0; // 摄像头角度

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
// 构建一个完整的数据包
// ============================================================================

static size_t buildPacket(uint32_t seq, uint8_t *pkt, size_t pkt_size) {
    // TYPE_NO_IMAGE 包: header(52) + crc(2) = 54 bytes total
    const size_t total = kHeaderSize + kCrcSize;
    if (pkt_size < total) return 0;

    memset(pkt, 0, total);

    uint64_t ts_ms = getTimeMs();

    // MAGIC
    writeBe16(pkt + OFF_MAGIC, kMagic);
    // VERSION
    pkt[OFF_VERSION] = kVersion;
    // TYPE (no image)
    pkt[OFF_TYPE] = kTypeNoImage;
    // SEQ
    writeBe32(pkt + OFF_SEQ, seq);
    // TS_MS
    writeBe64(pkt + OFF_TS_MS, ts_ms);
    // LAT
    writeBeDouble(pkt + OFF_LAT, g_lat_deg);
    // LON
    writeBeDouble(pkt + OFF_LON, g_lon_deg);
    // COURSE
    writeBeFloat(pkt + OFF_COURSE, (float)g_course_deg);
    // SPEED
    writeBeFloat(pkt + OFF_SPEED, (float)g_speed_mps);
    // HEIGHT
    writeBeFloat(pkt + OFF_HEIGHT, (float)g_height_m);
    // CAM_ANGLE
    writeBeFloat(pkt + OFF_CAM_ANGLE, (float)g_cam_angle_deg);
    // IMAGE_LEN = 0
    writeBe32(pkt + OFF_IMAGE_LEN, 0);

    // CRC16 (覆盖 VERSION 到 IMAGE_LEN，即 header 去掉 MAGIC)
    uint16_t crc = crc16Ccitt(pkt + OFF_VERSION, kHeaderSize - OFF_VERSION);
    writeBe16(pkt + kHeaderSize, crc);

    return total;
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
    // 解析参数
    const char *server_ip   = (argc >= 2) ? argv[1] : "192.168.1.100";
    uint16_t    server_port = (argc >= 3) ? (uint16_t)std::max(1, std::atoi(argv[2])) : 8766;

    printf("=============================================\n");
    printf("  小车静止 GPS 测试程序\n");
    printf("=============================================\n");
    printf("  上位机:    %s:%u\n", server_ip, server_port);
    printf("  纬度:      %.6f°\n", g_lat_deg);
    printf("  经度:      %.6f°\n", g_lon_deg);
    printf("  高度:      %.1f m\n", g_height_m);
    printf("  航向:      %.1f°\n", g_course_deg);
    printf("  速度:      %.1f m/s (静止)\n", g_speed_mps);
    printf("  发送间隔:  %d ms\n", g_interval_ms);
    printf("  包类型:    TYPE_NO_IMAGE (0x02)\n");
    printf("=============================================\n");

    // 注册信号处理
    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);

    // 创建 TCP socket
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        fprintf(stderr, "ERROR: socket() failed: %s\n", strerror(errno));
        return 1;
    }

    // 设置 TCP_NODELAY (降低延迟)
    int flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    // 连接上位机
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(server_port);
    if (inet_pton(AF_INET, server_ip, &addr.sin_addr) != 1) {
        fprintf(stderr, "ERROR: invalid IP address: %s\n", server_ip);
        close(fd);
        return 1;
    }

    printf("\n正在连接 %s:%u ...\n", server_ip, server_port);
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "ERROR: connect() failed: %s\n", strerror(errno));
        fprintf(stderr, "请确认:\n");
        fprintf(stderr, "  1. CarView2 已启动并监听端口 %u\n", server_port);
        fprintf(stderr, "  2. IP 地址 %s 可达\n", server_ip);
        fprintf(stderr, "  3. 防火墙未阻止连接\n");
        close(fd);
        return 1;
    }
    printf("连接成功!\n\n");

    // 发送循环
    uint32_t seq = 0;
    uint64_t total_sent = 0;
    uint64_t total_bytes = 0;
    uint64_t last_print_ts = getTimeMs();

    uint8_t pkt[256];
    const size_t pkt_size = buildPacket(0, pkt, sizeof(pkt));
    printf("每包大小: %zu bytes (header=%zu + crc=%zu)\n\n", pkt_size, kHeaderSize, kCrcSize);

    while (!g_stop) {
        // 构建并发送一个包
        size_t len = buildPacket(seq, pkt, sizeof(pkt));

        ssize_t n = write(fd, pkt, len);
        if (n < 0) {
            if (errno == EINTR) continue;
            fprintf(stderr, "ERROR: write() failed: %s (连接断开?)\n", strerror(errno));
            break;
        }
        if ((size_t)n != len) {
            fprintf(stderr, "WARNING: 只发送了 %zd/%zu 字节\n", n, len);
        }

        seq++;
        total_sent++;
        total_bytes += len;

        // 每秒打印一次状态
        uint64_t now = getTimeMs();
        if (now - last_print_ts >= 1000) {
            printf("\r[seq=%u] 已发送 %lu 包 (%lu KB)",
                   seq, (unsigned long)total_sent, (unsigned long)(total_bytes / 1024));
            fflush(stdout);
            last_print_ts = now;
        }

        // 等待间隔
        usleep((useconds_t)g_interval_ms * 1000);
    }

    printf("\n\n正在关闭...\n");
    close(fd);
    printf("已停止。共发送 %lu 包 (%lu KB)\n",
           (unsigned long)total_sent, (unsigned long)(total_bytes / 1024));
    return 0;
}
