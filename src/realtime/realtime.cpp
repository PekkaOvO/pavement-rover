#include <iostream>
#include "realtime.h"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string>
#include <vector>

#include "common/angle.h"

#ifndef D2R
#define D2R (M_PI / 180.0)
#endif

namespace {

constexpr float GYRO_SENS_2000DPS = 16.4f;
constexpr float ACCEL_SENS_16G = 2048.0f;
constexpr float GRAVITY_M_S2 = 9.80665f;
constexpr float RAD_PER_DEG = 0.01745329251994329576923690768489f;
constexpr double DEFAULT_DT_SECONDS = 0.01;
constexpr size_t GPS_LINE_MAX = 1024;

speed_t baudToTermios(int baudrate) {
    switch (baudrate) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    default:     return B115200;
    }
}

void imuRawEnuToNed(float x_east, float y_north, float z_up,
                    float &north, float &east, float &down) {
    north = y_north;
    east = x_east;
    down = -z_up;
}

} // namespace

// ==================== RealtimeImuReader ====================
RealtimeImuReader::RealtimeImuReader() : fd_(-1), last_ts_ns_(0) {}

RealtimeImuReader::~RealtimeImuReader() { closeDevice(); }

bool RealtimeImuReader::openDevice(const std::string &devpath) {
    closeDevice();
    fd_ = open(devpath.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::perror("open imu device");
        return false;
    }
    last_ts_ns_ = 0;
    return true;
}

void RealtimeImuReader::closeDevice() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
    last_ts_ns_ = 0;
}

bool RealtimeImuReader::isOpen() const { return fd_ >= 0; }

bool RealtimeImuReader::readSample(IMU &imu) {
    if (fd_ < 0) return false;

    signed int databuf[7] = {0};
    const ssize_t want = static_cast<ssize_t>(sizeof(databuf));
    const ssize_t nread = read(fd_, databuf, sizeof(databuf));
    if (nread < 0) {
        std::perror("read imu device");
        return false;
    }
    if (!(nread == 0 || nread == want)) return false;

    const int64_t ts_ns = RealtimeGnssReader::monotonicNs();
    double dt = DEFAULT_DT_SECONDS;
    if (last_ts_ns_ > 0 && ts_ns > last_ts_ns_) {
        dt = static_cast<double>(ts_ns - last_ts_ns_) * 1e-9;
    }
    last_ts_ns_ = ts_ns;

    const float gx_rad_s = (static_cast<float>(databuf[0]) / GYRO_SENS_2000DPS) * RAD_PER_DEG;
    const float gy_rad_s = (static_cast<float>(databuf[1]) / GYRO_SENS_2000DPS) * RAD_PER_DEG;
    const float gz_rad_s = (static_cast<float>(databuf[2]) / GYRO_SENS_2000DPS) * RAD_PER_DEG;

    const float ax_m_s2 = (static_cast<float>(databuf[3]) / ACCEL_SENS_16G) * GRAVITY_M_S2;
    const float ay_m_s2 = (static_cast<float>(databuf[4]) / ACCEL_SENS_16G) * GRAVITY_M_S2;
    const float az_m_s2 = (static_cast<float>(databuf[5]) / ACCEL_SENS_16G) * GRAVITY_M_S2;

    float dtheta_n = 0.0f, dtheta_e = 0.0f, dtheta_d = 0.0f;
    float dv_n = 0.0f, dv_e = 0.0f, dv_d = 0.0f;

    imuRawEnuToNed(gx_rad_s * static_cast<float>(dt),
                   gy_rad_s * static_cast<float>(dt),
                   gz_rad_s * static_cast<float>(dt),
                   dtheta_n, dtheta_e, dtheta_d);

    imuRawEnuToNed(ax_m_s2 * static_cast<float>(dt),
                   ay_m_s2 * static_cast<float>(dt),
                   az_m_s2 * static_cast<float>(dt),
                   dv_n, dv_e, dv_d);

    imu.time = static_cast<double>(ts_ns) * 1e-9;
    imu.dt = dt;
    imu.dtheta << dtheta_n, dtheta_e, dtheta_d;
    imu.dvel << dv_n, dv_e, dv_d;

    return true;
}

// ==================== RealtimeGnssReader ====================
RealtimeGnssReader::RealtimeGnssReader() : fd_(-1), linebuf_(), has_gst_(false) {}

RealtimeGnssReader::~RealtimeGnssReader() { closeDevice(); }

bool RealtimeGnssReader::openDevice(const std::string &devpath, int baudrate) {
    closeDevice();
    fd_ = open(devpath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::perror("open gnss device");
        return false;
    }
    if (!configureSerial(baudrate)) {
        closeDevice();
        return false;
    }
    linebuf_.clear();
    has_gst_ = false;
    has_vtg_ = false;
    has_vtg_course_ = false;
    has_vtg_speed_ = false;
    return true;
}

void RealtimeGnssReader::closeDevice() {
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
    linebuf_.clear();
    has_gst_ = false;
    has_vtg_ = false;
    has_vtg_course_ = false;
    has_vtg_speed_ = false;
}

bool RealtimeGnssReader::isOpen() const { return fd_ >= 0; }

bool RealtimeGnssReader::configureSerial(int baudrate) {
    struct termios tio;
    if (tcgetattr(fd_, &tio) < 0) {
        std::perror("tcgetattr");
        return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    const speed_t spd = baudToTermios(baudrate);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    if (tcsetattr(fd_, TCSANOW, &tio) < 0) {
        std::perror("tcsetattr");
        return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
}

bool RealtimeGnssReader::pollSample(GNSS &gnss, const std::vector<double> &default_std) {
    if (fd_ < 0) return false;

    char rx[256];
    while (true) {
        const ssize_t nread = read(fd_, rx, sizeof(rx));
        if (nread < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
            std::perror("read gnss device");
            return false;
        }
        if (nread == 0) return false;

        for (ssize_t i = 0; i < nread; ++i) {
            const char c = rx[i];
            if (c == '\r') continue;
            if (c == '\n') {
                std::string line = linebuf_;
                linebuf_.clear();
                if (!line.empty() && handleLine(line, gnss, default_std)) {
                    return true;
                }
                continue;
            }
            if (linebuf_.size() < GPS_LINE_MAX - 1) {
                linebuf_.push_back(c);
            } else {
                linebuf_.clear();
            }
        }
    }
}

void RealtimeGnssReader::parseAndCacheGST(const std::string &line) {
    // 按逗号切分，保留空字段
    std::vector<std::string> fields;
    fields.reserve(16);

    size_t start = 0;
    while (true) {
        size_t pos = line.find(',', start);
        if (pos == std::string::npos) {
            fields.emplace_back(line.substr(start));
            break;
        }
        fields.emplace_back(line.substr(start, pos - start));
        start = pos + 1;
    }

    auto parseDouble = [](const std::string &s) -> double {
        if (s.empty()) return 0.0;
        size_t star = s.find('*');
        std::string num = (star == std::string::npos) ? s : s.substr(0, star);
        if (num.empty()) return 0.0;
        return std::strtod(num.c_str(), nullptr);
    };

    // 直接取最后3个字段作为 std
    if (fields.size() >= 9) {
        gst_std_n_ = parseDouble(fields[fields.size() - 3]);
        gst_std_e_ = parseDouble(fields[fields.size() - 2]);
        gst_std_d_ = parseDouble(fields[fields.size() - 1]);
        has_gst_ = true;
    }
}

void RealtimeGnssReader::parseAndCacheVTG(const std::string &line) {
    std::vector<std::string> fields;
    fields.reserve(12);

    size_t start = 0;
    while (true) {
        size_t pos = line.find(',', start);
        if (pos == std::string::npos) {
            fields.emplace_back(line.substr(start));
            break;
        }
        fields.emplace_back(line.substr(start, pos - start));
        start = pos + 1;
    }

    auto parseDouble = [](const std::string &s, double &out) -> bool {
        if (s.empty()) return false;
        size_t star = s.find('*');
        std::string num = (star == std::string::npos) ? s : s.substr(0, star);
        if (num.empty()) return false;
        char *end = nullptr;
        out = std::strtod(num.c_str(), &end);
        return end != num.c_str() && *end == '\0';
    };

    if (fields.size() < 8) return;

    double course_deg = 0.0;
    double speed_kph = 0.0;
    bool has_course = parseDouble(fields[1], course_deg);
    bool has_speed = parseDouble(fields[7], speed_kph);

    if (!has_speed && fields.size() >= 6) {
        double speed_knots = 0.0;
        if (parseDouble(fields[5], speed_knots)) {
            speed_kph = speed_knots * 1.852;
            has_speed = true;
        }
    }

    if (has_course) {
        while (course_deg >= 360.0) course_deg -= 360.0;
        while (course_deg < 0.0) course_deg += 360.0;
        vtg_course_deg_ = course_deg;
    }
    if (has_speed) {
        vtg_speed_kph_ = speed_kph;
        vtg_speed_mps_ = speed_kph / 3.6;
    }
    has_vtg_course_ = has_course;
    has_vtg_speed_ = has_speed;
    has_vtg_ = has_course || has_speed;
}

bool RealtimeGnssReader::handleLine(const std::string &line, GNSS &gnss, const std::vector<double> &default_std) {
    // 处理 GST 电文（仅缓存，不输出定位）
    if (line.rfind("$GPGST", 0) == 0 || line.rfind("$GNGST", 0) == 0 || line.rfind("$GBGST", 0) == 0) {
        parseAndCacheGST(line);
        return false;   // GST 不是完整定位样本
    }

    // 处理 GGA 电文
    if (line.rfind("$GPVTG", 0) == 0 || line.rfind("$GNVTG", 0) == 0 || line.rfind("$GBVTG", 0) == 0) {
        parseAndCacheVTG(line);
        return false;
    }

    if (!(line.rfind("$GPGGA", 0) == 0 || line.rfind("$GNGGA", 0) == 0 || line.rfind("$GBGGA", 0) == 0)) {
        return false;
    }

    char buf[GPS_LINE_MAX];
    std::snprintf(buf, sizeof(buf), "%s", line.c_str());

    char *fields[20] = {nullptr};
    int field_count = 0;
    char *saveptr = nullptr;
    char *token = strtok_r(buf, ",", &saveptr);
    while (token != nullptr && field_count < 20) {
        fields[field_count++] = token;
        token = strtok_r(nullptr, ",", &saveptr);
    }
    // 需要至少12个字段才能获取大地水准面高
    if (field_count < 12) return false;

    const char *lat_str = fields[2];
    const char *lat_dir = fields[3];
    const char *lon_str = fields[4];
    const char *lon_dir = fields[5];
    const char *alt_msl_str = fields[9];    // 海拔高 (MSL)
    const char *geoid_str = fields[11];     // 大地水准面高

    if (!lat_str || !lat_dir || !lon_str || !lon_dir || !alt_msl_str || !geoid_str) return false;
    if (lat_str[0]=='\0' || lat_dir[0]=='\0' || lon_str[0]=='\0' || lon_dir[0]=='\0' ||
        alt_msl_str[0]=='\0' || geoid_str[0]=='\0') return false;

    const double lat_deg = nmeaDegMinToDeg(lat_str, lat_dir[0]);
    const double lon_deg = nmeaDegMinToDeg(lon_str, lon_dir[0]);
    const double alt_ellipsoid = std::atof(alt_msl_str) + std::atof(geoid_str);  // 椭球高

    gnss.time = static_cast<double>(monotonicNs()) * 1e-9;
    gnss.blh << lat_deg * D2R, lon_deg * D2R, alt_ellipsoid;
    gnss.isvalid = true;
    gnss.has_course = has_vtg_course_;
    gnss.has_speed = has_vtg_speed_;
    gnss.course_deg = vtg_course_deg_;
    gnss.speed_mps = vtg_speed_mps_;
    gnss.speed_kph = vtg_speed_kph_;

    // 使用 GST 缓存的标准差（如果有），否则使用传入的默认值
    if (has_gst_) {
        gnss.std << gst_std_n_, gst_std_e_, gst_std_d_;
        // 注意：不自动清空缓存，因为 GST 通常每秒输出一次，与 GGA 对齐
    } else {
        if (default_std.size() == 3)
            gnss.std << default_std[0], default_std[1], default_std[2];
        else
            gnss.std << 0.1, 0.1, 0.1;
    }

    return true;
}

double RealtimeGnssReader::nmeaDegMinToDeg(const char *s, char dir) {
    if (s == nullptr || *s == '\0') return 0.0;
    const double val = std::atof(s);
    const int deg = static_cast<int>(val / 100.0);
    const double min = val - static_cast<double>(deg) * 100.0;
    double out = static_cast<double>(deg) + min / 60.0;
    if (dir == 'S' || dir == 'W') out = -out;
    return out;
}

int64_t RealtimeGnssReader::monotonicNs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
