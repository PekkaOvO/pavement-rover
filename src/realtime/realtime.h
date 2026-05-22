#ifndef REALTIME_IO_H
#define REALTIME_IO_H

#include <cstdint>
#include <string>
#include <vector>

#include "common/types.h"

class RealtimeImuReader {
public:
    RealtimeImuReader();
    ~RealtimeImuReader();

    bool openDevice(const std::string &devpath);
    void closeDevice();
    bool isOpen() const;

    bool readSample(IMU &imu);

private:
    int fd_;
    int64_t last_ts_ns_;
};

class RealtimeGnssReader {
public:
    RealtimeGnssReader();
    ~RealtimeGnssReader();

    bool openDevice(const std::string &devpath, int baudrate);
    void closeDevice();
    bool isOpen() const;

    bool pollSample(GNSS &gnss, const std::vector<double> &default_std);

    static int64_t monotonicNs();

private:
    bool configureSerial(int baudrate);
    bool handleLine(const std::string &line, GNSS &gnss, const std::vector<double> &default_std);
    static double nmeaDegMinToDeg(const char *s, char dir);
    void parseAndCacheVTG(const std::string &line);
    void parseAndCacheGST(const std::string &line);  // 解析GST并缓存标准差

private:
    int fd_;
    std::string linebuf_;

    // GST 缓存
    bool has_gst_{false};
    double gst_std_n_{0.0};
    double gst_std_e_{0.0};
    double gst_std_d_{0.0};

    bool has_vtg_{false};
    bool has_vtg_course_{false};
    bool has_vtg_speed_{false};
    double vtg_course_deg_{0.0};
    double vtg_speed_mps_{0.0};
    double vtg_speed_kph_{0.0};
};

#endif
