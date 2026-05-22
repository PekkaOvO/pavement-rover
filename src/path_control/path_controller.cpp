#include "path_controller.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <yaml-cpp/yaml.h>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

#ifndef O_CLOEXEC
#define O_CLOEXEC 0
#endif

namespace path_control {
namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double EARTH_RADIUS_M = 6378137.0;

double degToRad(double deg) {
    return deg * PI / 180.0;
}

double normalizeAngle(double angle) {
    while (angle > PI) {
        angle -= 2.0 * PI;
    }
    while (angle < -PI) {
        angle += 2.0 * PI;
    }
    return angle;
}

double pointDistance(Point2d a, Point2d b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

int roundToInt(double value) {
    if (value >= 0.0) {
        return static_cast<int>(value + 0.5);
    }
    return static_cast<int>(value - 0.5);
}

int clampInt(int value, int min_value, int max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

double clampDouble(double value, double min_value, double max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

std::string trim(const std::string &text) {
    size_t first = 0;
    while (first < text.size() &&
           std::isspace(static_cast<unsigned char>(text[first])) != 0) {
        ++first;
    }

    size_t last = text.size();
    while (last > first &&
           std::isspace(static_cast<unsigned char>(text[last - 1])) != 0) {
        --last;
    }

    return text.substr(first, last - first);
}

std::vector<std::string> splitCsv(const std::string &line) {
    std::vector<std::string> fields;
    std::string field;
    std::stringstream ss(line);
    while (std::getline(ss, field, ',')) {
        fields.push_back(trim(field));
    }
    return fields;
}

std::vector<std::string> splitGnssFields(const std::string &line) {
    std::vector<std::string> fields;
    std::string field;
    for (size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == ',' || c == ';' || std::isspace(static_cast<unsigned char>(c)) != 0) {
            if (!field.empty()) {
                fields.push_back(field);
                field.clear();
            }
            continue;
        }
        field.push_back(c);
    }
    if (!field.empty()) {
        fields.push_back(field);
    }
    return fields;
}

bool parseDouble(const std::string &text, double &out) {
    const std::string value = trim(text);
    char *end = nullptr;
    errno = 0;
    out = std::strtod(value.c_str(), &end);
    return errno == 0 && end != value.c_str() && *end == '\0';
}

bool parseInt(const std::string &text, int &out) {
    const std::string value = trim(text);
    char *end = nullptr;
    errno = 0;
    const long parsed = std::strtol(value.c_str(), &end, 10);
    if (errno != 0 || end == value.c_str() || *end != '\0') {
        return false;
    }
    out = static_cast<int>(parsed);
    return true;
}

std::string stripNmeaChecksum(const std::string &text) {
    const size_t star = text.find('*');
    if (star == std::string::npos) {
        return text;
    }
    return text.substr(0, star);
}

bool parseNmeaDouble(const std::string &text, double &out) {
    return parseDouble(stripNmeaChecksum(text), out);
}

bool isPrefix(const std::string &field, const char *prefix) {
    const std::string trimmed = trim(field);
    const size_t prefix_len = std::strlen(prefix);
    if (trimmed.size() != prefix_len) {
        return false;
    }
    for (size_t i = 0; i < prefix_len; ++i) {
        const int a = std::toupper(static_cast<unsigned char>(trimmed[i]));
        const int b = std::toupper(static_cast<unsigned char>(prefix[i]));
        if (a != b) {
            return false;
        }
    }
    return true;
}

bool looksLikeLatLon(double lat_deg, double lon_deg) {
    return std::isfinite(lat_deg) && std::isfinite(lon_deg) &&
           lat_deg >= -90.0 && lat_deg <= 90.0 &&
           lon_deg >= -180.0 && lon_deg <= 180.0;
}

bool isNmeaType(const std::string &field, const char *type) {
    const std::string value = trim(field);
    if (value.size() < 6 || value[0] != '$') {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        const int a = std::toupper(static_cast<unsigned char>(value[3 + i]));
        const int b = std::toupper(static_cast<unsigned char>(type[i]));
        if (a != b) {
            return false;
        }
    }
    return true;
}

double normalizeHeadingDeg(double heading_deg) {
    while (heading_deg >= 360.0) {
        heading_deg -= 360.0;
    }
    while (heading_deg < 0.0) {
        heading_deg += 360.0;
    }
    return heading_deg;
}

double nmeaDegMinToDeg(const std::string &text, char dir) {
    double value = 0.0;
    if (!parseNmeaDouble(text, value)) {
        return 0.0;
    }
    const int deg = static_cast<int>(value / 100.0);
    const double min = value - static_cast<double>(deg) * 100.0;
    double out = static_cast<double>(deg) + min / 60.0;
    if (dir == 'S' || dir == 'W') {
        out = -out;
    }
    return out;
}

bool parseNmeaTimeSeconds(const std::string &text, double &seconds) {
    double raw = 0.0;
    if (!parseNmeaDouble(text, raw)) {
        return false;
    }
    const int hour = static_cast<int>(raw / 10000.0);
    const int minute = static_cast<int>((raw - hour * 10000.0) / 100.0);
    const double second = raw - hour * 10000.0 - minute * 100.0;
    seconds = hour * 3600.0 + minute * 60.0 + second;
    return true;
}

void setCourseSpeed(GnssPosition &fix, double course_deg, double speed_kph) {
    if (std::isfinite(course_deg)) {
        fix.course_deg = normalizeHeadingDeg(course_deg);
        fix.has_course = true;
    }
    if (std::isfinite(speed_kph) && speed_kph >= 0.0) {
        fix.speed_kph = speed_kph;
        fix.speed_mps = speed_kph / 3.6;
        fix.has_speed = true;
    }
}

void applyVtg(GnssPosition &fix, const GnssVtg &vtg) {
    if (vtg.has_course) {
        fix.course_deg = vtg.course_deg;
        fix.has_course = true;
    }
    if (vtg.has_speed) {
        fix.speed_mps = vtg.speed_mps;
        fix.speed_kph = vtg.speed_kph;
        fix.has_speed = true;
    }
}

Point2d geoToLocalMeters(GeoPoint origin, GeoPoint point) {
    const double lat0 = degToRad(origin.lat_deg);
    const double dlat = degToRad(point.lat_deg - origin.lat_deg);
    const double dlon = degToRad(point.lon_deg - origin.lon_deg);

    Point2d local;
    local.x = EARTH_RADIUS_M * dlon * std::cos(lat0);
    local.y = EARTH_RADIUS_M * dlat;
    return local;
}

double headingDegToYawRad(double heading_deg) {
    return normalizeAngle(PI / 2.0 - degToRad(heading_deg));
}

double calcForwardSpeed(double vn, double ve, double heading_deg) {
    const double heading_rad = degToRad(heading_deg);
    return vn * std::cos(heading_rad) + ve * std::sin(heading_rad);
}

bool fillGnssPosition(const std::vector<double> &values,
                      int lat_index,
                      int lon_index,
                      int time_index,
                      int height_index,
                      int course_index,
                      int speed_kph_index,
                      GnssPosition &fix) {
    if (lat_index < 0 || lon_index < 0 ||
        lat_index >= static_cast<int>(values.size()) ||
        lon_index >= static_cast<int>(values.size()) ||
        !looksLikeLatLon(values[lat_index], values[lon_index])) {
        return false;
    }

    GnssPosition parsed;
    parsed.position.lat_deg = values[lat_index];
    parsed.position.lon_deg = values[lon_index];
    if (time_index >= 0 && time_index < static_cast<int>(values.size()) &&
        std::isfinite(values[time_index])) {
        parsed.time = values[time_index];
        parsed.has_time = true;
    }
    if (height_index >= 0 && height_index < static_cast<int>(values.size()) &&
        std::isfinite(values[height_index])) {
        parsed.height_m = values[height_index];
        parsed.has_height = true;
    }
    if (course_index >= 0 && course_index < static_cast<int>(values.size()) &&
        speed_kph_index >= 0 && speed_kph_index < static_cast<int>(values.size())) {
        setCourseSpeed(parsed, values[course_index], values[speed_kph_index]);
    }

    fix = parsed;
    return true;
}

bool fillGnssPositionFromNumbers(const std::vector<double> &values,
                                 bool prefixed,
                                 GnssPosition &fix) {
    if (values.size() < 2) {
        return false;
    }

    if (prefixed && values.size() >= 4 &&
        fillGnssPosition(values, 2, 3, 1, 4, 5, 6, fix)) {
        fix.frame = roundToInt(values[0]);
        return true;
    }
    if (values.size() >= 4 &&
        fillGnssPosition(values, 2, 3, 1, 4, 5, 6, fix)) {
        return true;
    }
    if (values.size() >= 3 &&
        fillGnssPosition(values, 1, 2, 0, 3, 4, 5, fix)) {
        return true;
    }
    return fillGnssPosition(values, 0, 1, -1, 2, 3, 4, fix);
}

int wheelSpeedToDriverPercent(double wheel_speed_mps, double max_wheel_speed_mps) {
    if (max_wheel_speed_mps <= 0.0) {
        return 0;
    }
    const double ratio = clampDouble(wheel_speed_mps / max_wheel_speed_mps, -1.0, 1.0);
    return roundToInt(ratio * 100.0);
}

int findTargetIndex(const RobotState &state, const std::vector<Point2d> &path, double lookahead_m) {
    Point2d robot;
    robot.x = state.x;
    robot.y = state.y;

    int nearest = 0;
    double nearest_dist = pointDistance(robot, path[0]);
    for (int i = 1; i < static_cast<int>(path.size()); ++i) {
        const double distance = pointDistance(robot, path[i]);
        if (distance < nearest_dist) {
            nearest = i;
            nearest_dist = distance;
        }
    }

    int target = nearest;
    // 从 nearest 沿路径向前找到第一个距离 >= lookahead 的点
    // 防止在机器人远离路径时 target 永远卡在 0
    for (int i = nearest; i < static_cast<int>(path.size()); ++i) {
        if (pointDistance(robot, path[i]) >= lookahead_m) {
            target = i;
            break;
        }
    }
    return target;
}

ControlOutput purePursuitDiffDrive(const RobotState &state,
                                   const std::vector<Point2d> &path,
                                   const ControlParams &params) {
    ControlOutput out;
    out.path_size = static_cast<int>(path.size());

    Point2d robot;
    robot.x = state.x;
    robot.y = state.y;

    const Point2d goal = path.back();
    const double distance_to_goal = pointDistance(robot, goal);
    const double lookahead = std::max(params.lookahead_base_m,
                                      params.lookahead_base_m +
                                          params.lookahead_gain * std::fabs(state.forward_speed_mps));

    out.distance_to_goal_m = distance_to_goal;
    out.lookahead_m = lookahead;

    if (distance_to_goal <= params.goal_tolerance_m) {
        out.target_index = static_cast<int>(path.size()) - 1;
        out.goal_reached = true;
        return out;
    }

    const int target_index = findTargetIndex(state, path, lookahead);
    const Point2d target = path[target_index];
    const double target_angle = std::atan2(target.y - state.y, target.x - state.x);
    const double alpha = normalizeAngle(target_angle - state.yaw_rad);
    const double curvature = 2.0 * std::sin(alpha) / lookahead;

    double v_cmd = params.target_speed_mps;
    if (distance_to_goal < params.slow_radius_m && params.slow_radius_m > 0.0) {
        const double slow_ratio = distance_to_goal / params.slow_radius_m;
        v_cmd = std::max(params.min_speed_mps, params.target_speed_mps * slow_ratio);
    }

    const double omega_cmd = v_cmd * curvature;
    const double left_wheel = v_cmd - omega_cmd * params.track_width_m / 2.0;
    const double right_wheel = v_cmd + omega_cmd * params.track_width_m / 2.0;

    out.target_index = target_index;
    out.v_cmd_mps = v_cmd;
    out.omega_cmd_radps = omega_cmd;
    out.left_wheel_mps = left_wheel;
    out.right_wheel_mps = right_wheel;
    out.left_percent = wheelSpeedToDriverPercent(left_wheel, params.max_wheel_speed_mps);
    out.right_percent = wheelSpeedToDriverPercent(right_wheel, params.max_wheel_speed_mps);
    return out;
}

template <typename T>
void readOptional(const YAML::Node &node, const char *key, T &value) {
    if (node && node[key]) {
        value = node[key].as<T>();
    }
}

void readRuntimeOptions(const YAML::Node &node, RuntimeOptions &runtime) {
    readOptional(node, "device", runtime.device);
    readOptional(node, "tb6612_dev", runtime.device);
    readOptional(node, "dry_run", runtime.dry_run);
    readOptional(node, "swap_ab", runtime.swap_ab);
    readOptional(node, "invert_left", runtime.invert_left);
    readOptional(node, "invert_right", runtime.invert_right);
    readOptional(node, "left_scale", runtime.left_scale);
    readOptional(node, "right_scale", runtime.right_scale);
    readOptional(node, "max_command_percent", runtime.max_command_percent);
    readOptional(node, "print_every", runtime.print_every);
    runtime.max_command_percent = clampInt(runtime.max_command_percent, 0, 100);
    runtime.print_every = std::max(1, runtime.print_every);
}

void readControlParams(const YAML::Node &node, ControlParams &control) {
    readOptional(node, "target_speed_mps", control.target_speed_mps);
    readOptional(node, "min_speed_mps", control.min_speed_mps);
    readOptional(node, "lookahead_base_m", control.lookahead_base_m);
    readOptional(node, "lookahead_gain", control.lookahead_gain);
    readOptional(node, "track_width_m", control.track_width_m);
    readOptional(node, "max_wheel_speed_mps", control.max_wheel_speed_mps);
    readOptional(node, "goal_tolerance_m", control.goal_tolerance_m);
    readOptional(node, "slow_radius_m", control.slow_radius_m);
    readOptional(node, "path_step_m", control.path_step_m);
}

void readGnssOnlyParams(const YAML::Node &node, GnssOnlyParams &gnss_only) {
    readOptional(node, "heading_min_distance_m", gnss_only.heading_min_distance_m);
    readOptional(node, "max_speed_mps", gnss_only.max_speed_mps);
    readOptional(node, "vtg_min_speed_mps", gnss_only.vtg_min_speed_mps);
}

} // namespace

bool parseNavOutputLine(const std::string &line, NavOutput &nav) {
    const std::vector<std::string> fields = splitCsv(line);
    if (fields.size() < 12 || fields[0] != "NAV") {
        return false;
    }

    NavOutput parsed;
    if (!parseInt(fields[1], parsed.frame) ||
        !parseDouble(fields[2], parsed.time) ||
        !parseDouble(fields[3], parsed.position.lat_deg) ||
        !parseDouble(fields[4], parsed.position.lon_deg) ||
        !parseDouble(fields[5], parsed.height_m) ||
        !parseDouble(fields[6], parsed.velocity_north_mps) ||
        !parseDouble(fields[7], parsed.velocity_east_mps) ||
        !parseDouble(fields[8], parsed.velocity_down_mps) ||
        !parseDouble(fields[9], parsed.roll_deg) ||
        !parseDouble(fields[10], parsed.pitch_deg) ||
        !parseDouble(fields[11], parsed.yaw_deg)) {
        return false;
    }

    if (fields.size() >= 15 &&
        parseDouble(fields[12], parsed.std_n) &&
        parseDouble(fields[13], parsed.std_e) &&
        parseDouble(fields[14], parsed.std_d)) {
        parsed.has_std = std::isfinite(parsed.std_n) &&
                         std::isfinite(parsed.std_e) &&
                         std::isfinite(parsed.std_d);
    }

    nav = parsed;
    return true;
}

bool parseGnssVtgLine(const std::string &line, GnssVtg &vtg) {
    const std::vector<std::string> fields = splitCsv(line);
    if (fields.empty()) {
        return false;
    }

    const bool nmea_vtg = isNmeaType(fields[0], "VTG");
    const bool simple_vtg = isPrefix(fields[0], "VTG");
    if (!nmea_vtg && !simple_vtg) {
        return false;
    }

    GnssVtg parsed;
    if (nmea_vtg) {
        if (fields.size() < 8) {
            return false;
        }

        double course_deg = 0.0;
        if (parseNmeaDouble(fields[1], course_deg)) {
            parsed.course_deg = normalizeHeadingDeg(course_deg);
            parsed.has_course = true;
        }

        double speed_kph = 0.0;
        if (parseNmeaDouble(fields[7], speed_kph)) {
            parsed.speed_kph = speed_kph;
            parsed.speed_mps = speed_kph / 3.6;
            parsed.has_speed = true;
        } else {
            double speed_knots = 0.0;
            if (fields.size() >= 6 && parseNmeaDouble(fields[5], speed_knots)) {
                parsed.speed_kph = speed_knots * 1.852;
                parsed.speed_mps = speed_knots * 0.5144444444444445;
                parsed.has_speed = true;
            }
        }
    } else {
        if (fields.size() < 3) {
            return false;
        }

        double course_deg = 0.0;
        double speed_kph = 0.0;
        if (!parseDouble(fields[1], course_deg) ||
            !parseDouble(fields[2], speed_kph)) {
            return false;
        }
        parsed.course_deg = normalizeHeadingDeg(course_deg);
        parsed.has_course = true;
        parsed.speed_kph = speed_kph;
        parsed.speed_mps = speed_kph / 3.6;
        parsed.has_speed = true;
    }

    if (!parsed.has_course && !parsed.has_speed) {
        return false;
    }
    vtg = parsed;
    return true;
}

bool parseGnssPositionLine(const std::string &line, GnssPosition &fix) {
    const std::string stripped = trim(line);
    if (stripped.empty() || stripped[0] == '#') {
        return false;
    }

    const std::vector<std::string> csv_fields = splitCsv(stripped);
    if (!csv_fields.empty() && isNmeaType(csv_fields[0], "GGA")) {
        if (csv_fields.size() < 12) {
            return false;
        }

        const std::string lat_dir = trim(csv_fields[3]);
        const std::string lon_dir = trim(csv_fields[5]);
        if (csv_fields[2].empty() || lat_dir.empty() ||
            csv_fields[4].empty() || lon_dir.empty()) {
            return false;
        }

        GnssPosition parsed;
        parsed.position.lat_deg = nmeaDegMinToDeg(csv_fields[2], lat_dir[0]);
        parsed.position.lon_deg = nmeaDegMinToDeg(csv_fields[4], lon_dir[0]);
        if (!looksLikeLatLon(parsed.position.lat_deg, parsed.position.lon_deg)) {
            return false;
        }

        double time_seconds = 0.0;
        if (parseNmeaTimeSeconds(csv_fields[1], time_seconds)) {
            parsed.time = time_seconds;
            parsed.has_time = true;
        }

        double alt_msl = 0.0;
        double geoid = 0.0;
        if (parseNmeaDouble(csv_fields[9], alt_msl) &&
            parseNmeaDouble(csv_fields[11], geoid)) {
            parsed.height_m = alt_msl + geoid;
            parsed.has_height = true;
        }

        fix = parsed;
        return true;
    }

    const std::vector<std::string> fields = splitGnssFields(stripped);
    if (fields.empty()) {
        return false;
    }
    if (isPrefix(fields[0], "NAV")) {
        return false;
    }

    if (isNmeaType(fields[0], "GGA")) {
        if (fields.size() < 12) {
            return false;
        }

        const std::string lat_dir = trim(fields[3]);
        const std::string lon_dir = trim(fields[5]);
        if (fields[2].empty() || lat_dir.empty() ||
            fields[4].empty() || lon_dir.empty()) {
            return false;
        }

        GnssPosition parsed;
        parsed.position.lat_deg = nmeaDegMinToDeg(fields[2], lat_dir[0]);
        parsed.position.lon_deg = nmeaDegMinToDeg(fields[4], lon_dir[0]);
        if (!looksLikeLatLon(parsed.position.lat_deg, parsed.position.lon_deg)) {
            return false;
        }

        double time_seconds = 0.0;
        if (parseNmeaTimeSeconds(fields[1], time_seconds)) {
            parsed.time = time_seconds;
            parsed.has_time = true;
        }

        double alt_msl = 0.0;
        double geoid = 0.0;
        if (parseNmeaDouble(fields[9], alt_msl) &&
            parseNmeaDouble(fields[11], geoid)) {
            parsed.height_m = alt_msl + geoid;
            parsed.has_height = true;
        }

        fix = parsed;
        return true;
    }

    const bool prefixed = isPrefix(fields[0], "GNSS") ||
                          isPrefix(fields[0], "GPS") ||
                          isPrefix(fields[0], "FIX");
    const size_t first_numeric = prefixed ? 1U : 0U;
    std::vector<double> values;
    values.reserve(fields.size());
    for (size_t i = first_numeric; i < fields.size(); ++i) {
        double value = 0.0;
        if (!parseDouble(fields[i], value)) {
            return false;
        }
        values.push_back(value);
    }

    return fillGnssPositionFromNumbers(values, prefixed, fix);
}

bool loadPathControlConfig(const std::string &yaml_path, PathControlConfig &config, std::string &error) {
    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception &e) {
        error = std::string("failed to read configuration file: ") + e.what();
        return false;
    }

    const YAML::Node node = yaml["path_control"];
    if (!node) {
        error = "missing path_control section in yaml";
        return false;
    }

    try {
        if (node["goal"]) {
            const std::vector<double> goal = node["goal"].as<std::vector<double>>();
            if (goal.size() != 2) {
                error = "path_control.goal must be [lat_deg, lon_deg]";
                return false;
            }
            config.goal.lat_deg = goal[0];
            config.goal.lon_deg = goal[1];
            config.has_goal = true;
        } else if (node["goal_lat"] && node["goal_lon"]) {
            config.goal.lat_deg = node["goal_lat"].as<double>();
            config.goal.lon_deg = node["goal_lon"].as<double>();
            config.has_goal = true;
        }

        readRuntimeOptions(node, config.runtime);
        readControlParams(node, config.control);
        readGnssOnlyParams(node, config.gnss_only);
        readRuntimeOptions(node["runtime"], config.runtime);
        readControlParams(node["control"], config.control);
        readGnssOnlyParams(node["gnss_only"], config.gnss_only);
        readGnssOnlyParams(node["gnss"], config.gnss_only);
    } catch (const YAML::Exception &e) {
        error = std::string("failed to parse path_control configuration: ") + e.what();
        return false;
    }

    if (!config.has_goal) {
        error = "missing path_control.goal or path_control.goal_lat/goal_lon";
        return false;
    }
    if (config.control.path_step_m < 0.02) {
        config.control.path_step_m = 0.02;
    }
    if (config.control.lookahead_base_m <= 0.0) {
        config.control.lookahead_base_m = 0.60;
    }
    if (config.control.track_width_m <= 0.0) {
        config.control.track_width_m = 0.26;
    }
    if (config.control.max_wheel_speed_mps <= 0.0) {
        config.control.max_wheel_speed_mps = 0.80;
    }
    if (config.control.goal_tolerance_m < 0.0) {
        config.control.goal_tolerance_m = 0.25;
    }
    if (config.control.slow_radius_m < 0.0) {
        config.control.slow_radius_m = 0.0;
    }
    if (config.control.min_speed_mps < 0.0) {
        config.control.min_speed_mps = 0.0;
    }
    if (config.gnss_only.heading_min_distance_m < 0.02) {
        config.gnss_only.heading_min_distance_m = 0.02;
    }
    if (config.gnss_only.max_speed_mps < 0.0) {
        config.gnss_only.max_speed_mps = 0.0;
    }
    if (config.gnss_only.vtg_min_speed_mps < 0.0) {
        config.gnss_only.vtg_min_speed_mps = 0.0;
    }

    return true;
}

PathController::PathController(const GeoPoint &goal, const ControlParams &params)
    : goal_(goal), params_(params) {}

bool PathController::update(const NavOutput &nav, RobotState &state, ControlOutput &output, std::string &error) {
    const double yaw_rad = headingDegToYawRad(nav.yaw_deg);
    const double forward_speed_mps = calcForwardSpeed(nav.velocity_north_mps,
                                                      nav.velocity_east_mps,
                                                      nav.yaw_deg);
    return updateFromPose(nav.position, yaw_rad, forward_speed_mps, state, output, error);
}

bool PathController::updateFromPose(const GeoPoint &position,
                                    double yaw_rad,
                                    double forward_speed_mps,
                                    RobotState &state,
                                    ControlOutput &output,
                                    std::string &error) {
    if (!ready_ && !initializePath(position, error)) {
        return false;
    }

    const Point2d local = geoToLocalMeters(origin_, position);
    state.x = local.x;
    state.y = local.y;
    state.yaw_rad = normalizeAngle(yaw_rad);
    state.forward_speed_mps = forward_speed_mps;
    output = purePursuitDiffDrive(state, path_, params_);
    return true;
}

bool PathController::isReady() const {
    return ready_;
}

GeoPoint PathController::origin() const {
    return origin_;
}

Point2d PathController::goalLocal() const {
    return goal_local_;
}

int PathController::pathSize() const {
    return static_cast<int>(path_.size());
}

bool PathController::initializePath(const GeoPoint &origin, std::string &error) {
    origin_ = origin;
    goal_local_ = geoToLocalMeters(origin_, goal_);
    const Point2d start;
    const double total = pointDistance(start, goal_local_);
    const int count = std::max(1, static_cast<int>(std::ceil(total / params_.path_step_m)));

    path_.resize(static_cast<size_t>(count) + 1U);
    for (int i = 0; i <= count; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(count);
        path_[i].x = start.x + (goal_local_.x - start.x) * t;
        path_[i].y = start.y + (goal_local_.y - start.y) * t;
    }

    if (path_.empty()) {
        error = "failed to create path";
        return false;
    }

    ready_ = true;
    return true;
}

GnssPathController::GnssPathController(const GeoPoint &goal,
                                       const ControlParams &control_params,
                                       const GnssOnlyParams &gnss_params)
    : controller_(goal, control_params), goal_(goal), params_(gnss_params) {}

bool GnssPathController::update(const GnssPosition &fix,
                                RobotState &state,
                                ControlOutput &output,
                                std::string &error) {
    Point2d local;
    if (controller_.isReady()) {
        local = geoToLocalMeters(controller_.origin(), fix.position);
    } else {
        local.x = 0.0;
        local.y = 0.0;
        const Point2d goal_local = geoToLocalMeters(fix.position, goal_);
        if (pointDistance(local, goal_local) > 1e-6) {
            yaw_rad_ = std::atan2(goal_local.y - local.y, goal_local.x - local.x);
            has_yaw_ = true;
        }
    }

    const bool use_vtg_course = fix.has_course &&
                                (!fix.has_speed || fix.speed_mps >= params_.vtg_min_speed_mps);
    if (use_vtg_course) {
        yaw_rad_ = headingDegToYawRad(fix.course_deg);
        has_yaw_ = true;
    }
    if (fix.has_speed) {
        forward_speed_mps_ = fix.speed_mps;
        if (params_.max_speed_mps > 0.0) {
            forward_speed_mps_ = clampDouble(forward_speed_mps_, 0.0, params_.max_speed_mps);
        }
    }

    if (has_previous_fix_) {
        const double distance = pointDistance(local, previous_local_);
        double estimated_speed = 0.0;
        if (fix.has_time && has_time_ && fix.time > previous_time_) {
            estimated_speed = distance / (fix.time - previous_time_);
            if (params_.max_speed_mps > 0.0) {
                estimated_speed = clampDouble(estimated_speed, 0.0, params_.max_speed_mps);
            }
        }

        if (!use_vtg_course && distance >= params_.heading_min_distance_m) {
            yaw_rad_ = std::atan2(local.y - previous_local_.y, local.x - previous_local_.x);
            has_yaw_ = true;
        }
        if (!fix.has_speed && (distance >= params_.heading_min_distance_m ||
                               (fix.has_time && has_time_))) {
            forward_speed_mps_ = estimated_speed;
        }
    }

    if (!has_yaw_) {
        const Point2d goal_local = controller_.isReady() ? controller_.goalLocal()
                                                        : geoToLocalMeters(fix.position, goal_);
        yaw_rad_ = std::atan2(goal_local.y - local.y, goal_local.x - local.x);
        has_yaw_ = true;
    }

    const bool ok = controller_.updateFromPose(fix.position,
                                               yaw_rad_,
                                               forward_speed_mps_,
                                               state,
                                               output,
                                               error);
    if (!ok) {
        return false;
    }

    if (controller_.isReady()) {
        previous_local_ = geoToLocalMeters(controller_.origin(), fix.position);
    } else {
        previous_local_ = local;
    }
    previous_time_ = fix.time;
    has_time_ = fix.has_time;
    has_previous_fix_ = true;
    return true;
}

bool GnssPathController::isReady() const {
    return controller_.isReady();
}

GeoPoint GnssPathController::origin() const {
    return controller_.origin();
}

Point2d GnssPathController::goalLocal() const {
    return controller_.goalLocal();
}

int GnssPathController::pathSize() const {
    return controller_.pathSize();
}

Tb6612Driver::Tb6612Driver(const RuntimeOptions &options) : options_(options) {}

Tb6612Driver::~Tb6612Driver() {
    close();
}

bool Tb6612Driver::open(std::string &error) {
    close();
    if (options_.dry_run) {
        return true;
    }

#ifdef _WIN32
    fd_ = _open(options_.device.c_str(), O_WRONLY | O_CLOEXEC);
#else
    fd_ = ::open(options_.device.c_str(), O_WRONLY | O_CLOEXEC);
#endif
    if (fd_ < 0) {
        error = "failed to open " + options_.device + ": " + std::strerror(errno);
        return false;
    }
    return true;
}

void Tb6612Driver::close() {
    if (fd_ >= 0) {
#ifdef _WIN32
        _close(fd_);
#else
        ::close(fd_);
#endif
        fd_ = -1;
    }
}

bool Tb6612Driver::stop(std::string &error) {
    return writeCommand("stop\n", error);
}

bool Tb6612Driver::set(int left_percent, int right_percent, std::string &error) {
    left_percent = roundToInt(static_cast<double>(left_percent) * options_.left_scale);
    right_percent = roundToInt(static_cast<double>(right_percent) * options_.right_scale);

    if (options_.invert_left) {
        left_percent = -left_percent;
    }
    if (options_.invert_right) {
        right_percent = -right_percent;
    }

    const int limit = clampInt(options_.max_command_percent, 0, 100);
    left_percent = clampInt(left_percent, -limit, limit);
    right_percent = clampInt(right_percent, -limit, limit);

    int motor_a = left_percent;
    int motor_b = right_percent;
    if (options_.swap_ab) {
        motor_a = right_percent;
        motor_b = left_percent;
    }

    char command[64];
    std::snprintf(command, sizeof(command), "set %d %d\n", motor_a, motor_b);
    return writeCommand(command, error);
}

bool Tb6612Driver::writeCommand(const std::string &command, std::string &error) {
    if (options_.dry_run) {
        std::cout << "driver_cmd: " << command;
        return true;
    }

    const char *ptr = command.c_str();
    size_t left = command.size();
    while (left > 0) {
#ifdef _WIN32
        const unsigned int chunk = static_cast<unsigned int>(std::min<size_t>(left, 32767U));
        const int written = _write(fd_, ptr, chunk);
#else
        const ssize_t written = ::write(fd_, ptr, left);
#endif
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            error = "write to " + options_.device + " failed: " + std::strerror(errno);
            return false;
        }
        ptr += written;
        left -= static_cast<size_t>(written);
    }
    return true;
}

} // namespace path_control
