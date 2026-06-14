#ifndef PATH_CONTROL_PATH_CONTROLLER_H
#define PATH_CONTROL_PATH_CONTROLLER_H

#include <string>
#include <vector>

namespace path_control {

struct GeoPoint {
    double lat_deg = 0.0;
    double lon_deg = 0.0;
};

struct Point2d {
    double x = 0.0; // east, meter
    double y = 0.0; // north, meter
};

struct NavOutput {
    int frame = 0;
    double time = 0.0;
    GeoPoint position;
    double height_m = 0.0;
    double velocity_north_mps = 0.0;
    double velocity_east_mps = 0.0;
    double velocity_down_mps = 0.0;
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    double std_n = 0.0;
    double std_e = 0.0;
    double std_d = 0.0;
    bool has_std = false;
};

struct GnssPosition {
    int frame = 0;
    double time = 0.0;
    bool has_time = false;
    GeoPoint position;
    double height_m = 0.0;
    bool has_height = false;
    double course_deg = 0.0;
    bool has_course = false;
    double speed_mps = 0.0;
    double speed_kph = 0.0;
    bool has_speed = false;
};

struct GnssVtg {
    double course_deg = 0.0;
    bool has_course = false;
    double speed_mps = 0.0;
    double speed_kph = 0.0;
    bool has_speed = false;
};

struct ControlParams {
    double target_speed_mps = 0.35;
    double min_speed_mps = 0.12;
    double lookahead_base_m = 0.60;
    double lookahead_gain = 0.60;
    double track_width_m = 0.16;
    double max_wheel_speed_mps = 0.80;
    double goal_tolerance_m = 0.25;
    double slow_radius_m = 1.20;
    double path_step_m = 0.20;
};

struct GnssOnlyParams {
    double heading_min_distance_m = 0.20;
    double max_speed_mps = 3.0;
    double vtg_min_speed_mps = 0.05;
};

struct RuntimeOptions {
    std::string device = "/dev/tb6612";
    bool dry_run = false;
    bool swap_ab = false;
    bool invert_left = false;
    bool invert_right = false;
    double left_scale = 1.0;
    double right_scale = 1.0;
    int max_command_percent = 100;
    int print_every = 1;
};

struct PathControlConfig {
    RuntimeOptions runtime;
    ControlParams control;
    GnssOnlyParams gnss_only;
    GeoPoint goal;
    bool has_goal = false;
    std::vector<GeoPoint> goals;
    bool has_goals = false;
};

struct RobotState {
    double x = 0.0;
    double y = 0.0;
    double yaw_rad = 0.0;
    double forward_speed_mps = 0.0;
};

struct ControlOutput {
    int target_index = 0;
    int path_size = 0;
    double distance_to_goal_m = 0.0;
    double lookahead_m = 0.0;
    double v_cmd_mps = 0.0;
    double omega_cmd_radps = 0.0;
    double left_wheel_mps = 0.0;
    double right_wheel_mps = 0.0;
    int left_percent = 0;
    int right_percent = 0;
    bool goal_reached = false;
};

bool parseNavOutputLine(const std::string &line, NavOutput &nav);
bool parseGnssPositionLine(const std::string &line, GnssPosition &fix);
bool parseGnssVtgLine(const std::string &line, GnssVtg &vtg);
bool loadPathControlConfig(const std::string &yaml_path, PathControlConfig &config, std::string &error);

class PathController {
public:
    PathController(const std::vector<GeoPoint> &goals, const ControlParams &params);

    bool update(const NavOutput &nav, RobotState &state, ControlOutput &output, std::string &error);
    bool updateFromPose(const GeoPoint &position,
                        double yaw_rad,
                        double forward_speed_mps,
                        RobotState &state,
                        ControlOutput &output,
                        std::string &error);

    bool isReady() const;
    GeoPoint origin() const;
    Point2d goalLocal() const;
    int pathSize() const;
    int goalsCount() const;
    int currentGoalIndex() const;

private:
    bool initializePath(const GeoPoint &origin, std::string &error);

private:
    GeoPoint origin_;
    std::vector<GeoPoint> goals_;
    Point2d goal_local_;
    ControlParams params_;
    std::vector<Point2d> path_;
    bool ready_ = false;
    int current_goal_index_ = 0;
};

class GnssPathController {
public:
    GnssPathController(const std::vector<GeoPoint> &goals,
                       const ControlParams &control_params,
                       const GnssOnlyParams &gnss_params);

    bool update(const GnssPosition &fix, RobotState &state, ControlOutput &output, std::string &error);

    bool isReady() const;
    GeoPoint origin() const;
    Point2d goalLocal() const;
    int pathSize() const;

private:
    PathController controller_;
    std::vector<GeoPoint> goals_;
    GnssOnlyParams params_;
    bool has_previous_fix_ = false;
    bool has_yaw_ = false;
    bool has_time_ = false;
    Point2d previous_local_;
    double previous_time_ = 0.0;
    double yaw_rad_ = 0.0;
    double forward_speed_mps_ = 0.0;
};

class Tb6612Driver {
public:
    explicit Tb6612Driver(const RuntimeOptions &options);
    ~Tb6612Driver();

    bool open(std::string &error);
    void close();
    bool stop(std::string &error);
    bool set(int left_percent, int right_percent, std::string &error);
    bool servo(float angle_deg, std::string &error);   // -90..90°

private:
    bool writeCommand(const std::string &command, std::string &error);

private:
    RuntimeOptions options_;
    int fd_ = -1;
};

} // namespace path_control

#endif // PATH_CONTROL_PATH_CONTROLLER_H
