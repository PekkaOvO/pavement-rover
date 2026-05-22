#include <Eigen/Dense>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "common/angle.h"
#include "kf-gins/gi_engine.h"
#include "realtime/realtime.h"

bool loadConfig(YAML::Node &config, GINSOptions &options);
void printNavResult(int frame_id, double time, const NavState &navstate,
                    const Eigen::Vector3d &gnss_std, bool has_gnss_std);

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cout << "usage: KF-GINS-Realtime kf-gins.yaml" << std::endl;
        return -1;
    }

    YAML::Node config;
    try {
        config = YAML::LoadFile(argv[1]);
    } catch (const YAML::Exception &) {
        std::cout << "Failed to read configuration file." << std::endl;
        return -1;
    }

    GINSOptions options;
    if (!loadConfig(config, options)) {
        std::cout << "Error occurs in the configuration file!" << std::endl;
        return -1;
    }

    std::string imu_dev = "/dev/icm20608";
    std::string gnss_dev = "/dev/ttyUSB0";
    int gnss_baud = 115200;
    int imu_poll_us = 1000;
    int output_decimation = 10;
    std::vector<double> gnss_std = {0.1, 0.1, 0.1};

    try {
        if (config["imu_dev"]) {
            imu_dev = config["imu_dev"].as<std::string>();
        }
        if (config["gnss_dev"]) {
            gnss_dev = config["gnss_dev"].as<std::string>();
        }
        if (config["gnss_baud"]) {
            gnss_baud = config["gnss_baud"].as<int>();
        }
        if (config["imu_poll_us"]) {
            imu_poll_us = config["imu_poll_us"].as<int>();
        }
        if (config["output_decimation"]) {
            output_decimation = config["output_decimation"].as<int>();
        }
        if (config["gnss_std"]) {
            gnss_std = config["gnss_std"].as<std::vector<double>>();
        }
    } catch (const YAML::Exception &) {
        std::cout << "Failed when loading realtime configuration." << std::endl;
        return -1;
    }

    if (gnss_std.size() != 3) {
        std::cout << "gnss_std must contain exactly 3 elements." << std::endl;
        return -1;
    }
    if (output_decimation <= 0) {
        output_decimation = 10;
    }

    GINSOptions options_copy = options;
    GIEngine giengine(options_copy);

    RealtimeImuReader imu_reader;
    RealtimeGnssReader gnss_reader;

    if (!imu_reader.openDevice(imu_dev)) {
        std::cout << "Failed to open imu device: " << imu_dev << std::endl;
        return -1;
    }
    if (!gnss_reader.openDevice(gnss_dev, gnss_baud)) {
        std::cout << "Failed to open gnss device: " << gnss_dev << std::endl;
        return -1;
    }

    std::cout << "KF-GINS realtime started" << std::endl;
    std::cout << "IMU device : " << imu_dev << std::endl;
    std::cout << "GNSS device: " << gnss_dev << std::endl;
    std::cout << "GNSS std(default): " << gnss_std[0] << ", " << gnss_std[1] << ", " << gnss_std[2] << std::endl;
    std::cout << "Output decimation: " << output_decimation << std::endl;
    std::cout << "NAV,frame,time,lat_deg,lon_deg,h_m,vn,ve,vd,roll_deg,pitch_deg,yaw_deg,std_n,std_e,std_d" << std::endl;

    bool first_imu = true;
    int frame_count = 0;

    while (true) {
        GNSS gnss;
        while (gnss_reader.pollSample(gnss, gnss_std)) {
            giengine.addGnssData(gnss);
        }

        IMU imu;
        if (!imu_reader.readSample(imu)) {
            std::this_thread::sleep_for(std::chrono::microseconds(imu_poll_us));
            continue;
        }

        if (first_imu) {
            giengine.addImuData(imu, true);
            first_imu = false;
            continue;
        }

        giengine.addImuData(imu);
        giengine.newImuProcess();
        frame_count++;

        if (frame_count % output_decimation == 0) {
            const NavState navstate = giengine.getNavState();
            const Eigen::Vector3d used_gnss_std = giengine.getLastGnssStd();
            const bool has_used_gnss_std = giengine.hasLastGnssStd();
            printNavResult(frame_count, giengine.timestamp(), navstate, used_gnss_std, has_used_gnss_std);
        }
    }

    return 0;
}

bool loadConfig(YAML::Node &config, GINSOptions &options) {
    std::vector<double> vec1, vec2, vec3, vec4, vec5, vec6;
    try {
        vec1 = config["initpos"].as<std::vector<double>>();
        vec2 = config["initvel"].as<std::vector<double>>();
        vec3 = config["initatt"].as<std::vector<double>>();
    } catch (const YAML::Exception &) {
        std::cout << "Failed when loading configuration. Please check initial position, velocity, and attitude!" << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        options.initstate.pos[i] = vec1[i] * D2R;
        options.initstate.vel[i] = vec2[i];
        options.initstate.euler[i] = vec3[i] * D2R;
    }
    options.initstate.pos[2] *= R2D;

    try {
        vec1 = config["initgyrbias"].as<std::vector<double>>();
        vec2 = config["initaccbias"].as<std::vector<double>>();
        vec3 = config["initgyrscale"].as<std::vector<double>>();
        vec4 = config["initaccscale"].as<std::vector<double>>();
    } catch (const YAML::Exception &) {
        std::cout << "Failed when loading configuration. Please check initial IMU error!" << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        options.initstate.imuerror.gyrbias[i] = vec1[i] * D2R / 3600.0;
        options.initstate.imuerror.accbias[i] = vec2[i] * 1e-5;
        options.initstate.imuerror.gyrscale[i] = vec3[i] * 1e-6;
        options.initstate.imuerror.accscale[i] = vec4[i] * 1e-6;
    }

    try {
        vec1 = config["initposstd"].as<std::vector<double>>();
        vec2 = config["initvelstd"].as<std::vector<double>>();
        vec3 = config["initattstd"].as<std::vector<double>>();
    } catch (const YAML::Exception &) {
        std::cout << "Failed when loading configuration. Please check initial std of position, velocity, and attitude!" << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        options.initstate_std.pos[i] = vec1[i];
        options.initstate_std.vel[i] = vec2[i];
        options.initstate_std.euler[i] = vec3[i] * D2R;
    }

    try {
        vec1 = config["imunoise"]["arw"].as<std::vector<double>>();
        vec2 = config["imunoise"]["vrw"].as<std::vector<double>>();
        vec3 = config["imunoise"]["gbstd"].as<std::vector<double>>();
        vec4 = config["imunoise"]["abstd"].as<std::vector<double>>();
        vec5 = config["imunoise"]["gsstd"].as<std::vector<double>>();
        vec6 = config["imunoise"]["asstd"].as<std::vector<double>>();
        options.imunoise.corr_time = config["imunoise"]["corrtime"].as<double>();
    } catch (const YAML::Exception &) {
        std::cout << "Failed when loading configuration. Please check IMU noise!" << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {
        options.imunoise.gyr_arw[i] = vec1[i];
        options.imunoise.acc_vrw[i] = vec2[i];
        options.imunoise.gyrbias_std[i] = vec3[i];
        options.imunoise.accbias_std[i] = vec4[i];
        options.imunoise.gyrscale_std[i] = vec5[i];
        options.imunoise.accscale_std[i] = vec6[i];
    }

    try {
        vec1 = config["initbgstd"].as<std::vector<double>>();
    } catch (const YAML::Exception &) {
        vec1 = {options.imunoise.gyrbias_std.x(), options.imunoise.gyrbias_std.y(), options.imunoise.gyrbias_std.z()};
    }
    try {
        vec2 = config["initbastd"].as<std::vector<double>>();
    } catch (const YAML::Exception &) {
        vec2 = {options.imunoise.accbias_std.x(), options.imunoise.accbias_std.y(), options.imunoise.accbias_std.z()};
    }
    try {
        vec3 = config["initsgstd"].as<std::vector<double>>();
    } catch (const YAML::Exception &) {
        vec3 = {options.imunoise.gyrscale_std.x(), options.imunoise.gyrscale_std.y(), options.imunoise.gyrscale_std.z()};
    }
    try {
        vec4 = config["initsastd"].as<std::vector<double>>();
    } catch (const YAML::Exception &) {
        vec4 = {options.imunoise.accscale_std.x(), options.imunoise.accscale_std.y(), options.imunoise.accscale_std.z()};
    }
    for (int i = 0; i < 3; i++) {
        options.initstate_std.imuerror.gyrbias[i] = vec1[i] * D2R / 3600.0;
        options.initstate_std.imuerror.accbias[i] = vec2[i] * 1e-5;
        options.initstate_std.imuerror.gyrscale[i] = vec3[i] * 1e-6;
        options.initstate_std.imuerror.accscale[i] = vec4[i] * 1e-6;
    }

    options.imunoise.gyr_arw *= (D2R / 60.0);
    options.imunoise.acc_vrw /= 60.0;
    options.imunoise.gyrbias_std *= (D2R / 3600.0);
    options.imunoise.accbias_std *= 1e-5;
    options.imunoise.gyrscale_std *= 1e-6;
    options.imunoise.accscale_std *= 1e-6;
    options.imunoise.corr_time *= 3600;

    try {
        vec1 = config["antlever"].as<std::vector<double>>();
    } catch (const YAML::Exception &) {
        std::cout << "Failed when loading configuration. Please check antenna leverarm!" << std::endl;
        return false;
    }
    options.antlever = Eigen::Vector3d(vec1.data());
    return true;
}

void printNavResult(int frame_id, double time, const NavState &navstate,
                    const Eigen::Vector3d &gnss_std, bool has_gnss_std) {
    std::cout << std::fixed << std::setprecision(10)
              << "NAV," << frame_id << "," << time << ","
              << navstate.pos[0] * R2D << ","
              << navstate.pos[1] * R2D << ","
              << std::setprecision(4) << navstate.pos[2] << ","
              << navstate.vel[0] << ","
              << navstate.vel[1] << ","
              << navstate.vel[2] << ","
              << navstate.euler[0] * R2D << ","
              << navstate.euler[1] * R2D << ","
              << navstate.euler[2] * R2D << ",";

    if (has_gnss_std) {
        std::cout << gnss_std[0] << "," << gnss_std[1] << "," << gnss_std[2];
    } else {
        std::cout << "nan,nan,nan";
    }

    std::cout << std::endl;
}