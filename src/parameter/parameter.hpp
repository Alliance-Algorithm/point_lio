#pragma once

#include "../preprocess/preprocess.h"

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <cstring>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

inline bool is_first_frame   = true;
inline double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
inline double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
inline int pcd_index = 0;

inline std::string lid_topic, imu_topic;
inline bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
inline bool use_imu_as_input, space_down_sample, publish_odometry_without_downsample;
inline int init_map_size, con_frame_num;
inline double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
inline float plane_thr;
inline double filter_size_surf_min, filter_size_map_min, fov_deg;
inline double cube_len;
inline float DET_RANGE;
inline bool imu_en, gravity_align, non_station_start;
inline double imu_time_inte;
inline double laser_point_cov, acc_norm;
inline double vel_cov, acc_cov_input, gyr_cov_input;
inline double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
inline double imu_meas_acc_cov, imu_meas_omg_cov;
inline int lidar_type, pcd_save_interval;
inline std::vector<double> gravity_init, gravity;
inline std::vector<double> extrinT;
inline std::vector<double> extrinR;
inline bool runtime_pos_log, pcd_save_en, path_en, extrinsic_est_en = true;
inline bool scan_pub_en, scan_body_pub_en;
inline shared_ptr<Preprocess> p_pre;
inline double time_lag_imu_to_lidar = 0.0;

inline void readParameters(shared_ptr<rclcpp::Node>& node) {

    p_pre.reset(new Preprocess());

    // nh->declare_parameter<bool>("prop_at_freq_of_imu", true);
    // nh->declare_parameter<bool>("use_imu_as_input", true);
    // nh->declare_parameter<bool>("check_satu", true);
    // nh->declare_parameter<int>("init_map_size", 100);
    // nh->declare_parameter<bool>("space_down_sample", true);
    // nh->declare_parameter<double>("mapping.satu_acc", 3.0);
    // nh->declare_parameter<double>("mapping.satu_gyro", 35.0);
    // nh->declare_parameter<double>("mapping.acc_norm", 1.0);
    // nh->declare_parameter<float>("mapping.plane_thr", 0.05f);
    // nh->declare_parameter<int>("point_filter_num", 2);
    // nh->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
    // nh->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
    // nh->declare_parameter<bool>("common.con_frame", false);
    // nh->declare_parameter<int>("common.con_frame_num", 1);
    // nh->declare_parameter<bool>("common.cut_frame", false);
    // nh->declare_parameter<double>("common.cut_frame_time_interval", 0.1);
    // nh->declare_parameter<double>("common.time_lag_imu_to_lidar", 0.0);
    // nh->declare_parameter<double>("filter_size_surf", 0.5);
    // nh->declare_parameter<double>("filter_size_map", 0.5);
    // nh->declare_parameter<double>("cube_side_length", 200);
    // nh->declare_parameter<float>("mapping.det_range", 300.f);
    // nh->declare_parameter<double>("mapping.fov_degree", 180);
    // nh->declare_parameter<bool>("mapping.imu_en", true);
    // nh->declare_parameter<bool>("mapping.start_in_aggressive_motion", false);
    // nh->declare_parameter<bool>("mapping.extrinsic_est_en", true);
    // nh->declare_parameter<double>("mapping.imu_time_inte", 0.005);
    // nh->declare_parameter<double>("mapping.lidar_meas_cov", 0.1);
    // nh->declare_parameter<double>("mapping.acc_cov_input", 0.1);
    // nh->declare_parameter<double>("mapping.vel_cov", 20);
    // nh->declare_parameter<double>("mapping.gyr_cov_input", 0.1);
    // nh->declare_parameter<double>("mapping.gyr_cov_output", 0.1);
    // nh->declare_parameter<double>("mapping.acc_cov_output", 0.1);
    // nh->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
    // nh->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
    // nh->declare_parameter<double>("mapping.imu_meas_acc_cov", 0.1);
    // nh->declare_parameter<double>("mapping.imu_meas_omg_cov", 0.1);
    // nh->declare_parameter<double>("preprocess.blind", 1.0);
    // nh->declare_parameter<int>("preprocess.lidar_type", 1);
    // nh->declare_parameter<int>("preprocess.scan_line", 16);
    // nh->declare_parameter<int>("preprocess.scan_rate", 10);
    // nh->declare_parameter<int>("preprocess.timestamp_unit", 1);
    // nh->declare_parameter<double>("mapping.match_s", 81);
    // nh->declare_parameter<bool>("mapping.gravity_align", true);
    // nh->declare_parameter<std::vector<double>>("mapping.gravity", {0, 0, -9.810});
    // nh->declare_parameter<std::vector<double>>("mapping.gravity_init", {0, 0, -9.810});
    // nh->declare_parameter<std::vector<double>>("mapping.extrinsic_T", {0, 0, 0});
    // nh->declare_parameter<std::vector<double>>("mapping.extrinsic_R", {1, 0, 0, 0, 1, 0, 0, 0,
    // 1}); nh->declare_parameter<bool>("odometry.publish_odometry_without_downsample", false);
    // nh->declare_parameter<bool>("publish.path_en", true);
    // nh->declare_parameter<bool>("publish.scan_publish_en", true);
    // nh->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
    // nh->declare_parameter<bool>("runtime_pos_log_enable", false);
    // nh->declare_parameter<bool>("pcd_save.pcd_save_en", false);
    // nh->declare_parameter<int>("pcd_save.interval", -1);

    node->get_parameter("prop_at_freq_of_imu", prop_at_freq_of_imu);
    node->get_parameter("use_imu_as_input", use_imu_as_input);
    node->get_parameter("check_satu", check_satu);
    node->get_parameter("init_map_size", init_map_size);
    node->get_parameter("space_down_sample", space_down_sample);
    node->get_parameter("point_filter_num", p_pre->point_filter_num);
    node->get_parameter("cube_side_length", cube_len);
    node->get_parameter("filter_size_surf", filter_size_surf_min);
    node->get_parameter("filter_size_map", filter_size_map_min);
    node->get_parameter("runtime_pos_log_enable", runtime_pos_log);

    node->get_parameter("mapping.satu_acc", satu_acc);
    node->get_parameter("mapping.satu_gyro", satu_gyro);
    node->get_parameter("mapping.acc_norm", acc_norm);
    node->get_parameter("mapping.plane_thr", plane_thr);
    node->get_parameter("mapping.det_range", DET_RANGE);
    node->get_parameter("mapping.fov_degree", fov_deg);
    node->get_parameter("mapping.imu_en", imu_en);
    node->get_parameter("mapping.start_in_aggressive_motion", non_station_start);
    node->get_parameter("mapping.extrinsic_est_en", extrinsic_est_en);
    node->get_parameter("mapping.imu_time_inte", imu_time_inte);
    node->get_parameter("mapping.lidar_meas_cov", laser_point_cov);
    node->get_parameter("mapping.acc_cov_input", acc_cov_input);
    node->get_parameter("mapping.vel_cov", vel_cov);
    node->get_parameter("mapping.gyr_cov_input", gyr_cov_input);
    node->get_parameter("mapping.gyr_cov_output", gyr_cov_output);
    node->get_parameter("mapping.acc_cov_output", acc_cov_output);
    node->get_parameter("mapping.b_gyr_cov", b_gyr_cov);
    node->get_parameter("mapping.b_acc_cov", b_acc_cov);
    node->get_parameter("mapping.imu_meas_acc_cov", imu_meas_acc_cov);
    node->get_parameter("mapping.imu_meas_omg_cov", imu_meas_omg_cov);
    node->get_parameter("mapping.match_s", match_s);
    node->get_parameter("mapping.gravity_align", gravity_align);
    node->get_parameter("mapping.gravity", gravity);
    node->get_parameter("mapping.gravity_init", gravity_init);
    node->get_parameter("mapping.extrinsic_T", extrinT);
    node->get_parameter("mapping.extrinsic_R", extrinR);

    node->get_parameter("common.lid_topic", lid_topic);
    node->get_parameter("common.imu_topic", imu_topic);
    node->get_parameter("common.con_frame", con_frame);
    node->get_parameter("common.con_frame_num", con_frame_num);
    node->get_parameter("common.cut_frame", cut_frame);
    node->get_parameter("common.cut_frame_time_interval", cut_frame_time_interval);
    node->get_parameter("common.time_lag_imu_to_lidar", time_lag_imu_to_lidar);

    node->get_parameter("preprocess.blind", p_pre->blind);
    node->get_parameter("preprocess.lidar_type", lidar_type);
    node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
    node->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);
    node->get_parameter("preprocess.timestamp_unit", p_pre->time_unit);

    node->get_parameter("publish.path_en", path_en);
    node->get_parameter("publish.scan_publish_en", scan_pub_en);
    node->get_parameter("publish.scan_bodyframe_pub_en", scan_body_pub_en);

    node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
    node->get_parameter("pcd_save.interval", pcd_save_interval);

    node->get_parameter(
        "odometry.publish_odometry_without_downsample", publish_odometry_without_downsample);
}
