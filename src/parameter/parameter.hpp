#pragma once

#include "../preprocess/preprocess.h"

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include <cstring>
#include <string>

inline bool is_first_frame = true;
inline double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;

// subscription callback
inline double stamp_lidar_last = -1.0;
inline double stamp_imu_last = -1.0;

// map save process
inline int pcd_index = 0;

inline std::string lid_topic, imu_topic;
inline bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
inline bool use_imu_as_input, space_down_sample, publish_odometry_without_down_sample;
inline int init_map_size, con_frame_num;
inline double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
inline float plane_thr;
inline double filter_size_surf_min, filter_size_map_min, fov_deg;
inline double cube_len;
inline float DET_RANGE;
inline bool imu_en, gravity_align, is_start_in_aggressive_motion;
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
inline bool scan_publish_enable, scan_body_pub_en;
inline shared_ptr<Preprocess> p_pre;
inline double lag_between_imu_and_lidar = 0.0;

// premapping
inline bool is_premapping_enable = false;
inline std::string pcd_path;

inline void readParameters(shared_ptr<rclcpp::Node>& node)
{

    p_pre.reset(new Preprocess());

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
    node->get_parameter("mapping.start_in_aggressive_motion", is_start_in_aggressive_motion);
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
    node->get_parameter("common.time_lag_imu_to_lidar", lag_between_imu_and_lidar);

    node->get_parameter("preprocess.blind", p_pre->blind);
    node->get_parameter("preprocess.lidar_type", lidar_type);
    node->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
    node->get_parameter("preprocess.scan_rate", p_pre->SCAN_RATE);
    node->get_parameter("preprocess.timestamp_unit", p_pre->time_unit);

    node->get_parameter("publish.path_en", path_en);
    node->get_parameter("publish.scan_publish_en", scan_publish_enable);
    node->get_parameter("publish.scan_body_link_pub_en", scan_body_pub_en);

    node->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
    node->get_parameter("pcd_save.interval", pcd_save_interval);

    node->get_parameter("odometry.publish_odometry_without_downsample", publish_odometry_without_down_sample);

    node->get_parameter("pre_mapping.enable", is_premapping_enable);
    node->get_parameter("pre_mapping.pcd_path", pcd_path);
}
