#pragma once

#include "preprocess.h"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <cstring>
#include <rclcpp/rclcpp.hpp>

inline int pcd_index = 0;
inline bool is_first_frame = true;
inline double lidar_end_time = 0.0, first_lidar_time = 0.0;
inline double time_con = 0.0, last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
inline std::string lidar_topic, imu_topic;
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
inline std::vector<double> gravity_init(3, 0.0);
inline std::vector<double> gravity(3, 0.0);
inline std::vector<double> extrinT(3, 0.0);
inline std::vector<double> extrinR(9, 0.0);
inline bool runtime_pos_log, pcd_save_en, path_en, extrinsic_est_en = true;
inline bool scan_pub_en, scan_body_pub_en;
inline shared_ptr<Preprocess> preprocess = std::make_shared<Preprocess>();
inline double time_lag_imu_to_lidar = 0.0;