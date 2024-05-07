#pragma once

#include "../common/common_lib.hpp"
#include "../common/so3_math.hpp"
#include "../estimator/Estimator.h"
#include "../imu_process/imu_process.hpp"
#include "../parameter/parameter.hpp"

#include <boost/config/detail/suffix.hpp>
#include <csignal>
#include <fstream>
#include <math.h>
#include <memory>
#include <mutex>
#include <omp.h>
#include <thread>
#include <unistd.h>

// Thirdparty libraries
#include <Eigen/Core>
#include <Python.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Ros2 libraries
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

// const
inline constexpr auto time_log_size_max = 720000;
inline constexpr auto publish_frame_interval = 20;

inline auto root_dir = std::string(ROOT_DIR);

// flag
inline auto flag_map_initialized = false;
inline auto flag_first_scan = true;
inline auto flag_reset = false;
inline auto flag_exit = false;

// process
inline auto package = CombinedPackage();
inline auto imu_process = std::make_shared<ImuProcess>();
inline auto imu_buffer = deque<sensor_msgs::msg::Imu::ConstSharedPtr>();
inline auto imu_last = sensor_msgs::msg::Imu();
inline auto imu_last_ptr = sensor_msgs::msg::Imu::ConstSharedPtr();
inline auto imu_next = sensor_msgs::msg::Imu();
inline auto down_sample_voxel_grid = pcl::VoxelGrid<PointType>();
inline auto point_cloud_undistorted = std::make_shared<PointCloudXYZI>();
inline auto init_feats_world = std::make_shared<PointCloudXYZI>();
inline auto pcl_wait_save = std::make_shared<PointCloudXYZI>();

inline int cloud_sample_down_size = 0;
inline int time_log_counter = 0;
inline int scan_count = 0;

inline double time_update_last = 0.0;
inline double time_current = 0.0;
inline double time_predict_last_const = 0.0;
inline double time_last = 0.0;

inline double match_time = 0;
inline double solve_time = 0;
inline double propag_time = 0;
inline double update_time = 0;

// math
inline auto euler_cur = Eigen::Matrix<double, 3, 1>();

// log
inline auto logger = rclcpp::get_logger("laserMapping");
inline double
    log_time_1[time_log_size_max],
    log_time_0[time_log_size_max],
    log_time_2[time_log_size_max],
    log_time_3[time_log_size_max],
    log_time_11[time_log_size_max];

// publish
inline auto path = nav_msgs::msg::Path();

void signal_handle(int signal);
void dump_lio_state_to_log(FILE* fp);
void laser_map_fov_segment();
bool unpack(CombinedPackage& package);
void pre_increase_map(const PointCloudXYZI::Ptr& cloud);
void increase_map();

// ros2 interface
void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher);
void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);
void publish_init_ikd_tree(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);
void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher, const PointCloudXYZI::Ptr& point);
void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& publisher, std::shared_ptr<tf2_ros::TransformBroadcaster>& broadcaster);
void pointcloud_subscription_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
void livox_subscription_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
void imu_subscription_callback(const sensor_msgs::msg::Imu::SharedPtr msg);