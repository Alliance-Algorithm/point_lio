#pragma once

#include "./common/so3_math.hpp"
#include "./estimator/Estimator.h"
#include "./imu_process/imu_process.hpp"
#include "./parameter/parameter.hpp"

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

#define MAXN            (720000)
#define PUBFRAME_PERIOD (20)

extern string root_dir;

extern int feats_down_size, time_log_counter;

extern double time_update_last, time_current, time_predict_last_const, t_last;

extern shared_ptr<ImuProcess> p_imu;
extern bool init_map, flg_first_scan;

// Time Log Variables
extern double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
extern double match_time, solve_time, propag_time, update_time;

extern bool flg_reset, flg_exit;

extern deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

// surf feature in map
extern PointCloudXYZI::Ptr feats_undistort;
extern PointCloudXYZI::Ptr init_feats_world;

extern pcl::VoxelGrid<PointType> downSizeFilterSurf;
extern pcl::VoxelGrid<PointType> downSizeFilterMap;

extern V3D euler_cur;

extern MeasureGroup Measures;

extern sensor_msgs::msg::Imu imu_last, imu_next;
extern nav_msgs::msg::Path path;
extern nav_msgs::msg::Odometry odomAftMapped;
extern geometry_msgs::msg::PoseStamped msg_body_pose;

extern rclcpp::Logger logger;

extern PointCloudXYZI::Ptr pcl_wait_save;

void SigHandle(int sig);
void dump_lio_state_to_log(FILE* fp);
void lasermap_fov_segment();
bool sync_packages(MeasureGroup& meas);
void map_incremental();

void publish_init_kdtree(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFullRes);
void publish_frame_world(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFullRes);
void publish_frame_body(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFull_body);
void publish_odometry(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pubOdomAftMapped,
    std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_br);
void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pubPath);

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in);