#pragma once

#include <cmath>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <math.h>
#include <mutex>
#include <thread>

#include <common_lib.h>
#include <so3_math.h>

#include <Eigen/Eigen>
// #include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rcpputils/asserts.hpp>
#include <tf2_ros/transform_broadcaster.h>

/// *************Preconfiguration

#define MAX_INI_COUNT (100)

/// *************IMU Process and undistortion
class ImuProcess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void Reset();
    void Reset(double start_timestamp, const sensor_msgs::msg::Imu::SharedPtr& lastimu);
    void Process(const MeasureGroup& meas, PointCloudXYZI::Ptr pcl_un_);
    void Set_init(Eigen::Vector3d& tmp_gravity, Eigen::Matrix3d& rot);

    ofstream fout_imu;
    // double first_lidar_time;
    int lidar_type;
    bool imu_en;
    V3D mean_acc, gravity_;
    bool imu_need_init_ = true;
    bool b_first_frame_ = true;
    bool gravity_align_ = false;

private:
    void IMU_init(const MeasureGroup& meas, int& N);
    V3D mean_gyr;
    int init_iter_num = 1;
};
