#pragma once

#include "../common/common_lib.hpp"
#include "../common/so3_math.hpp"

#include <cmath>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <math.h>
#include <mutex>
#include <thread>

#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

/// *************Preconfiguration

#define MAX_INI_COUNT (100)

/// *************IMU Process and undistortion
class ImuProcess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess()
        : is_first_frame_(true)
        , is_need_initialize_(true)
        , gravity_align_(false)
        , logger_(rclcpp::get_logger("laserMapping"))
    {
        imu_en = true;
        initialize_count_ = 1;
        mean_acc = V3D(0, 0, -1.0);
        mean_gyr_ = V3D(0, 0, 0);
    }

    void reset()
    {
        RCLCPP_WARN(logger_, "Reset ImuProcess");
        mean_acc = V3D(0, 0, -1.0);
        mean_gyr_ = V3D(0, 0, 0);
        is_need_initialize_ = true;
        initialize_count_ = 1;
    }

    void process(const CombinedPackage& package, PointCloudXYZI::Ptr& point_cloud_undistorted)
    {
        if (imu_en) {
            if (package.imu.empty())
                return;

            assert(package.lidar != nullptr);

            if (is_need_initialize_) {
                /// The very first lidar frame
                imu_init(package);

                is_need_initialize_ = true;

                if (initialize_count_ > MAX_INI_COUNT) {
                    RCLCPP_INFO(logger_, "IMU Initializing: %.1f %%", 100.0);
                    is_need_initialize_ = false;
                    *point_cloud_undistorted = *(package.lidar);
                }
                return;
            }
            if (!gravity_align_)
                gravity_align_ = true;
            *point_cloud_undistorted = *(package.lidar);
            return;

        } else {
            if (!is_first_frame_) {

                if (!gravity_align_)
                    gravity_align_ = true;

            } else {

                is_first_frame_ = false;
                return;
            }

            *point_cloud_undistorted = *(package.lidar);
            return;
        }
    }

    void set_init(Eigen::Vector3d& tmp_gravity, Eigen::Matrix3d& rot)
    {
        /** 1. initializing the gravity, gyro bias, acc and gyro covariance
         ** 2. normalize the acceleration measurenments to unit gravity **/
        // V3D tmp_gravity = - mean_acc / mean_acc.norm() * G_m_s2; // state_gravity;
        M3D hat_grav;
        hat_grav << 0.0, gravity_(2), -gravity_(1),
            -gravity_(2), 0.0, gravity_(0),
            gravity_(1), -gravity_(0), 0.0;
        double align_norm = (hat_grav * tmp_gravity).norm() / tmp_gravity.norm() / gravity_.norm();
        double align_cos = gravity_.transpose() * tmp_gravity;
        align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();
        if (align_norm < 1e-6) {
            if (align_cos > 1e-6) {
                rot = Eye3d;
            } else {
                rot = -Eye3d;
            }
        } else {
            V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos);
            rot = Exp(align_angle(0), align_angle(1), align_angle(2));
        }
    }

    ofstream fout_imu;
    // double first_lidar_time;
    int lidar_type;
    bool imu_en;
    V3D mean_acc, gravity_;
    bool is_need_initialize_ = true;
    bool is_first_frame_ = true;
    bool gravity_align_ = false;

private:
    void imu_init(const CombinedPackage& package)
    {
        // 1. initializing the gravity, gyro bias, acc and gyro covariance
        // 2. normalize the acceleration measurements to unit gravity
        RCLCPP_INFO(logger_, "IMU Initializing: %.1f %%", double(initialize_count_) / MAX_INI_COUNT * 100);

        auto current_acc = Eigen::Matrix<double, 3, 1>();
        auto current_gyr = Eigen::Matrix<double, 3, 1>();

        if (is_first_frame_) {
            this->reset();
            initialize_count_ = 1;
            is_first_frame_ = false;
            const auto& imu_acc = package.imu.front()->linear_acceleration;
            const auto& gyr_acc = package.imu.front()->angular_velocity;
            mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
            mean_gyr_ << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        }

        for (const auto& imu : package.imu) {

            current_acc << imu->linear_acceleration.x,
                imu->linear_acceleration.y,
                imu->linear_acceleration.z;
            current_gyr << imu->angular_velocity.x,
                imu->angular_velocity.y,
                imu->angular_velocity.z;

            mean_acc += (current_acc - mean_acc) / initialize_count_;
            mean_gyr_ += (current_gyr - mean_gyr_) / initialize_count_;

            initialize_count_++;
        }
    }

    V3D mean_gyr_;
    int initialize_count_ = 1;
    rclcpp::Logger logger_;
};
