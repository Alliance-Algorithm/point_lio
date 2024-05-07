#include "laser_mapping.hpp"
#include <cstdint>
#include <memory>
#include <pcl/common/io.h>

void transform_from_lidar_to_imu(PointType const* const lidar_frame, PointType* const imu_frame)
{
    auto point_body_lidar = V3D(lidar_frame->x, lidar_frame->y, lidar_frame->z);
    auto point_body_imu = V3D();

    if (!extrinsic_est_en) {
        point_body_imu = Lidar_R_wrt_IMU * point_body_lidar + Lidar_T_wrt_IMU;

    } else {
        if (!use_imu_as_input) {
            point_body_imu = kf_output.x_.offset_R_L_I.normalized() * point_body_lidar + kf_output.x_.offset_T_L_I;

        } else {
            point_body_imu = kf_input.x_.offset_R_L_I.normalized() * point_body_lidar + kf_input.x_.offset_T_L_I;
        }
    }

    imu_frame->x = point_body_imu(0);
    imu_frame->y = point_body_imu(1);
    imu_frame->z = point_body_imu(2);

    imu_frame->intensity = lidar_frame->intensity;
}

void set_pose_stamp(geometry_msgs::msg::Pose& pose)
{
    if (!use_imu_as_input) {
        pose.position.x = kf_output.x_.pos(0);
        pose.position.y = kf_output.x_.pos(1);
        pose.position.z = kf_output.x_.pos(2);
        pose.orientation.x = kf_output.x_.rot.coeffs()[0];
        pose.orientation.y = kf_output.x_.rot.coeffs()[1];
        pose.orientation.z = kf_output.x_.rot.coeffs()[2];
        pose.orientation.w = kf_output.x_.rot.coeffs()[3];
    } else {
        pose.position.x = kf_input.x_.pos(0);
        pose.position.y = kf_input.x_.pos(1);
        pose.position.z = kf_input.x_.pos(2);
        pose.orientation.x = kf_input.x_.rot.coeffs()[0];
        pose.orientation.y = kf_input.x_.rot.coeffs()[1];
        pose.orientation.z = kf_input.x_.rot.coeffs()[2];
        pose.orientation.w = kf_input.x_.rot.coeffs()[3];
    }
}

void publish_init_ikd_tree(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
{
    auto laser_init_cloud = PointCloudXYZI::Ptr(new PointCloudXYZI(ikd_tree.size(), 1));
    auto msg = sensor_msgs::msg::PointCloud2();

    ikd_tree.PCL_Storage.clear();
    ikd_tree.flatten(ikd_tree.Root_Node, ikd_tree.PCL_Storage, NOT_RECORD);

    laser_init_cloud->points = ikd_tree.PCL_Storage;
    pcl::toROSMsg(*laser_init_cloud, msg);

    msg.header.stamp = get_ros_time(lidar_end_time);
    msg.header.frame_id = "lidar_init";

    publisher->publish(msg);
}

void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher, const PointCloudXYZI::Ptr& point)
{
    auto msg = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*point, msg);
    msg.header.stamp = get_ros_time(lidar_end_time);
    msg.header.frame_id = "lidar_init";

    publisher->publish(msg);
}

void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
{
    int size = point_cloud_undistorted->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        transform_from_lidar_to_imu(&point_cloud_undistorted->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    publisher->publish(laserCloudmsg);
}

void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& publisher, std::shared_ptr<tf2_ros::TransformBroadcaster>& broadcaster)
{
    auto odom = nav_msgs::msg::Odometry();

    odom.header.frame_id = "lidar_init";
    odom.child_frame_id = "lidar_link";

    odom.header.stamp = publish_odometry_without_down_sample ? get_ros_time(time_current) : get_ros_time(lidar_end_time);

    set_pose_stamp(odom.pose.pose);

    publisher->publish(odom);

    auto transform = geometry_msgs::msg::TransformStamped();

    transform.header.frame_id = "lidar_init";
    transform.child_frame_id = "lidar_link";
    transform.header.stamp = odom.header.stamp;

    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;

    transform.transform.rotation.w = odom.pose.pose.orientation.w;
    transform.transform.rotation.x = odom.pose.pose.orientation.x;
    transform.transform.rotation.y = odom.pose.pose.orientation.y;
    transform.transform.rotation.z = odom.pose.pose.orientation.z;

    broadcaster->sendTransform(transform);
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher)
{
    auto pose = geometry_msgs::msg::PoseStamped();

    set_pose_stamp(pose.pose);

    pose.header.stamp = get_ros_time(lidar_end_time);
    pose.header.frame_id = "lidar_init";

    // if path is too large, the rviz2 will crash
    path.poses.emplace_back(pose);
    publisher->publish(path);
}
