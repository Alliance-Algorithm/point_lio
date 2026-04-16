#include "collection.h"
#include "li_initialization.h"
#include "util/transform.hpp"

#include <atomic>
#include <thread>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std;

bool init_map = false, flg_first_scan = true;

bool flg_reset = false, flg_exit = false;

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort_world(new PointCloudXYZI());
pcl::VoxelGrid<PointType> downSizeFilterSurf;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

auto LOGGER = rclcpp::get_logger("laserMapping");

void SigHandle(int sig) {
    flg_exit = true;
    RCLCPP_WARN(LOGGER, "catch sig %d", sig);
    sig_buffer.notify_all();
}

void pointBodyLidarToIMU(PointType const* const pi, PointType* const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
        } else {
            p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    } else {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void MapIncremental() {
    PointVector points_to_add;
    int cur_pts = feats_down_world->size();
    points_to_add.reserve(cur_pts);

    for (size_t i = 0; i < cur_pts; ++i) {
        PointType& point_world = feats_down_world->points[i];
        if (!Nearest_Points[i].empty()) {
            const PointVector& points_near = Nearest_Points[i];

            Eigen::Vector3f center =
                ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5)
                * filter_size_map_min;
            bool need_add = true;
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
                if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min
                    && fabs(dis_2_center.y()) < 0.5 * filter_size_map_min
                    && fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) {
                points_to_add.emplace_back(point_world);
            }
        } else {
            points_to_add.emplace_back(point_world);
        }
    }
    ivox_->AddPoints(points_to_add);
}

void publish_frame_world(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFullRes) {
    if (scan_pub_en) {
        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*feats_down_world, laserCloudmsg);

        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFullRes->publish(laserCloudmsg);
    }
}

void publish_frame_body(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFull_body) {
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "base_link";
    pubLaserCloudFull_body->publish(laserCloudmsg);
}

void publish_frame_world_undistort(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
        pubLaserCloudFullResUndistort) {
    if (scan_pub_en) {
        /// Transform the undistorted body-frame cloud into the world frame.
        feats_undistort_world->clear();
        feats_undistort_world->resize(feats_undistort->points.size());

        for (size_t i = 0; i < feats_undistort->points.size(); i++) {
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_undistort_world->points[i]));
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*feats_undistort_world, laserCloudmsg);

        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFullResUndistort->publish(laserCloudmsg);
    }
}

template <typename T>
void set_posestamp(T& out) {
    if (!use_imu_as_input) {
        out.position.x = kf_output.x_.pos(0);
        out.position.y = kf_output.x_.pos(1);
        out.position.z = kf_output.x_.pos(2);
        Eigen::Quaterniond q(kf_output.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    } else {
        out.position.x = kf_input.x_.pos(0);
        out.position.y = kf_input.x_.pos(1);
        out.position.z = kf_input.x_.pos(2);
        Eigen::Quaterniond q(kf_input.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    }
}

void publish_odometry(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& pubOdomAftMapped,
    std::shared_ptr<tf2_ros::TransformBroadcaster>& tf_br) {
    odomAftMapped.header.frame_id = "odom";
    odomAftMapped.child_frame_id = "base_link";
    if (publish_odometry_without_downsample) {
        odomAftMapped.header.stamp = get_ros_time(time_current);
    } else {
        odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);

    pubOdomAftMapped->publish(odomAftMapped);

    if (tf_send_en) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
        transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
        transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
        transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
        transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
        transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
        transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
        transform.header.stamp = odomAftMapped.header.stamp;
        tf_br->sendTransform(transform);
    }
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath) {
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = "odom";
    path.poses.emplace_back(msg_body_pose);
    pubPath->publish(path);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("laserMapping");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh);

    readParameters(nh);
    std::cout << "lidar_type: " << lidar_type << '\n';
    ivox_ = std::make_shared<IVoxType>(ivox_options_);

    path.header.stamp = get_ros_time(lidar_end_time);
    path.header.frame_id = "odom";

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    downSizeFilterSurf.setLeafSize(
        filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    Lidar_T_wrt_IMU = vector3d_from_array(extrinT);
    Lidar_R_wrt_IMU = matrix3d_from_array(extrinR);

    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        } else {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }

    p_imu->lidar_type = p_pre->lidar_type = lidar_type;
    p_imu->imu_en = imu_en;

    kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_3h(
        get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    Eigen::Matrix<double, 24, 24> P_init;
    reset_cov(P_init);
    kf_input.change_P(P_init);
    Eigen::Matrix<double, 30, 30> P_init_output;
    reset_cov_output(P_init_output);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();

    /// ROS subscription and publication setup.
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
    if (p_pre->lidar_type == AVIA) {
        sub_pcl_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, rclcpp::SensorDataQoS(),
            [](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { livox_pcl_cbk(msg); });
    } else {
        sub_pcl_pc = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, rclcpp::SensorDataQoS(),
            [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { standard_pcl_cbk(msg); });
    }
    auto sub_imu =
        nh->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);
    auto pub_laser_cloud_full_res =
        nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 1000);
    auto pub_laser_cloud_full_res_body =
        nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body", 1000);
    auto pub_laser_cloud_full_res_undistort =
        nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_undistort", 1000);
    auto pub_odom_aft_mapped = nh->create_publisher<nav_msgs::msg::Odometry>("odom", 1000);

    auto pub_path = nh->create_publisher<nav_msgs::msg::Path>("path", 1000);
    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);

    auto collection = std::make_shared<point_lio::Collection>();
    collection->set_time_limit(20min);
    collection->set_saving_path(std::filesystem::path{pcd_saving_path});
    collection->start_timing();

    auto save_pcd_service = nh->create_service<std_srvs::srv::Trigger>(
        "save_pcd_map", [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
                            const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
            std::ignore = request;
            if (!collection->try_start_saving()) {
                response->success = false;
                response->message = "Save request rejected, map saving is in progress";
                return;
            }

            response->success = true;
            response->message = "Save request accepted, check path " + pcd_saving_path;

            std::thread([collection] {
                const auto filename = collection->save_once();
                RCLCPP_INFO(LOGGER, "Map has been saved to: %s", filename.string().c_str());
            }).detach();
        });

    signal(SIGINT, SigHandle);
    rclcpp::Rate rate(500);

    /// Main Loop

    while (rclcpp::ok() && !flg_exit) {
        executor.spin_some();

        if (sync_packages(Measures)) {
            {
                using namespace point_lio;

                /// Compute the initial base_link pose relative to lidar.
                const auto base_from_lidar_rotation =
                    util::Quaternion<double>::from_angle(init_pose_orientation).toRotationMatrix();
                const auto base_from_lidar_translation = init_pose_translation;

                /// Read the lidar-to-imu extrinsics.
                auto lidar_to_imu_rotation = Eigen::Matrix3d{};
                lidar_to_imu_rotation << extrinR[0], extrinR[1], extrinR[2], extrinR[3], extrinR[4],
                    extrinR[5], extrinR[6], extrinR[7], extrinR[8];

                auto lidar_in_imu_translation = Eigen::Vector3d{};
                lidar_in_imu_translation << extrinT[0], extrinT[1], extrinT[2];

                /// Derive the imu pose relative to base_link.
                const auto base_from_imu_rotation =
                    base_from_lidar_rotation * lidar_to_imu_rotation.transpose();

                const auto imu_in_base_translation =
                    base_from_lidar_translation - base_from_imu_rotation * lidar_in_imu_translation;

                const auto imu_to_base_lever_arm = -imu_in_base_translation;

                /// Convert incoming lidar points directly into base_link.
                for (auto& point : Measures.lidar->points) {
                    const auto point_in_lidar = util::Vector3<double>::from_xyz(point);

                    const auto point_in_base =
                        base_from_lidar_rotation * point_in_lidar + base_from_lidar_translation;

                    util::Vector3<double>{point_in_base}.sync_xyz(point);
                }

                /// Keep the previous angular velocity for lever-arm compensation.
                auto has_prev_angular_velocity = false;
                auto prev_angular_velocity = Eigen::Vector3d{};
                auto prev_imu_time = 0.0;

                const auto transform_imu =
                    [&](std::deque<std::shared_ptr<const sensor_msgs::msg::Imu>>& data) {
                        for (auto& imu : data) {
                            if (imu->header.frame_id != "base_link") {
                                auto corrected_imu = std::make_shared<sensor_msgs::msg::Imu>(*imu);

                                auto angular_velocity = Eigen::Vector3d{
                                    base_from_imu_rotation
                                    * util::Vector3<double>::from_xyz(imu->angular_velocity)};

                                auto linear_acceleration = Eigen::Vector3d{
                                    base_from_imu_rotation
                                    * util::Vector3<double>::from_xyz(imu->linear_acceleration)};

                                const auto imu_time = get_time_sec(imu->header.stamp);
                                auto angular_acceleration =
                                    Eigen::Vector3d{Eigen::Vector3d::Zero()};

                                if (has_prev_angular_velocity && imu_time > prev_imu_time) {
                                    angular_acceleration =
                                        (angular_velocity - prev_angular_velocity)
                                        / (imu_time - prev_imu_time);
                                }

                                /// Add the lever-arm induced acceleration.
                                const auto lever_arm_acceleration =
                                    angular_acceleration.cross(imu_to_base_lever_arm)
                                    + angular_velocity.cross(
                                        angular_velocity.cross(imu_to_base_lever_arm));

                                linear_acceleration += lever_arm_acceleration * acc_norm / G_m_s2;

                                util::Vector3<double>{angular_velocity}.sync_xyz(
                                    corrected_imu->angular_velocity);

                                util::Vector3<double>{linear_acceleration}.sync_xyz(
                                    corrected_imu->linear_acceleration);

                                corrected_imu->header.frame_id = "base_link";
                                imu = corrected_imu;
                            }

                            /// Update the previous angular velocity sample.
                            prev_angular_velocity =
                                util::Vector3<double>::from_xyz(imu->angular_velocity);
                            prev_imu_time = get_time_sec(imu->header.stamp);
                            has_prev_angular_velocity = true;
                        }
                    };
                transform_imu(Measures.imu);
                transform_imu(imu_deque);

                /// Synchronize the imu samples at both ends of the scan window.
                if (!Measures.imu.empty()) {
                    imu_last = *Measures.imu.back();
                }

                if (!imu_deque.empty()) {
                    imu_next = *imu_deque.front();
                }

                /// Process the remaining pipeline directly in base_link.
                Lidar_R_wrt_IMU = Eye3d;
                Lidar_T_wrt_IMU = Zero3d;
                kf_input.x_.offset_R_L_I = Eye3d;
                kf_input.x_.offset_T_L_I = Zero3d;
                kf_output.x_.offset_R_L_I = Eye3d;
                kf_output.x_.offset_T_L_I = Zero3d;
            }

            if (flg_reset) {
                RCLCPP_WARN(LOGGER, "reset when rosbag play back");
                p_imu->Reset();
                feats_undistort.reset(new PointCloudXYZI());
                if (use_imu_as_input) {
                    state_in = state_input();
                    kf_input.change_P(P_init);
                } else {
                    state_out = state_output();
                    kf_output.change_P(P_init_output);
                }
                flg_first_scan = true;
                is_first_frame = true;
                flg_reset = false;
                init_map = false;

                { ivox_.reset(new IVoxType(ivox_options_)); }
            }

            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                if (first_imu_time < 1) {
                    first_imu_time = get_time_sec(imu_next.header.stamp);
                    printf("first imu time: %f\n", first_imu_time);
                }
                time_current = 0.0;
                if (imu_en) {
                    kf_input.x_.gravity = vector3d_from_array(gravity);
                    kf_output.x_.gravity = vector3d_from_array(gravity);

                    {
                        while (Measures.lidar_beg_time > get_time_sec(imu_next.header.stamp)) {
                            imu_deque.pop_front();
                            if (imu_deque.empty()) {
                                break;
                            }
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                        }
                    }
                } else {
                    kf_input.x_.gravity = vector3d_from_array(gravity);
                    kf_output.x_.gravity = vector3d_from_array(gravity);
                    kf_output.x_.acc = vector3d_from_array(gravity);
                    kf_output.x_.acc *= -1;
                    p_imu->imu_need_init_ = false;
                }
                G_m_s2 = std::sqrt(
                    gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
            }

            /// Downsample the feature points in the current scan.
            p_imu->Process(Measures, feats_undistort);
            if (space_down_sample) {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            } else {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            }
            {
                time_seq = time_compressing<int>(feats_down_body);
                feats_down_size = feats_down_body->points.size();
            }

            if (!p_imu->after_imu_init_) {
                if (!p_imu->imu_need_init_) {
                    V3D tmp_gravity;
                    if (imu_en) {
                        tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
                    } else {
                        tmp_gravity = vector3d_from_array(gravity_init);
                        p_imu->after_imu_init_ = true;
                    }
                    M3D rot_init;
                    p_imu->Set_init(tmp_gravity, rot_init);
                    kf_input.x_.rot = rot_init;
                    kf_output.x_.rot = rot_init;
                    kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
                } else {
                    continue;
                }
            }
            /// Initialize the map.
            if (!init_map) {
                feats_down_world->resize(feats_undistort->size());
                for (int i = 0; i < feats_undistort->size(); i++) {
                    pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                if (init_feats_world->size() < init_map_size) {
                    init_map = false;
                } else {
                    ivox_->AddPoints(init_feats_world->points);

                    init_feats_world.reset(new PointCloudXYZI());
                    init_map = true;
                }
                continue;
            }

            /// Run ICP and update the Kalman filter.
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            Nearest_Points.resize(feats_down_size);

            /// Iterated state estimation.
            crossmat_list.reserve(feats_down_size);
            pbody_list.reserve(feats_down_size);

            for (size_t i = 0; i < feats_down_body->size(); i++) {
                V3D point_this(
                    feats_down_body->points[i].x, feats_down_body->points[i].y,
                    feats_down_body->points[i].z);
                pbody_list[i] = point_this;
                if (!extrinsic_est_en) {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                    M3D point_crossmat = skew_sym_mat(point_this);
                    crossmat_list[i] = point_crossmat;
                }
            }
            if (!use_imu_as_input) {
                bool imu_upda_cov = false;
                effct_feat_num = 0;
                /// Point-by-point update.
                if (time_seq.size() > 0) {
                    double pcl_beg_time = Measures.lidar_beg_time;
                    idx = -1;
                    for (k = 0; k < time_seq.size(); k++) {
                        PointType& point_body = feats_down_body->points[idx + time_seq[k]];

                        time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                        if (is_first_frame) {
                            if (imu_en) {
                                while (time_current > get_time_sec(imu_next.header.stamp)) {
                                    imu_deque.pop_front();
                                    if (imu_deque.empty())
                                        break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front());
                                }
                                angvel_avr << imu_last.angular_velocity.x,
                                    imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                                acc_avr << imu_last.linear_acceleration.x,
                                    imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            }
                            is_first_frame = false;
                            imu_upda_cov = true;
                            time_update_last = time_current;
                            time_predict_last_const = time_current;
                        }
                        if (imu_en && !imu_deque.empty()) {
                            bool last_imu = get_time_sec(imu_next.header.stamp)
                                         == get_time_sec(imu_deque.front()->header.stamp);
                            while (get_time_sec(imu_next.header.stamp) < time_predict_last_const
                                   && !imu_deque.empty()) {
                                if (!last_imu) {
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front());
                                    break;
                                } else {
                                    imu_deque.pop_front();
                                    if (imu_deque.empty())
                                        break;
                                    imu_last = imu_next;
                                    imu_next = *(imu_deque.front());
                                }
                            }
                            bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                            while (imu_comes) {
                                imu_upda_cov = true;
                                angvel_avr << imu_next.angular_velocity.x,
                                    imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                                acc_avr << imu_next.linear_acceleration.x,
                                    imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                                /// Covariance update.
                                double dt =
                                    get_time_sec(imu_next.header.stamp) - time_predict_last_const;
                                kf_output.predict(dt, Q_output, input_in, true, false);
                                time_predict_last_const = get_time_sec(imu_next.header.stamp);

                                {
                                    double dt_cov =
                                        get_time_sec(imu_next.header.stamp) - time_update_last;

                                    if (dt_cov > 0.0) {
                                        time_update_last = get_time_sec(imu_next.header.stamp);
                                        kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        kf_output.update_iterated_dyn_share_IMU();
                                    }
                                }
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                            }
                        }
                        if (flg_reset) {
                            break;
                        }

                        double dt = time_current - time_predict_last_const;
                        if (!prop_at_freq_of_imu) {
                            double dt_cov = time_current - time_update_last;
                            if (dt_cov > 0.0) {
                                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                time_update_last = time_current;
                            }
                        }
                        kf_output.predict(dt, Q_output, input_in, true, false);
                        time_predict_last_const = time_current;

                        if (feats_down_size < 1) {
                            RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");
                            idx += time_seq[k];
                            continue;
                        }
                        if (!kf_output.update_iterated_dyn_share_modified()) {
                            idx = idx + time_seq[k];
                            continue;
                        }

                        if (publish_odometry_without_downsample) {
                            publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
                        }

                        for (int j = 0; j < time_seq[k]; j++) {
                            PointType& point_body_j = feats_down_body->points[idx + j + 1];
                            PointType& point_world_j = feats_down_world->points[idx + j + 1];
                            pointBodyToWorld(&point_body_j, &point_world_j);
                        }
                        idx += time_seq[k];
                    }
                } else {
                    if (!imu_deque.empty()) {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());

                        while (get_time_sec(imu_next.header.stamp) > time_current
                               && ((
                                   get_time_sec(imu_next.header.stamp)
                                   < Measures.lidar_beg_time + lidar_time_inte))) { // >= ?
                            if (is_first_frame) {
                                {
                                    {
                                        while (get_time_sec(imu_next.header.stamp)
                                               < Measures.lidar_beg_time + lidar_time_inte) {
                                            imu_deque.pop_front();
                                            if (imu_deque.empty())
                                                break;
                                            imu_last = imu_next;
                                            imu_next = *(imu_deque.front());
                                        }
                                    }
                                    break;
                                }
                                angvel_avr << imu_last.angular_velocity.x,
                                    imu_last.angular_velocity.y, imu_last.angular_velocity.z;

                                acc_avr << imu_last.linear_acceleration.x,
                                    imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;

                                imu_upda_cov = true;
                                time_update_last = time_current;
                                time_predict_last_const = time_current;

                                is_first_frame = false;
                            }
                            time_current = get_time_sec(imu_next.header.stamp);

                            if (!is_first_frame) {
                                double dt = time_current - time_predict_last_const;
                                {
                                    double dt_cov = time_current - time_update_last;
                                    if (dt_cov > 0.0) {
                                        kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                        time_update_last = time_current;
                                    }
                                    kf_output.predict(dt, Q_output, input_in, true, false);
                                }

                                time_predict_last_const = time_current;

                                angvel_avr << imu_next.angular_velocity.x,
                                    imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                                acc_avr << imu_next.linear_acceleration.x,
                                    imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;
                                kf_output.update_iterated_dyn_share_IMU();
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            } else {
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                        }
                    }
                }
            } else {
                bool imu_prop_cov = false;
                effct_feat_num = 0;
                if (time_seq.size() > 0) {
                    double pcl_beg_time = Measures.lidar_beg_time;
                    idx = -1;
                    for (k = 0; k < time_seq.size(); k++) {
                        PointType& point_body = feats_down_body->points[idx + time_seq[k]];
                        time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                        if (is_first_frame) {
                            while (time_current > get_time_sec(imu_next.header.stamp)) {
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                            imu_prop_cov = true;

                            is_first_frame = false;
                            t_last = time_current;
                            time_update_last = time_current;
                            {
                                input_in.gyro << imu_last.angular_velocity.x,
                                    imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                                input_in.acc << imu_last.linear_acceleration.x,
                                    imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                            }
                        }

                        while (time_current > get_time_sec(imu_next.header.stamp)) {
                            imu_deque.pop_front();

                            input_in.gyro << imu_last.angular_velocity.x,
                                imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            input_in.acc << imu_last.linear_acceleration.x,
                                imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                            double dt = get_time_sec(imu_last.header.stamp) - t_last;

                            double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
                            if (dt_cov > 0.0) {
                                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                                time_update_last =
                                    get_time_sec(imu_last.header.stamp);            // time_current;
                            }
                            kf_input.predict(dt, Q_input, input_in, true, false);
                            t_last = get_time_sec(imu_last.header.stamp);
                            imu_prop_cov = true;

                            if (imu_deque.empty())
                                break;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                        }
                        if (flg_reset) {
                            break;
                        }
                        double dt = time_current - t_last;
                        t_last = time_current;
                        if (!prop_at_freq_of_imu) {
                            double dt_cov = time_current - time_update_last;
                            if (dt_cov > 0.0) {
                                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                                time_update_last = time_current;
                            }
                        }
                        kf_input.predict(dt, Q_input, input_in, true, false);

                        if (feats_down_size < 1) {
                            RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");

                            idx += time_seq[k];
                            continue;
                        }
                        if (!kf_input.update_iterated_dyn_share_modified()) {
                            idx = idx + time_seq[k];
                            continue;
                        }

                        if (publish_odometry_without_downsample) {
                            publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
                        }

                        for (int j = 0; j < time_seq[k]; j++) {
                            PointType& point_body_j = feats_down_body->points[idx + j + 1];
                            PointType& point_world_j = feats_down_world->points[idx + j + 1];
                            pointBodyToWorld(&point_body_j, &point_world_j);
                        }
                        idx = idx + time_seq[k];
                    }
                } else {
                    if (!imu_deque.empty()) {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        while (get_time_sec(imu_next.header.stamp) > time_current
                               && ((
                                   get_time_sec(imu_next.header.stamp)
                                   < Measures.lidar_beg_time + lidar_time_inte))) { // >= ?
                            if (is_first_frame) {
                                {
                                    {
                                        while (get_time_sec(imu_next.header.stamp)
                                               < Measures.lidar_beg_time + lidar_time_inte) {
                                            imu_deque.pop_front();
                                            if (imu_deque.empty())
                                                break;
                                            imu_last = imu_next;
                                            imu_next = *(imu_deque.front());
                                        }
                                    }

                                    break;
                                }
                                imu_prop_cov = true;

                                t_last = time_current;
                                time_update_last = time_current;
                                input_in.gyro << imu_last.angular_velocity.x,
                                    imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                                input_in.acc << imu_last.linear_acceleration.x,
                                    imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                                is_first_frame = false;
                            }
                            time_current = get_time_sec(imu_next.header.stamp);

                            if (!is_first_frame) {
                                double dt = time_current - t_last;

                                double dt_cov = time_current - time_update_last;
                                if (dt_cov > 0.0) {
                                    time_update_last = get_time_sec(imu_next.header.stamp);
                                }

                                t_last = get_time_sec(imu_next.header.stamp);

                                input_in.gyro << imu_next.angular_velocity.x,
                                    imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                                input_in.acc << imu_next.linear_acceleration.x,
                                    imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;
                                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            } else {
                                imu_deque.pop_front();
                                if (imu_deque.empty())
                                    break;
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                            }
                        }
                    }
                }
            }
            /// Publish the downsampled odometry result.
            if (!publish_odometry_without_downsample) {
                publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
            }

            /// Add the feature points to the map.
            if (feats_down_size > 4) {
                MapIncremental();
            }
            /// Publish point clouds.

            if (path_en)
                publish_path(pub_path);
            if (scan_pub_en)
                publish_frame_world(pub_laser_cloud_full_res);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pub_laser_cloud_full_res_body);
            if (scan_pub_en)
                publish_frame_world_undistort(pub_laser_cloud_full_res_undistort);

            collection->spin(*feats_down_world);
        }
        rate.sleep();
    }

    return 0;
}
