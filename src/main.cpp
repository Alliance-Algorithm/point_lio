#include "laser_mapping/laser_mapping.hpp"
#include <rclcpp/logging.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("mapping",
        rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true));

    readParameters(node);

    RCLCPP_INFO(logger, "lidar_type: %d", lidar_type);

    path.header.stamp = get_ros_time(lidar_end_time);
    path.header.frame_id = "lidar_init";

    // variables definition for counting
    int frame_num = 0;

    double aver_time_consu = 0;
    double aver_time_icp = 0;
    double aver_time_match = 0;
    double aver_time_incre = 0;
    double aver_time_solve = 0;
    double aver_time_propag = 0;

    std::time_t startTime, endTime;

    // initialize variables
    double FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    double HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    memset(point_selected_surf, true, sizeof(point_selected_surf));

    down_sample_voxel_grid.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

    Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

    if (extrinsic_est_en) {
        if (!use_imu_as_input) {
            kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        } else {
            kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
            kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
        }
    }

    imu_process->lidar_type = p_pre->lidar_type = lidar_type;
    imu_process->imu_en = imu_en;

    kf_input.init_dyn_share_modified(get_f_input, df_dx_input, h_model_input);
    kf_output.init_dyn_share_modified_2h(
        get_f_output, df_dx_output, h_model_output, h_model_IMU_output);

    Eigen::Matrix<double, 24, 24> P_init = MD(24, 24)::Identity() * 0.01;
    P_init.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init.block<6, 6>(15, 15) = MD(6, 6)::Identity() * 0.001;
    P_init.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;

    kf_input.change_P(P_init);

    Eigen::Matrix<double, 30, 30> P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3, 3)::Identity() * 0.0001;
    P_init_output.block<6, 6>(6, 6) = MD(6, 6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6, 6)::Identity() * 0.001;

    kf_input.change_P(P_init);
    kf_output.change_P(P_init_output);

    Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();

    // debug record
    FILE* log_file;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    log_file = fopen(pos_log_dir.c_str(), "w");

    ofstream fout_out, fout_imu_pbp;
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);

    if (fout_out && fout_imu_pbp)
        RCLCPP_INFO(logger, "log file opened: %s", root_dir.c_str());
    else
        RCLCPP_INFO(logger, "doesn't exist: %s", root_dir.c_str());

    // premapping
    auto point_premapping = std::make_shared<pcl::PointCloud<PointType>>();

    if (is_premapping_enable) {

        if (pcl::io::loadPCDFile<PointType>(pcd_path, *point_premapping) == -1) {
            RCLCPP_ERROR(logger, "pcd file open failed");
            rclcpp::shutdown();
            exit(-1);
        }

        RCLCPP_INFO(logger, "pcd file opened, size: %zu", point_premapping->size());
    }

    // ros2 interface
    auto standard_lidar_subscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr();
    auto livox_subscription = rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr();
    auto imu_subscription = node->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000, imu_subscription_callback);

    auto cloud_registered_world_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_world", 100000);
    auto cloud_registered_body_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 100000);
    auto laser_map_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_map", 100000);
    auto odometry_publisher = node->create_publisher<nav_msgs::msg::Odometry>("/position", 100000);
    auto path_publisher = node->create_publisher<nav_msgs::msg::Path>("/path", 100000);

    auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    if (p_pre->lidar_type == AVIA) {
        livox_subscription = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, 20, livox_subscription_callback);
    } else {
        standard_lidar_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, rclcpp::SensorDataQoS(), pointcloud_subscription_callback);
    }

    signal(SIGINT, signal_handle);
    rclcpp::Rate rate(5000);

    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);

    // main process
    while (rclcpp::ok()) {

        if (flag_exit)
            break;

        // handle available callback
        executor.spin_some();

        if (!unpack(package)) {
            rate.sleep();
            continue;
        }

        if (flag_first_scan) {
            first_lidar_time = package.lidar_begin_time;
            flag_first_scan = false;

            RCLCPP_WARN(logger, "first lidar time: %lf", first_lidar_time);
        }

        if (flag_reset) {
            RCLCPP_WARN(logger, "reset when ros bag play back");
            imu_process->reset();
            flag_reset = false;
            continue;
        }

        // package ready, process begin

        double time_0;
        double time_1;
        double time_2;
        double time_3;
        double time_5;
        double time_solve_start;

        match_time = 0;
        solve_time = 0;
        propag_time = 0;
        update_time = 0;

        time_0 = omp_get_wtime();

        // make undistorted point cloud using imu
        imu_process->process(package, point_cloud_undistorted);

        if (point_cloud_undistorted->empty()) {
            continue;
        }

        // use your imu
        if (!imu_process->gravity_align_ && imu_en) {
            // load imu deque, many imu frames fit a lidar frame
            while (package.lidar_begin_time > get_time_sec(imu_next.header.stamp)) {
                imu_last = imu_next;
                imu_next = *(imu_buffer.front());
                imu_buffer.pop_front();
            }

            // load gravity
            if (is_start_in_aggressive_motion) {
                state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                state_out.acc << VEC_FROM_ARRAY(gravity_init);
                state_out.acc *= -1;
            } else {
                state_in.gravity = -1 * imu_process->mean_acc * G_m_s2 / acc_norm;
                state_out.gravity = -1 * imu_process->mean_acc * G_m_s2 / acc_norm;
                state_out.acc = imu_process->mean_acc * G_m_s2 / acc_norm;
            }

            // load gravity align (optional)
            if (gravity_align) {
                Eigen::Matrix3d rot_init;
                imu_process->gravity_ << VEC_FROM_ARRAY(gravity);
                imu_process->set_init(state_in.gravity, rot_init);

                state_in.gravity = state_out.gravity = imu_process->gravity_;
                state_in.rot = state_out.rot = rot_init;
                state_in.rot.normalize();
                state_out.rot.normalize();
                state_out.acc = -rot_init.transpose() * state_out.gravity;
            }

            // load kalman filter param
            kf_input.change_x(state_in);
            kf_output.change_x(state_out);

            // sure, you can also run without imu
        } else if (!imu_process->gravity_align_ && !imu_en) {
            state_in.gravity << VEC_FROM_ARRAY(gravity_init);
            state_out.gravity << VEC_FROM_ARRAY(gravity_init);
            state_out.acc << VEC_FROM_ARRAY(gravity_init);
            state_out.acc *= -1;
        }

        // segment map in lidar fov
        laser_map_fov_segment();

        time_1 = omp_get_wtime();

        // down sample the feature points in a scan
        // if you want to make pc crash and memory occupied, you can try not to down sample
        if (space_down_sample) {
            down_sample_voxel_grid.setInputCloud(point_cloud_undistorted);
            down_sample_voxel_grid.filter(*cloud_body_link);

            sort(cloud_body_link->points.begin(), cloud_body_link->points.end(), time_list);

        } else {
            cloud_body_link = package.lidar;
            sort(cloud_body_link->points.begin(), cloud_body_link->points.end(), time_list);
        }

        time_seq = time_compressing<int>(cloud_body_link);
        cloud_sample_down_size = cloud_body_link->points.size();

        // initialize the map ikd tree
        if (!flag_map_initialized) {

            if (ikd_tree.Root_Node == nullptr) {
                ikd_tree.set_down_sample_param(filter_size_map_min);
            }

            cloud_world_link->resize(cloud_sample_down_size);

            for (int i = 0; i < cloud_sample_down_size; i++) {
                transform_body_to_world(
                    &(cloud_body_link->points[i]), &(cloud_world_link->points[i]));
            }

            for (size_t i = 0; i < cloud_world_link->size(); i++) {
                init_feats_world->points.emplace_back(cloud_world_link->points[i]);
            }

            if (init_feats_world->size() < init_map_size)
                continue;

            ikd_tree.Build(init_feats_world->points);

            flag_map_initialized = true;

            publish_init_ikd_tree(laser_map_publisher);

            continue;
        }

        // ICP and Kalman filter update
        normvec->resize(cloud_sample_down_size);
        cloud_world_link->resize(cloud_sample_down_size);

        Nearest_Points.resize(cloud_sample_down_size);

        time_2 = omp_get_wtime();

        // iterated state estimation
        crossmat_list.reserve(cloud_sample_down_size);
        pbody_list.reserve(cloud_sample_down_size);

        for (size_t i = 0; i < cloud_body_link->size(); i++) {

            auto point_this = Vector3d(
                cloud_body_link->points[i].x, cloud_body_link->points[i].y,
                cloud_body_link->points[i].z);

            pbody_list[i] = point_this;

            if (extrinsic_est_en) {
                if (!use_imu_as_input) {
                    point_this = kf_output.x_.offset_R_L_I.normalized() * point_this
                        + kf_output.x_.offset_T_L_I;
                } else {
                    point_this = kf_input.x_.offset_R_L_I.normalized() * point_this
                        + kf_input.x_.offset_T_L_I;
                }
            } else {
                point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
            }

            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);
            crossmat_list[i] = point_crossmat;
        }

        if (!use_imu_as_input) {

            bool imu_upda_cov = false;
            effect_feat_num = 0;

            // point by point update
            double pcl_begin_time = package.lidar_begin_time;

            idx = -1;

            for (k = 0; k < time_seq.size(); k++) {
                PointType& point_body = cloud_body_link->points[idx + time_seq[k]];

                time_current = point_body.curvature / 1000.0 + pcl_begin_time;

                if (is_first_frame) {
                    if (imu_en) {
                        while (time_current > get_time_sec(imu_next.header.stamp)) {
                            imu_last = imu_next;
                            imu_next = *(imu_buffer.front());
                            imu_buffer.pop_front();
                        }

                        angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                            imu_last.angular_velocity.z;
                        acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                            imu_last.linear_acceleration.z;
                    }
                    is_first_frame = false;
                    imu_upda_cov = true;
                    time_update_last = time_current;
                    time_predict_last_const = time_current;
                }
                if (imu_en) {
                    bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                    while (imu_comes) {
                        imu_upda_cov = true;
                        angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                            imu_next.angular_velocity.z;
                        acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                            imu_next.linear_acceleration.z;

                        /*** covariance update ***/
                        imu_last = imu_next;
                        imu_next = *(imu_buffer.front());
                        imu_buffer.pop_front();
                        double dt = get_time_sec(imu_last.header.stamp) - time_predict_last_const;
                        kf_output.predict(dt, Q_output, input_in, true, false);
                        time_predict_last_const = get_time_sec(imu_last.header.stamp); // big problem
                        imu_comes = time_current > get_time_sec(imu_next.header.stamp);

                        double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;

                        if (dt_cov > 0.0) {
                            time_update_last = get_time_sec(imu_last.header.stamp);
                            double propag_imu_start = omp_get_wtime();

                            kf_output.predict(dt_cov, Q_output, input_in, false, true);

                            propag_time += omp_get_wtime() - propag_imu_start;
                            double solve_imu_start = omp_get_wtime();
                            kf_output.update_iterated_dyn_share_IMU();
                            solve_time += omp_get_wtime() - solve_imu_start;
                        }
                    }
                }

                double dt = time_current - time_predict_last_const;
                double propag_state_start = omp_get_wtime();

                if (!prop_at_freq_of_imu) {
                    double dt_cov = time_current - time_update_last;
                    if (dt_cov > 0.0) {
                        kf_output.predict(dt_cov, Q_output, input_in, false, true);
                        time_update_last = time_current;
                    }
                }

                kf_output.predict(dt, Q_output, input_in, true, false);
                propag_time += omp_get_wtime() - propag_state_start;
                time_predict_last_const = time_current;

                double t_update_start = omp_get_wtime();

                if (cloud_sample_down_size < 1) {
                    RCLCPP_WARN(logger, "No point, skip this scan!\n");
                    idx += time_seq[k];
                    continue;
                }

                if (!kf_output.update_iterated_dyn_share_modified()) {
                    idx = idx + time_seq[k];
                    continue;
                }

                if (prop_at_freq_of_imu) {
                    double dt_cov = time_current - time_update_last;

                    if (!imu_en && (dt_cov >= imu_time_inte)) {
                        double propag_cov_start = omp_get_wtime();
                        kf_output.predict(dt_cov, Q_output, input_in, false, true);
                        imu_upda_cov = false;
                        time_update_last = time_current;
                        propag_time += omp_get_wtime() - propag_cov_start;
                    }
                }

                time_solve_start = omp_get_wtime();

                if (publish_odometry_without_down_sample) {
                    /******* Publish odometry *******/

                    publish_odometry(odometry_publisher, tf_broadcaster);
                    if (runtime_pos_log) {
                        state_out = kf_output.x_;
                        euler_cur = SO3ToEuler(state_out.rot);
                        fout_out << setw(20) << package.lidar_begin_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << state_out.pos.transpose() << " "
                                 << state_out.vel.transpose() << " " << state_out.omg.transpose()
                                 << " " << state_out.acc.transpose() << " "
                                 << state_out.gravity.transpose() << " " << state_out.bg.transpose()
                                 << " " << state_out.ba.transpose() << " "
                                 << point_cloud_undistorted->points.size() << std::endl;
                    }
                }

                for (int j = 0; j < time_seq[k]; j++) {
                    PointType& point_body_j = cloud_body_link->points[idx + j + 1];
                    PointType& point_world_j = cloud_world_link->points[idx + j + 1];
                    transform_body_to_world(&point_body_j, &point_world_j);
                }

                solve_time += omp_get_wtime() - time_solve_start;

                update_time += omp_get_wtime() - t_update_start;
                idx += time_seq[k];
            }
        } else {
            bool imu_prop_cov = false;
            effect_feat_num = 0;

            double pcl_beg_time = package.lidar_begin_time;
            idx = -1;

            for (k = 0; k < time_seq.size(); k++) {
                PointType& point_body = cloud_body_link->points[idx + time_seq[k]];
                time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                if (is_first_frame) {
                    while (time_current > get_time_sec(imu_next.header.stamp)) {
                        imu_last = imu_next;
                        imu_next = *(imu_buffer.front());
                        imu_buffer.pop_front();
                    }

                    imu_prop_cov = true;

                    is_first_frame = false;
                    time_last = time_current;
                    time_update_last = time_current;

                    input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                        imu_last.angular_velocity.z;

                    input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                        imu_last.linear_acceleration.z;

                    input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                }

                while (time_current > get_time_sec(imu_next.header.stamp)) {

                    imu_last = imu_next;
                    imu_next = *(imu_buffer.front());
                    imu_buffer.pop_front();
                    input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                        imu_last.angular_velocity.z;
                    input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                        imu_last.linear_acceleration.z;

                    input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                    double dt = get_time_sec(imu_last.header.stamp) - time_last;

                    double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
                    if (dt_cov > 0.0) {
                        kf_input.predict(dt_cov, Q_input, input_in, false, true);
                        time_update_last = get_time_sec(imu_last.header.stamp); // time_current;
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false);
                    time_last = get_time_sec(imu_last.header.stamp);
                    imu_prop_cov = true;
                }

                double dt = time_current - time_last;
                time_last = time_current;
                double propag_start = omp_get_wtime();

                if (!prop_at_freq_of_imu) {
                    double dt_cov = time_current - time_update_last;
                    if (dt_cov > 0.0) {
                        kf_input.predict(dt_cov, Q_input, input_in, false, true);
                        time_update_last = time_current;
                    }
                }
                kf_input.predict(dt, Q_input, input_in, true, false);

                propag_time += omp_get_wtime() - propag_start;

                double t_update_start = omp_get_wtime();

                if (cloud_sample_down_size < 1) {
                    RCLCPP_WARN(logger, "No point, skip this scan!\n");

                    idx += time_seq[k];
                    continue;
                }
                if (!kf_input.update_iterated_dyn_share_modified()) {
                    idx = idx + time_seq[k];
                    continue;
                }

                time_solve_start = omp_get_wtime();

                if (publish_odometry_without_down_sample) {
                    /******* Publish odometry *******/

                    publish_odometry(odometry_publisher, tf_broadcaster);

                    if (runtime_pos_log) {
                        state_in = kf_input.x_;
                        euler_cur = SO3ToEuler(state_in.rot);
                        fout_out << setw(20) << package.lidar_begin_time - first_lidar_time << " "
                                 << euler_cur.transpose() << " " << state_in.pos.transpose() << " "
                                 << state_in.vel.transpose() << " " << state_in.bg.transpose()
                                 << " " << state_in.ba.transpose() << " "
                                 << state_in.gravity.transpose() << " "
                                 << point_cloud_undistorted->points.size() << std::endl;
                    }
                }

                for (int j = 0; j < time_seq[k]; j++) {
                    PointType& point_body_j = cloud_body_link->points[idx + j + 1];
                    PointType& point_world_j = cloud_world_link->points[idx + j + 1];

                    transform_body_to_world(&point_body_j, &point_world_j);
                }

                solve_time += omp_get_wtime() - time_solve_start;

                update_time += omp_get_wtime() - t_update_start;
                idx = idx + time_seq[k];
            }
        }

        // publish the odometry in down sampling
        if (!publish_odometry_without_down_sample) {
            publish_odometry(odometry_publisher, tf_broadcaster);
        }

        // add the feature points to map ikd tree
        time_3 = omp_get_wtime();

        if (cloud_sample_down_size > 4) {
            increase_map();
        }

        if (pcd_save_en)
            *pcl_wait_save += *cloud_world_link;

        time_5 = omp_get_wtime();

        // publish
        if (path_en)
            publish_path(path_publisher);

        if (scan_publish_enable)
            publish_frame_world(cloud_registered_world_publisher, cloud_world_link);

        if (scan_publish_enable && scan_body_pub_en)
            publish_frame_body(cloud_registered_body_publisher);

        /*** Debug variables Logging ***/
        if (runtime_pos_log) {
            frame_num++;
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (time_5 - time_0) / frame_num;
            {
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num;
            }
            aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
            aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
            aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;

            log_time_1[time_log_counter] = package.lidar_begin_time;
            log_time_0[time_log_counter] = time_5 - time_0;
            log_time_2[time_log_counter] = point_cloud_undistorted->points.size();
            log_time_3[time_log_counter] = aver_time_consu;

            time_log_counter++;

            printf(
                "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave "
                "solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f "
                "propogate: %0.6f \n",
                time_1 - time_0, aver_time_match, aver_time_solve, time_3 - time_1, time_5 - time_3,
                aver_time_consu, aver_time_icp, aver_time_propag);

            if (!publish_odometry_without_down_sample) {
                if (!use_imu_as_input) {
                    state_out = kf_output.x_;
                    euler_cur = SO3ToEuler(state_out.rot);
                    fout_out << setw(20) << package.lidar_begin_time - first_lidar_time << " "
                             << euler_cur.transpose() << " " << state_out.pos.transpose() << " "
                             << state_out.vel.transpose() << " " << state_out.omg.transpose() << " "
                             << state_out.acc.transpose() << " " << state_out.gravity.transpose()
                             << " " << state_out.bg.transpose() << " " << state_out.ba.transpose()
                             << " " << point_cloud_undistorted->points.size() << std::endl;
                } else {
                    state_in = kf_input.x_;
                    euler_cur = SO3ToEuler(state_in.rot);
                    fout_out << setw(20) << package.lidar_begin_time - first_lidar_time << " "
                             << euler_cur.transpose() << " " << state_in.pos.transpose() << " "
                             << state_in.vel.transpose() << " " << state_in.bg.transpose() << " "
                             << state_in.ba.transpose() << " " << state_in.gravity.transpose()
                             << " " << point_cloud_undistorted->points.size() << std::endl;
                }
            }
            dump_lio_state_to_log(log_file);

        } // if (runtime_pos_log) end

        rate.sleep();

    } // while (rclcpp::ok()) end

    // save map
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        auto directory = root_dir + "PCD/" + "scans.pcd";
        pcl::PCDWriter().writeBinary(directory, *pcl_wait_save);

        RCLCPP_INFO(logger, "save pcd file in path: %s", directory.c_str());
    }

    fout_out.close();
    fout_imu_pbp.close();

    return 0;
}
