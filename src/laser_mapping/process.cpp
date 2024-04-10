#include "laser_mapping.hpp"

void LaserMapping::main_process_timer_callback()
{
    if (flag_exit) {
        main_process_timer_->cancel();
    }

    if (sync_packages(Measures)) {

        if (is_first_scan) {
            first_lidar_time = Measures.lidar_beg_time;

            RCLCPP_INFO(this->get_logger(),
                "First lidar time is %lf", first_lidar_time);

            is_first_scan = false;
        }

        if (flag_reset) {
            RCLCPP_WARN(this->get_logger(),
                "reset when rosbag play back");

            imu_process->Reset();
            flag_reset = false;
            return;
        }

        double t0, t1, t2, t3, t4, t5, match_start, solve_start;

        match_time = 0;
        solve_time = 0;
        propag_time = 0;
        update_time = 0;

        t0 = omp_get_wtime();

        imu_process->Process(Measures, feats_undistort);

        if (imu_process->imu_need_init_) {
            DEBUG("imu need init");
            return;
        }

        if (imu_en) {

            if (!imu_process->gravity_align_) {
                DEBUG("gravity align");

                while (Measures.lidar_beg_time > rclcpp::Time(imu_next.header.stamp).seconds()) {
                    imu_last = imu_next;
                    imu_next = *(imu_deque.front());
                    imu_deque.pop_front();
                }

                if (non_station_start) {
                    state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc *= -1;

                } else {
                    state_in.gravity = -1 * imu_process->mean_acc * G_m_s2 / acc_norm;
                    state_out.gravity = -1 * imu_process->mean_acc * G_m_s2 / acc_norm;
                    state_out.acc = imu_process->mean_acc * G_m_s2 / acc_norm;
                }

                if (gravity_align) {
                    Eigen::Matrix3d rot_init;
                    imu_process->gravity_ << VEC_FROM_ARRAY(gravity);
                    imu_process->Set_init(state_in.gravity, rot_init);
                    state_in.gravity = imu_process->gravity_;
                    state_out.gravity = imu_process->gravity_;
                    state_in.rot = rot_init;
                    state_out.rot = rot_init;
                    state_out.acc = -rot_init.transpose() * state_out.gravity;
                }

                kf_input.change_x(state_in);
                kf_output.change_x(state_out);

                imu_process->gravity_align_ = true;
            }

        } else {

            if (!imu_process->gravity_align_) {
                DEBUG("gravity align");

                state_in.gravity << VEC_FROM_ARRAY(gravity_init);

                if (gravity_align) {
                    Eigen::Matrix3d rot_init;
                    imu_process->gravity_ << VEC_FROM_ARRAY(gravity);
                    imu_process->Set_init(state_in.gravity, rot_init);
                    state_out.gravity = imu_process->gravity_;
                    state_out.rot = rot_init;
                    // state_in.rot.normalize();
                    // state_out.rot.normalize();
                    state_out.acc = -rot_init.transpose() * state_out.gravity;
                } else {
                    state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc << VEC_FROM_ARRAY(gravity_init);
                    state_out.acc *= -1;
                }

                kf_output.change_x(state_out);
                imu_process->gravity_align_ = true;
            }
        }

        /*** Segment the map in lidar FOV ***/
        lasermap_fov_segment();

        /*** downsample the feature points in a scan ***/
        t1 = omp_get_wtime();

        if (space_down_sample) {
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
        } else {
            feats_down_body = Measures.lidar;
            sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
        }

        time_seq = time_compressing<int>(feats_down_body);
        feats_down_size = feats_down_body->points.size();

        /*** initialize the map kdtree ***/
        if (!init_map) {
            DEBUG("map is initializing");

            if (ikdtree.Root_Node == nullptr) //
            {
                ikdtree.set_downsample_param(filter_size_map_min);
            }

            feats_down_world->resize(feats_down_size);

            for (int i = 0; i < feats_down_size; i++) {
                pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
            }

            for (size_t i = 0; i < feats_down_world->size(); i++) {
                init_feats_world->points.emplace_back(feats_down_world->points[i]);
            }

            if (init_feats_world->size() < init_map_size)
                return;

            ikdtree.Build(init_feats_world->points);
            init_map = true;
            publish_init_kdtree(cloud_map_publisher_);

            return;
        }

        // ICP and Kalman filter update
        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);

        Nearest_Points.resize(feats_down_size);

        t2 = omp_get_wtime();

        // iterated state estimation
        crossmat_list.reserve(feats_down_size);
        pbody_list.reserve(feats_down_size);

        for (size_t i = 0; i < feats_down_body->size(); i++) {

            V3D point_this(
                feats_down_body->points[i].x,
                feats_down_body->points[i].y,
                feats_down_body->points[i].z);

            pbody_list[i] = point_this;

            if (extrinsic_est_en) {

                if (!use_imu_as_input)
                    point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
                else
                    point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;

            } else {
                point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
            }

            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);
            crossmat_list[i] = point_crossmat;
        }

        if (!use_imu_as_input) {
            // bool imu_upda_cov = false;
            effct_feat_num = 0;

            // point by point update
            double pcl_beg_time = Measures.lidar_beg_time;
            idx = -1;

            for (k = 0; k < time_seq.size(); k++) {
                PointType& point_body = feats_down_body->points[idx + time_seq[k]];

                time_current = point_body.curvature / 1000.0 + pcl_beg_time;

                if (is_first_frame) {

                    DEBUG("the first frame is initializing");

                    if (imu_en) {
                        while (time_current > rclcpp::Time(imu_next.header.stamp).seconds()) {
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            // imu_deque.pop();
                        }

                        angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                        acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                    }

                    is_first_frame = false;
                    // imu_upda_cov = true;

                    time_update_last = time_current;
                    time_predict_last_const = time_current;

                } // end if(is_first_frame)

                if (imu_en) {

                    bool imu_comes;

                    imu_comes = time_current > rclcpp::Time(imu_next.header.stamp).seconds();

                    while (imu_comes) {

                        // imu_upda_cov = true;
                        angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                        acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                        /*** covariance update ***/
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();

                        double dt = rclcpp::Time(imu_next.header.stamp).seconds() - time_predict_last_const;

                        kf_output.predict(dt, Q_output, input_in,
                            true, false);

                        time_predict_last_const = rclcpp::Time(imu_next.header.stamp).seconds(); // big problem
                        imu_comes = time_current > rclcpp::Time(imu_next.header.stamp).seconds();

                        double dt_cov = rclcpp::Time(imu_next.header.stamp).seconds() - time_update_last;

                        if (dt_cov > 0.0) {

                            time_update_last = rclcpp::Time(imu_next.header.stamp).seconds();
                            double propag_imu_start = omp_get_wtime();

                            kf_output.predict(dt_cov, Q_output, input_in, false, true);

                            propag_time += omp_get_wtime() - propag_imu_start;
                            double solve_imu_start = omp_get_wtime();
                            kf_output.update_iterated_dyn_share_IMU();
                            solve_time += omp_get_wtime() - solve_imu_start;
                        }
                    } // handle imu coming
                } // end if(imu_en)

                double dt = time_current - time_predict_last_const;
                double propag_state_start = omp_get_wtime();

                if (!prop_at_freq_of_imu) {
                    double dt_cov = time_current - time_update_last;
                    if (dt_cov > 0.0) {

                        kf_output.predict(dt_cov, Q_output, input_in,
                            false, true);

                        time_update_last = time_current;
                    }
                }

                kf_output.predict(dt, Q_output, input_in,
                    true, false);

                propag_time += omp_get_wtime() - propag_state_start;
                time_predict_last_const = time_current;

                double t_update_start = omp_get_wtime();

                if (feats_down_size < 1) {

                    RCLCPP_WARN(this->get_logger(),
                        "No point, skip this scan!\n");

                    idx += time_seq[k];
                    return;
                }

                if (!kf_output.update_iterated_dyn_share_modified()) {

                    RCLCPP_WARN(this->get_logger(),
                        "not update iterated dyn share modified");

                    idx = idx + time_seq[k];

                    return;
                }

                solve_start = omp_get_wtime();

                if (publish_odometry_without_downsample) {

                    // Publish odometry
                    publish_odometry(after_mapped_to_init_publisher_);

                    if (runtime_pos_log) {

                        state_out = kf_output.x_;
                        euler_cur = SO3ToEuler(state_out.rot);

                        fout_out
                            << setw(20) << Measures.lidar_beg_time - first_lidar_time
                            << " " << euler_cur.transpose()
                            << " " << state_out.pos.transpose()
                            << " " << state_out.vel.transpose()
                            << " " << state_out.omg.transpose()
                            << " " << state_out.acc.transpose()
                            << " " << state_out.gravity.transpose()
                            << " " << state_out.bg.transpose()
                            << " " << state_out.ba.transpose()
                            << " " << feats_undistort->points.size() << endl;
                    }
                }

                for (int j = 0; j < time_seq[k]; j++) {
                    PointType& point_body_j = feats_down_body->points[idx + j + 1];
                    PointType& point_world_j = feats_down_world->points[idx + j + 1];
                    pointBodyToWorld(&point_body_j, &point_world_j);
                }

                solve_time += omp_get_wtime() - solve_start;

                update_time += omp_get_wtime() - t_update_start;
                idx += time_seq[k];
            }

        } else {
            bool imu_prop_cov = false;
            effct_feat_num = 0;

            double pcl_beg_time = Measures.lidar_beg_time;
            idx = -1;
            for (k = 0; k < time_seq.size(); k++) {
                PointType& point_body = feats_down_body->points[idx + time_seq[k]];
                time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                if (is_first_frame) {
                    while (time_current > rclcpp::Time(imu_next.header.stamp).seconds()) {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                    }
                    imu_prop_cov = true;

                    is_first_frame = false;
                    t_last = time_current;
                    time_update_last = time_current;
                    {
                        input_in.gyro << imu_last.angular_velocity.x,
                            imu_last.angular_velocity.y,
                            imu_last.angular_velocity.z;

                        input_in.acc << imu_last.linear_acceleration.x,
                            imu_last.linear_acceleration.y,
                            imu_last.linear_acceleration.z;

                        input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                    }
                }

                while (time_current > rclcpp::Time(imu_next.header.stamp).seconds()) {
                    imu_last = imu_next;
                    imu_next = *(imu_deque.front());
                    imu_deque.pop_front();
                    input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                    input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;

                    input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                    double dt = rclcpp::Time(imu_next.header.stamp).seconds() - t_last;

                    double dt_cov = rclcpp::Time(imu_next.header.stamp).seconds() - time_update_last;
                    if (dt_cov > 0.0) {
                        kf_input.predict(dt_cov, Q_input, input_in, false, true);
                        time_update_last = rclcpp::Time(imu_next.header.stamp).seconds();
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false);
                    t_last = rclcpp::Time(imu_next.header.stamp).seconds();
                    imu_prop_cov = true;
                }

                double dt = time_current - t_last;
                t_last = time_current;
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

                if (feats_down_size < 1) {
                    RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");

                    idx += time_seq[k];
                    return;
                }
                if (!kf_input.update_iterated_dyn_share_modified()) {
                    idx = idx + time_seq[k];
                    return;
                }

                solve_start = omp_get_wtime();

                // Publish odometry
                if (publish_odometry_without_downsample) {

                    publish_odometry(after_mapped_to_init_publisher_);

                    if (runtime_pos_log) {
                        state_in = kf_input.x_;
                        euler_cur = SO3ToEuler(state_in.rot);
                        fout_out
                            << setw(20) << Measures.lidar_beg_time - first_lidar_time
                            << " " << euler_cur.transpose()
                            << " " << state_in.pos.transpose()
                            << " " << state_in.vel.transpose()
                            << " " << state_in.bg.transpose()
                            << " " << state_in.ba.transpose()
                            << " " << state_in.gravity.transpose()
                            << " " << feats_undistort->points.size() << endl;
                    }
                }

                for (int j = 0; j < time_seq[k]; j++) {
                    PointType& point_body_j = feats_down_body->points[idx + j + 1];
                    PointType& point_world_j = feats_down_world->points[idx + j + 1];
                    pointBodyToWorld(&point_body_j, &point_world_j);
                }
                solve_time += omp_get_wtime() - solve_start;

                update_time += omp_get_wtime() - t_update_start;
                idx = idx + time_seq[k];
            }
        }

        DEBUG("publish time");

        // Publish odometry downsample
        if (!publish_odometry_without_downsample) {
            publish_odometry(after_mapped_to_init_publisher_);
        }

        // add the feature points to map kdtree
        t3 = omp_get_wtime();

        if (feats_down_size > 4) {
            map_incremental();
        }

        t5 = omp_get_wtime();

        // Publish points
        if (path_en)
            publish_path(path_publisher_);

        if (scan_pub_en || pcd_save_en)
            publish_frame_world(cloud_registered_publisher_);

        if (scan_pub_en && scan_body_pub_en)
            publish_frame_body(cloud_registered_body_publisher_);

        // Debug variables Logging
        if (runtime_pos_log) {
            frame_num++;
            aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
            {
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num;
            }
            aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
            aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
            aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
            T1[time_log_counter] = Measures.lidar_beg_time;
            s_plot[time_log_counter] = t5 - t0;
            s_plot2[time_log_counter] = feats_undistort->points.size();
            s_plot3[time_log_counter] = aver_time_consu;
            time_log_counter++;
            printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n", t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp, aver_time_propag);
            if (!publish_odometry_without_downsample) {
                if (!use_imu_as_input) {
                    state_out = kf_output.x_;
                    euler_cur = SO3ToEuler(state_out.rot);
                    fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_out.pos.transpose() << " " << state_out.vel.transpose()
                             << " " << state_out.omg.transpose() << " " << state_out.acc.transpose() << " " << state_out.gravity.transpose() << " " << state_out.bg.transpose() << " " << state_out.ba.transpose() << " " << feats_undistort->points.size() << endl;
                } else {
                    state_in = kf_input.x_;
                    euler_cur = SO3ToEuler(state_in.rot);
                    fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_in.pos.transpose() << " " << state_in.vel.transpose()
                             << " " << state_in.bg.transpose() << " " << state_in.ba.transpose() << " " << state_in.gravity.transpose() << " " << feats_undistort->points.size() << endl;
                }
            }
            dump_lio_state_to_log(fp);
        }
    }
}