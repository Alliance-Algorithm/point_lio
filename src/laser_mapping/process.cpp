#include "laser_mapping.hpp"

constexpr float MOV_THRESHOLD = 1.5f;

auto mutex_buffer = std::mutex();
auto condition_variable_buffer = std::condition_variable();

int frame_count = 0;
bool lidar_pushed = false;

auto ptr_con = std::make_shared<PointCloudXYZI>();
auto feats_down_body_space = std::make_shared<PointCloudXYZI>();

auto lidar_buffer = deque<PointCloudXYZI::Ptr>();
auto time_buffer = deque<double>();
auto cub_needrm = vector<BoxPointType>();

void signal_handle(int signal)
{
    flag_exit = true;
    RCLCPP_WARN(logger, "catch sig %d", signal);
    condition_variable_buffer.notify_all();
}

void dump_lio_state_to_log(FILE* fp)
{
    V3D rot_ang;
    if (!use_imu_as_input) {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    } else {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }

    fprintf(fp, "%lf ", package.lidar_begin_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2)); // Angle
    if (use_imu_as_input) {
        fprintf(
            fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2)); // Pos
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // omega
        fprintf(
            fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2)); // Vel
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // Acc
        fprintf(
            fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2)); // Bias_g
        fprintf(
            fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2)); // Bias_a
        fprintf(
            fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1),
            kf_input.x_.gravity(2)); // Bias_a
    } else {
        fprintf(
            fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1),
            kf_output.x_.pos(2)); // Pos
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // omega
        fprintf(
            fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1),
            kf_output.x_.vel(2)); // Vel
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // Acc
        fprintf(
            fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1),
            kf_output.x_.bg(2)); // Bias_g
        fprintf(
            fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1),
            kf_output.x_.ba(2)); // Bias_a
        fprintf(
            fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
            kf_output.x_.gravity(2)); // Bias_a
    }
    fprintf(fp, "\r\n");
    fflush(fp);
}

int points_cache_size = 0;

void points_cache_collect() // seems for debug
{
    PointVector points_history;
    ikd_tree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;

void laser_map_fov_segment()
{
    cub_needrm.shrink_to_fit();

    V3D pos_LiD;
    if (use_imu_as_input) {
        pos_LiD = kf_input.x_.pos + kf_input.x_.rot.normalized() * Lidar_T_wrt_IMU;
    } else {
        pos_LiD = kf_output.x_.pos + kf_output.x_.rot.normalized() * Lidar_T_wrt_IMU;
    }
    if (!Localmap_Initialized) {
        for (int i = 0; i < 3; i++) {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE
            || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
        double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++) {
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    if (cub_needrm.size() > 0)
        int kdtree_delete_counter = ikd_tree.Delete_Point_Boxes(cub_needrm);
}

void pointcloud_subscription_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    mutex_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (get_time_sec(msg->header.stamp) < stamp_lidar_last) {
        RCLCPP_ERROR(logger, "lidar loop back, clear buffer");
        // lidar_buffer.shrink_to_fit();

        mutex_buffer.unlock();
        condition_variable_buffer.notify_all();
        return;
    }

    stamp_lidar_last = msg->header.stamp.sec;

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
    double time_div = get_time_sec(msg->header.stamp);
    p_pre->process(msg, ptr);
    if (cut_frame) {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {
            ptr_div->push_back(ptr->points[i]);
            // cout << "check time:" << ptr->points[i].curvature << endl;
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div
                > cut_frame_time_interval) {
                if (ptr_div->size() < 1)
                    continue;
                PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty()) {
            lidar_buffer.push_back(ptr_div);
            // ptr_div->clear();
            time_buffer.push_back(time_div);
        }
    } else if (con_frame) {
        if (frame_count == 0) {
            time_con = stamp_lidar_last; // get_time_sec(msg->header.stamp);
        }
        if (frame_count < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (stamp_lidar_last - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_count++;
        } else {
            PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_count = 0;
        }
    } else {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    log_time_11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mutex_buffer.unlock();
    condition_variable_buffer.notify_all();
}

void livox_subscription_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{

    mutex_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (get_time_sec(msg->header.stamp) < stamp_lidar_last) {
        RCLCPP_ERROR(logger, "lidar loop back, clear buffer");

        mutex_buffer.unlock();
        condition_variable_buffer.notify_all();
        return;
    }

    stamp_lidar_last = get_time_sec(msg->header.stamp);

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    double time_div = get_time_sec(msg->header.stamp);
    if (cut_frame) {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {
            ptr_div->push_back(ptr->points[i]);
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div
                > cut_frame_time_interval) {
                if (ptr_div->size() < 1)
                    continue;
                PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
                // cout << "ptr div num:" << ptr_div->size() << endl;
                *ptr_div_i = *ptr_div;
                // cout << "ptr div i num:" << ptr_div_i->size() << endl;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty()) {
            lidar_buffer.push_back(ptr_div);
            // ptr_div->clear();
            time_buffer.push_back(time_div);
        }
    } else if (con_frame) {
        if (frame_count == 0) {
            time_con = stamp_lidar_last; // get_time_sec(msg->header.stamp);
        }
        if (frame_count < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (stamp_lidar_last - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_count++;
        } else {
            PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_count = 0;
        }
    } else {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    log_time_11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mutex_buffer.unlock();
    condition_variable_buffer.notify_all();
}

void imu_subscription_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // publish_count++;

    msg->header.stamp = get_ros_time(get_time_sec(msg->header.stamp) - lag_between_imu_and_lidar);

    auto stamp = get_time_sec(msg->header.stamp);

    mutex_buffer.lock();

    if (stamp < stamp_imu_last) {
        RCLCPP_ERROR(logger, "imu loop back, clear deque");
        // imu_deque.shrink_to_fit();
        mutex_buffer.unlock();
        condition_variable_buffer.notify_all();
        return;
    }

    imu_buffer.emplace_back(msg);
    stamp_imu_last = stamp;
    mutex_buffer.unlock();
    condition_variable_buffer.notify_all();
}

bool unpack(CombinedPackage& package)
{
    // imu enable
    if (imu_en) {
        if (lidar_buffer.empty() || imu_buffer.empty()) {
            return false;
        }

        // push a pointcloud from lidar
        if (!lidar_pushed) {

            package.lidar = lidar_buffer.front();

            if (package.lidar->points.size() < 1) {
                cout << "lose lidar" << endl;
                lidar_buffer.pop_front();
                time_buffer.pop_front();
                return false;
            }

            package.lidar_begin_time = time_buffer.front();
            double end_time = package.lidar->points.back().curvature;

            for (auto pt : package.lidar->points) {
                if (pt.curvature > end_time) {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = package.lidar_begin_time + end_time / double(1000);

            package.lidar_end_time = lidar_end_time;
            lidar_pushed = true;
        }

        if (stamp_imu_last < lidar_end_time) {

            // RCLCPP_WARN(rclcpp::get_logger("laserMapping"), "lidar time error");

            return false;
        }

        /*** push imu data, and pop from imu buffer ***/
        if (imu_process->is_need_initialize_) {
            double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
            package.imu.shrink_to_fit();
            while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
                imu_time = get_time_sec(imu_buffer.front()->header.stamp);
                if (imu_time > lidar_end_time)
                    break;
                package.imu.emplace_back(imu_buffer.front());
                imu_last = imu_next;
                imu_last_ptr = imu_buffer.front();
                imu_next = *(imu_buffer.front());
                imu_buffer.pop_front();
            }
        } else if (!flag_map_initialized) {
            double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
            package.imu.shrink_to_fit();
            package.imu.emplace_back(imu_last_ptr);

            while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
                imu_time = get_time_sec(imu_buffer.front()->header.stamp);
                if (imu_time > lidar_end_time)
                    break;
                package.imu.emplace_back(imu_buffer.front());
                imu_last = imu_next;
                imu_last_ptr = imu_buffer.front();
                imu_next = *(imu_buffer.front());
                imu_buffer.pop_front();
            }
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;

        return true;

        // imu disable
    } else {

        if (!lidar_buffer.empty()) {
            package.lidar = lidar_buffer.front();
            package.lidar_begin_time = time_buffer.front();
            time_buffer.pop_front();
            lidar_buffer.pop_front();

            if (package.lidar->points.size() < 1) {
                cout << "lose lidar" << std::endl;
                return false;
            }

            double end_time = package.lidar->points.back().curvature;

            for (auto pt : package.lidar->points) {
                if (pt.curvature > end_time) {
                    end_time = pt.curvature;
                }
            }

            lidar_end_time = package.lidar_begin_time + end_time / double(1000);
            package.lidar_end_time = lidar_end_time;
            return true;
        }

        return false;
    }
}

void increase_map()
{
    PointVector point_to_add;
    PointVector point_no_down_sample_to_add;

    point_to_add.reserve(cloud_sample_down_size);
    point_no_down_sample_to_add.reserve(cloud_sample_down_size);

    for (int i = 0; i < cloud_sample_down_size; i++) {
        if (!Nearest_Points[i].empty()) {

            bool need_add = true;

            auto mid_point = PointType(
                floor(cloud_world_link->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min,
                floor(cloud_world_link->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min,
                floor(cloud_world_link->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min);

            /* If the nearest points is definitely outside the down sample box */
            if (0
                || fabs(Nearest_Points[i][0].x - mid_point.x) > 1.732 * filter_size_map_min
                || fabs(Nearest_Points[i][0].y - mid_point.y) > 1.732 * filter_size_map_min
                || fabs(Nearest_Points[i][0].z - mid_point.z) > 1.732 * filter_size_map_min) {

                point_no_down_sample_to_add.emplace_back(cloud_world_link->points[i]);

                continue;
            }

            /* Check if there is a point already in the down sample box */
            float dist = calc_dist<float>(cloud_world_link->points[i], mid_point);

            for (int readd_i = 0; readd_i < Nearest_Points[i].size(); readd_i++) {
                /* Those points which are outside the down sample box should not be considered. */
                if (1
                    && fabs(Nearest_Points[i][readd_i].x - mid_point.x) < 0.5 * filter_size_map_min
                    && fabs(Nearest_Points[i][readd_i].y - mid_point.y) < 0.5 * filter_size_map_min
                    && fabs(Nearest_Points[i][readd_i].z - mid_point.z) < 0.5 * filter_size_map_min) {

                    need_add = false;
                    break;
                }
            }

            if (need_add)
                point_to_add.emplace_back(cloud_world_link->points[i]);
        } else {
            point_no_down_sample_to_add.emplace_back(cloud_world_link->points[i]);
        }
    }

    ikd_tree.Add_Points(point_to_add, true);
    ikd_tree.Add_Points(point_no_down_sample_to_add, false);
}

void pre_increase_map(const PointCloudXYZI::Ptr& cloud)
{
    auto point_to_add = PointVector();

    point_to_add.reserve(cloud->size());

    for (auto&& point : *cloud) {
        point_to_add.emplace_back(point);
    }

    ikd_tree.Add_Points(point_to_add, true);
}