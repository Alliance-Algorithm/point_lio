#include "laser_mapping.hpp"

LaserMapping::LaserMapping()
    : Node("laser_mapping",
          rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
{
    DEBUG("Entry the construct");

    read_param();
    DEBUG("Read param in construct");

    initialize();
    DEBUG("Initialized");

    log_precess_timer_
        = this->create_wall_timer(1s, []() -> void {});

    // Main process
    main_process_timer_
        = this->create_wall_timer(
            1ms,
            std::bind(&LaserMapping::main_process_timer_callback, this));

    // Subscription
    livox_lidar_subscription_
        = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lidar_topic, rclcpp::SensorDataQoS(),
            std::bind(&LaserMapping::livox_pcl_callback, this, std::placeholders::_1));

    imu_subscription_
        = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            std::bind(&LaserMapping::imu_callback, this, std::placeholders::_1));

    // Public
    cloud_registered_publisher_
        = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 20);

    cloud_registered_body_publisher_
        = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 20);

    cloud_effected_publisher_
        = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 20);

    cloud_map_publisher_
        = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_map", 20);

    after_mapped_to_init_publisher_
        = this->create_publisher<nav_msgs::msg::Odometry>("/after_mapped_to_init", 20);

    path_publisher_
        = this->create_publisher<nav_msgs::msg::Path>("/path", 20);

    planner_normal_publisher_
        = this->create_publisher<visualization_msgs::msg::Marker>("planner_normal", 20);
}

LaserMapping::~LaserMapping()
{
    fout_out.close();
    fout_imu_pbp.close();
    fclose(fp);
}

void LaserMapping::initialize()
{
    signal(SIGINT, [](int signal) -> void {
        flag_exit = true;
        std::cout << "catch signal %d" << signal << std::endl;
        signal_buffer.notify_all();
        rclcpp::shutdown();
    });

    DEBUG("entry the initialization in construct");

    path.header.stamp = rclcpp::Time(lidar_end_time);
    path.header.frame_id = "camera_init";

    memset(point_selected_surf, true, sizeof(point_selected_surf));

    downSizeFilterSurf.setLeafSize(
        filter_size_surf_min,
        filter_size_surf_min,
        filter_size_surf_min);
    downSizeFilterMap.setLeafSize(
        filter_size_map_min,
        filter_size_map_min,
        filter_size_map_min);

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

    imu_process->lidar_type
        = preprocess->lidar_type
        = lidar_type;

    imu_process->imu_en = imu_en;

    kf_input.init_dyn_share_modified(
        get_f_input,
        df_dx_input,
        h_model_input);
    kf_output.init_dyn_share_modified_2h(
        get_f_output,
        df_dx_output,
        h_model_output,
        h_model_IMU_output);

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

    Q_input = process_noise_cov_input();
    Q_output = process_noise_cov_output();

    DEBUG("Create lots of values in Initialization");

    fp = fopen(pos_log_dir.c_str(), "w");

    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"), ios::out);

    if (fout_out && fout_imu_pbp)
        cout << "~~~~" << ROOT_DIR << " file opened" << endl;
    else
        cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;

    DEBUG("Open some files in Initialization");
}

void LaserMapping::read_param()
{
    int test = 0;
    this->get_parameter<int>("launch.test", test);
    RCLCPP_INFO(this->get_logger(), "The test number is %d", test);

    // Get parameter
    // launch
    this->get_parameter<bool>("launch.prop_at_freq_of_imu", prop_at_freq_of_imu);
    this->get_parameter<bool>("launch.use_imu_as_input", use_imu_as_input);
    this->get_parameter<bool>("launch.check_satu", check_satu);
    this->get_parameter<int>("launch.init_map_size", init_map_size);
    this->get_parameter<bool>("launch.space_down_sample", space_down_sample);
    this->get_parameter<double>("launch.filter_size_surf", filter_size_surf_min);
    this->get_parameter<double>("launch.filter_size_map", filter_size_map_min);
    this->get_parameter<double>("launch.cube_side_length", cube_len);
    this->get_parameter<bool>("launch.runtime_pos_log_enable", runtime_pos_log);
    this->get_parameter<int>("launch.point_filter_num", preprocess->point_filter_num);
    // common
    this->get_parameter<std::string>("common.lid_topic", lidar_topic);
    this->get_parameter<std::string>("common.imu_topic", imu_topic);
    this->get_parameter<bool>("common.con_frame", con_frame);
    this->get_parameter<int>("common.con_frame_num", con_frame_num);
    this->get_parameter<bool>("common.cut_frame", cut_frame);
    this->get_parameter<double>("common.cut_frame_time_interval", cut_frame_time_interval);
    this->get_parameter<double>("common.time_lag_imu_to_lidar", time_lag_imu_to_lidar);
    // mapping
    this->get_parameter<float>("mapping.det_range", DET_RANGE);
    this->get_parameter<double>("mapping.fov_degree", fov_deg);
    this->get_parameter<bool>("mapping.imu_en", imu_en);
    this->get_parameter<bool>("mapping.start_in_aggressive_motion", non_station_start);
    this->get_parameter<bool>("mapping.extrinsic_est_en", extrinsic_est_en);
    this->get_parameter<double>("mapping.imu_time_inte", imu_time_inte);
    this->get_parameter<double>("mapping.lidar_meas_cov", laser_point_cov);
    this->get_parameter<double>("mapping.acc_cov_input", acc_cov_input);
    this->get_parameter<double>("mapping.vel_cov", vel_cov);
    this->get_parameter<double>("mapping.gyr_cov_input", gyr_cov_input);
    this->get_parameter<double>("mapping.gyr_cov_output", gyr_cov_output);
    this->get_parameter<double>("mapping.acc_cov_output", acc_cov_output);
    this->get_parameter<double>("mapping.b_gyr_cov", b_gyr_cov);
    this->get_parameter<double>("mapping.b_acc_cov", b_acc_cov);
    this->get_parameter<double>("mapping.imu_meas_acc_cov", imu_meas_acc_cov);
    this->get_parameter<double>("mapping.imu_meas_omg_cov", imu_meas_omg_cov);
    this->get_parameter<double>("mapping.match_s", match_s);
    this->get_parameter<bool>("mapping.gravity_align", gravity_align);
    this->get_parameter<std::vector<double>>("mapping.gravity", gravity);
    this->get_parameter<std::vector<double>>("mapping.gravity_init", gravity_init);
    this->get_parameter<std::vector<double>>("mapping.extrinsic_T", extrinT);
    this->get_parameter<std::vector<double>>("mapping.extrinsic_R", extrinR);
    this->get_parameter<double>("mapping.satu_acc", satu_acc);
    this->get_parameter<double>("mapping.satu_gyro", satu_gyro);
    this->get_parameter<double>("mapping.acc_norm", acc_norm);
    this->get_parameter<float>("mapping.plane_thr", plane_thr);
    // preprocess
    this->get_parameter<double>("preprocess.blind", preprocess->blind);
    this->get_parameter<int>("preprocess.lidar_type", lidar_type);
    this->get_parameter<int>("preprocess.scan_line", preprocess->N_SCANS);
    this->get_parameter<int>("preprocess.scan_rate", preprocess->SCAN_RATE);
    this->get_parameter<int>("preprocess.timestamp_unit", preprocess->time_unit);
    // odometry
    this->get_parameter<bool>("odometry.publish_odometry_without_downsample", publish_odometry_without_downsample);
    // publish
    this->get_parameter<bool>("publish.path_en", path_en);
    this->get_parameter<bool>("publish.scan_publish_en", scan_pub_en);
    this->get_parameter<bool>("publish.scan_bodyframe_pub_en", scan_body_pub_en);
    // pcd save
    this->get_parameter<bool>("pcd_save.pcd_save_en", pcd_save_en);
    this->get_parameter<int>("pcd_save.interval", pcd_save_interval);

    DEBUG("End read param");
}

void LaserMapping::dump_lio_state_to_log(FILE* fp)
{
    V3D rot_ang;
    if (!use_imu_as_input) {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    } else {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }

    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2)); // Angle

    if (use_imu_as_input) {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2)); // Pos
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // omega
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2)); // Vel
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // Acc
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2)); // Bias_g
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2)); // Bias_a
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1),
            kf_input.x_.gravity(2)); // Bias_a
    } else {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2)); // Pos
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // omega
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2)); // Vel
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0); // Acc
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2)); // Bias_g
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2)); // Bias_a
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1), kf_output.x_.gravity(2)); // Bias_a
    }

    fprintf(fp, "\r\n");
    fflush(fp);
}

void LaserMapping::pointBodyLidarToIMU(PointType const* const pi, PointType* const po)
{
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

void LaserMapping::points_cache_collect() // seems for debug
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}

void LaserMapping::lasermap_fov_segment()
{
    cub_needrm.shrink_to_fit();

    V3D pos_LiD;
    if (use_imu_as_input) {
        pos_LiD = kf_input.x_.pos + kf_input.x_.rot * Lidar_T_wrt_IMU;
    } else {
        pos_LiD = kf_output.x_.pos + kf_output.x_.rot * Lidar_T_wrt_IMU;
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
        int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}

bool LaserMapping::sync_packages(MeasureGroup& meas)
{
    if (lidar_buffer.empty() || imu_deque.empty()) {
        return false;
    }

    // imu disable
    if (!imu_en) {
        if (!lidar_buffer.empty()) {

            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();

            time_buffer.pop_front();
            lidar_buffer.pop_front();

            if (meas.lidar->points.size() < 1) {

                RCLCPP_INFO(this->get_logger(), "loze lidar");

                return false;
            }

            double end_time = meas.lidar->points.back().curvature;

            for (auto pt : meas.lidar->points) {
                if (pt.curvature > end_time) {
                    end_time = pt.curvature;
                }
            }

            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;

            return true;
        }

        return false;
    }

    // push a lidar scan
    if (!lidar_pushed) {

        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();

        if (meas.lidar->points.size() < 1) {

            RCLCPP_INFO(this->get_logger(), "loze lidar");

            lidar_buffer.pop_front();
            time_buffer.pop_front();

            return false;
        }

        double end_time = meas.lidar->points.back().curvature;

        for (auto pt : meas.lidar->points) {
            if (pt.curvature > end_time) {
                end_time = pt.curvature;
            }
        }

        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);

        meas.lidar_last_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        DEBUG("error lidar timestamp");
        return false;
    }

    // push imu data, and pop from imu buffer
    if (imu_process->imu_need_init_) {

        RCLCPP_INFO(this->get_logger(), "imu is initializing");

        double imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds();
        meas.imu.shrink_to_fit();

        while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {
            imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds();
            if (imu_time > lidar_end_time)
                break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }

    } else if (!init_map) {

        RCLCPP_INFO(this->get_logger(), "map is initializing");

        double imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds();

        meas.imu.shrink_to_fit();
        meas.imu.emplace_back(imu_last_ptr);

        while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {
            imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds();
            if (imu_time > lidar_end_time)
                break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();

    lidar_pushed = false;

    return true;
}

void LaserMapping::map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);

    for (int i = 0; i < feats_down_size; i++) {
        if (!Nearest_Points[i].empty()) {
            const PointVector& points_near = Nearest_Points[i];
            bool need_add = true;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min
                + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min
                + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min
                + 0.5 * filter_size_map_min;
            /* If the nearest points is definitely outside the downsample box */
            if (fabs(points_near[0].x - mid_point.x) > 0.866 * filter_size_map_min
                || fabs(points_near[0].y - mid_point.y) > 0.866 * filter_size_map_min
                || fabs(points_near[0].z - mid_point.z) > 0.866 * filter_size_map_min) {
                PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
                return;
            }
            /* Check if there is a point already in the downsample box */
            float dist = calc_dist<float>(feats_down_world->points[i], mid_point);
            for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
                /* Those points which are outside the downsample box should not be considered. */
                if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min
                    && fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min
                    && fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min) {
                    need_add = false;
                    break;
                }
            }
            if (need_add)
                PointToAdd.emplace_back(feats_down_world->points[i]);
        } else {
            // PointToAdd.emplace_back(feats_down_world->points[i]);
            PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
        }
    }
    int add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
}

void LaserMapping::publish_init_kdtree(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFullRes)
{
    sensor_msgs::msg::PointCloud2 laser_cloud_msg;

    int size_init_ikdtree = ikdtree.size();

    auto laserCloudInit = PointCloudXYZI::Ptr(new PointCloudXYZI(size_init_ikdtree, 1));

    PointVector().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);

    laserCloudInit->points = ikdtree.PCL_Storage;
    pcl::toROSMsg(*laserCloudInit, laser_cloud_msg);

    laser_cloud_msg.header.stamp = rclcpp::Time(lidar_end_time);
    laser_cloud_msg.header.frame_id = "camera_init";

    pubLaserCloudFullRes->publish(laser_cloud_msg);
}

void LaserMapping::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFullRes)
{
    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            // if (i % 3 == 0)
            // {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // feats_down_world->points[i].y; //
            // }
        }
        sensor_msgs::msg::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

        laserCloudmsg.header.stamp = rclcpp::Time(lidar_end_time);

        laserCloudmsg.header.frame_id = "camera_init";

        pubLaserCloudFullRes->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performences
    if (pcd_save_en) {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0
            && scan_wait_num >= pcd_save_interval) {
            pcd_index++;
            string all_points_dir(
                string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void LaserMapping::publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }

    auto laserCloudmsg = sensor_msgs::msg::PointCloud2();

    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = rclcpp::Time(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";

    pubLaserCloudFull_body->publish(laserCloudmsg);

    publish_count -= PUBFRAME_PERIOD;
}

void LaserMapping::publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& after_mapped_to_init_publisher_)
{
    // Odometry public
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";

    if (publish_odometry_without_downsample) {
        odomAftMapped.header.stamp = rclcpp::Time(time_current);
    } else {
        odomAftMapped.header.stamp = rclcpp::Time(lidar_end_time);
    }

    set_posestamp(odomAftMapped.pose.pose);

    after_mapped_to_init_publisher_->publish(odomAftMapped);

    // Transform public
    static tf2_ros::TransformBroadcaster tf_broadcaster(this);

    auto transform_stamped = geometry_msgs::msg::TransformStamped();

    transform_stamped.header.frame_id = "camera_init";
    transform_stamped.child_frame_id = "aft_mapped";

    auto t = Eigen::Affine3d(Eigen::Translation3d(
        odomAftMapped.pose.pose.position.x,
        odomAftMapped.pose.pose.position.y,
        odomAftMapped.pose.pose.position.z));
    auto r = Eigen::Affine3d(Eigen::Quaterniond(
        odomAftMapped.pose.pose.orientation.w,
        odomAftMapped.pose.pose.orientation.x,
        odomAftMapped.pose.pose.orientation.y,
        odomAftMapped.pose.pose.orientation.z));
    auto transform = (t * r).inverse();

    auto translation = Eigen::Vector3d { transform.translation() };
    auto rotation = Eigen::Quaterniond { transform.linear() };

    transform_stamped.transform.translation.x = translation.x();
    transform_stamped.transform.translation.y = translation.y();
    transform_stamped.transform.translation.z = translation.z();

    transform_stamped.transform.rotation.w = rotation.w();
    transform_stamped.transform.rotation.x = rotation.x();
    transform_stamped.transform.rotation.y = rotation.y();
    transform_stamped.transform.rotation.z = rotation.z();

    tf_broadcaster.sendTransform(transform_stamped);
}

void LaserMapping::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose.pose);

    msg_body_pose.header.stamp = rclcpp::Time(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    path.poses.emplace_back(msg_body_pose);
    pubPath->publish(path);
}
