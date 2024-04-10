#include "laser_mapping.hpp"

void LaserMapping::standard_pcl_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    mtx_buffer.lock();
    scan_count++;
    double preprocess_start_time = omp_get_wtime();
    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {

        RCLCPP_ERROR(rclcpp::get_logger("standard_pcl_callback"), "lidar loop back, clear buffer");

        // lidar_buffer.shrink_to_fit();

        mtx_buffer.unlock();
        signal_buffer.notify_all();
        return;
    }

    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
    double time_div = rclcpp::Time(msg->header.stamp).seconds();
    preprocess->process(msg, ptr);

    if (cut_frame) {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {
            ptr_div->push_back(ptr->points[i]);
            // cout << "check time:" << ptr->points[i].curvature << endl;
            if (ptr->points[i].curvature / double(1000) + rclcpp::Time(msg->header.stamp).seconds() - time_div
                > cut_frame_time_interval) {
                if (ptr_div->size() < 1)
                    return;
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
            time_con = last_timestamp_lidar; // rclcpp::Time(msg->header.stamp).seconds();
        }
        if (frame_count < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
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
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    signal_buffer.notify_all();
}

void LaserMapping::livox_pcl_callback(livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
{
    mtx_buffer.lock();

    double preprocess_start_time = omp_get_wtime();

    scan_count++;

    if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {

        RCLCPP_ERROR(this->get_logger(), "Lidar loop back, clear buffer");

        mtx_buffer.unlock();
        signal_buffer.notify_all();

        return;
    }

    last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());

    preprocess->process(msg, ptr);

    double time_div = rclcpp::Time(msg->header.stamp).seconds();

    if (cut_frame) {

        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++) {

            ptr_div->push_back(ptr->points[i]);

            if (ptr->points[i].curvature / double(1000)
                    + rclcpp::Time(msg->header.stamp).seconds()
                    - time_div
                > cut_frame_time_interval) {

                if (ptr_div->size() < 1)
                    return;

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
            time_con = last_timestamp_lidar;
        }
        if (frame_count < con_frame_num) {
            for (int i = 0; i < ptr->size(); i++) {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
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
        // DEBUG("Here you push a lidar msg");

        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
    }

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    signal_buffer.notify_all();
}

void LaserMapping::imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
{
    publish_count++;

    // Repeated construction ?
    // from Creeper5820
    // auto msg = sensor_msgs::msg::Imu::SharedPtr(new sensor_msgs::msg::Imu(*msg_in));

    // note:    the construct of rclcpp::Time is in a mess,
    //          ros2 does not support construct this using seconds only;
    //          I *delete* the transformation from imu timestmap to msg timestamp,
    //          because the time_lag_imu_to_lidar equals zero

    // auto seconds = rclcpp::Time(msg->header.stamp).seconds() - time_lag_imu_to_lidar;
    // msg->header.stamp = rclcpp::Time(seconds, 0);

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {

        RCLCPP_ERROR(this->get_logger(),
            "imu loop back, clear deque");

        mtx_buffer.unlock();
        signal_buffer.notify_all();

        return;
    }

    last_timestamp_imu = timestamp;

    imu_deque.emplace_back(msg);
    mtx_buffer.unlock();
    signal_buffer.notify_all();
}
