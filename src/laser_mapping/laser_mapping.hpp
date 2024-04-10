#include "../Estimator.h"
#include "../imu_processing.hpp"
#include "../parameters.h"
#include <so3_math.h>

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

#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

#define DEBUG(x) \
    if (true)    \
    RCLCPP_INFO(this->get_logger(), x)

#define POINT() \
    if (true)   \
    DEBUG("point")

// TODO
// note: make these values exiting in a reasonable place

inline bool flag_reset = false;
inline bool flag_exit = false;
inline bool is_first_scan = true;
inline bool init_map = false;

inline bool lidar_pushed = false;

inline condition_variable signal_buffer;

inline const float MOV_THRESHOLD = 1.5f;

inline mutex mtx_buffer;

inline string root_dir = ROOT_DIR;

inline int feats_down_size = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;

inline int frame_count = 0;
inline double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;

inline shared_ptr<ImuProcess> imu_process = std::make_shared<ImuProcess>();
inline PointCloudXYZI::Ptr ptr_con = std::make_shared<PointCloudXYZI>();

// Time Log Variables
inline double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
inline double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

inline vector<BoxPointType> cub_needrm;

inline deque<PointCloudXYZI::Ptr> lidar_buffer;
inline deque<double> time_buffer;
inline deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;

// Surf feature in map
inline PointCloudXYZI::Ptr feats_undistort = std::make_shared<PointCloudXYZI>();
inline PointCloudXYZI::Ptr feats_down_body_space = std::make_shared<PointCloudXYZI>();
inline PointCloudXYZI::Ptr init_feats_world = std::make_shared<PointCloudXYZI>();

inline pcl::VoxelGrid<PointType> downSizeFilterSurf;
inline pcl::VoxelGrid<PointType> downSizeFilterMap;

inline V3D euler_cur;

inline MeasureGroup Measures;

inline sensor_msgs::msg::Imu imu_last, imu_next;
inline sensor_msgs::msg::Imu::SharedPtr imu_last_ptr;
inline nav_msgs::msg::Odometry odomAftMapped;
inline nav_msgs::msg::Path path;
inline geometry_msgs::msg::PoseStamped msg_body_pose;

// Variables definition for counting
inline int frame_num = 0;

inline double aver_time_consu = 0,
              aver_time_icp = 0,
              aver_time_match = 0,
              aver_time_incre = 0,
              aver_time_solve = 0,
              aver_time_propag = 0;

inline std::time_t startTime, endTime;

// Initialize variables
inline double FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
inline double HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

// Debug record
inline FILE* fp;
inline string pos_log_dir = root_dir + "/Log/pos_log.txt";
inline ofstream fout_out, fout_imu_pbp;

inline BoxPointType LocalMap_Points;

inline bool Localmap_Initialized = false;

inline int points_cache_size = 0;
inline int process_increments = 0;

inline PointCloudXYZI::Ptr pcl_wait_pub = PointCloudXYZI::Ptr(new PointCloudXYZI(500000, 1));
inline PointCloudXYZI::Ptr pcl_wait_save = PointCloudXYZI::Ptr(new PointCloudXYZI());

// main process values
inline Eigen::Matrix<double, 24, 24> Q_input;
inline Eigen::Matrix<double, 30, 30> Q_output;

class LaserMapping : public rclcpp::Node {
public:
    LaserMapping();

    ~LaserMapping();

private:
    rclcpp::TimerBase::SharedPtr main_process_timer_;
    rclcpp::TimerBase::SharedPtr log_precess_timer_;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_lidar_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_body_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_effected_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_map_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr after_mapped_to_init_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr planner_normal_publisher_;

private: // Callback
    void main_process_timer_callback();
    void standard_pcl_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void livox_pcl_callback(livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg_in);

private: // Utilities
    void initialize();

    void read_param();

    void dump_lio_state_to_log(FILE* fp);

    void pointBodyLidarToIMU(PointType const* const pi, PointType* const po);

    void points_cache_collect();

    void lasermap_fov_segment();

    bool sync_packages(MeasureGroup& meas);

    void map_incremental();

    void publish_init_kdtree(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFullRes);
    void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFullRes);
    void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pubLaserCloudFull_body);
    void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& after_mapped_to_init_publisher_);
    void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);

    template <typename T>
    void set_posestamp(T& out)
    {
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
};
