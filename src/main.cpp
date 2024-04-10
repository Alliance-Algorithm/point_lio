#include "laser_mapping/laser_mapping.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LaserMapping>());

    rclcpp::shutdown();

    return 0;
}