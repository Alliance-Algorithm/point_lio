import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():

    # Point lio launch
    package_path = get_package_share_directory("point_lio")

    use_sim_time = LaunchConfiguration("use_sim_time")
    decalare_use_sim_time = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    config_path = LaunchConfiguration("config_path")
    decalare_config_path = DeclareLaunchArgument(
        name="config_path",
        default_value=os.path.join(package_path, "config"),
        description="The path of config file (yaml)",
    )

    config_file = LaunchConfiguration("config_file")
    decalare_config_file = DeclareLaunchArgument(
        name="config_file",
        default_value="avia.yaml",
        description="Config file for livox mid-360",
    )

    point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {"use_sim_time", use_sim_time},
        ],
        output="screen",
    )

    # Rviz launch
    rviz_path_default = os.path.join(package_path, "rviz", "point_lio.rviz")

    rviz_path = LaunchConfiguration("rviz_path")
    decalare_rviz_path = DeclareLaunchArgument(
        name="rviz_path", default_value=rviz_path_default, description=""
    )

    rviz_condition = LaunchConfiguration("rviz_condition")
    decalare_rviz_condition = DeclareLaunchArgument(
        name="rviz_condition", default_value="false", description=""
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        condition=IfCondition(rviz_condition),
    )

    # Launch
    launch_description = LaunchDescription()

    launch_description.add_action(decalare_use_sim_time)
    launch_description.add_action(decalare_config_path)
    launch_description.add_action(decalare_config_file)
    launch_description.add_action(decalare_rviz_path)
    launch_description.add_action(decalare_rviz_condition)

    launch_description.add_action(point_lio_node)
    launch_description.add_action(rviz_node)

    return launch_description
