from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    description = LaunchDescription()

    # Action: namespace
    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )
    description.add_action(declare_namespace)

    # Action: declare config
    pkg_location = get_package_share_directory("point_lio")
    config_path = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution(
            [pkg_location, "config", "mid360.yaml"]),
        description="Path to the Point-LIO config file",
    )
    description.add_action(config_path)

    # Action: point-lio
    remapping_links = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]
    point_lio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        namespace=LaunchConfiguration("namespace"),
        parameters=[LaunchConfiguration("point_lio_cfg_dir")],
        remappings=remapping_links,
        output="screen",
    )
    description.add_action(point_lio)

    transform_args = [
        # 处理雷达初始点和底盘初始点的变换
        ["0", "0", "0.6", "0", "0", "0", "world", "camera_init"],
        # 处理雷达系和底盘系的变换
        ["0", "0", "0.6", "0", "3.1415", "0", "aft_mapped", "base_link"],
    ]
    for args in transform_args:
        static_transform_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            arguments=args,
            output="screen",
        )
        description.add_action(static_transform_node)

    return description
