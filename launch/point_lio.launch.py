from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    namespace = LaunchConfiguration("namespace")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")

    point_lio_dir = get_package_share_directory("point_lio")

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the node",
    )

    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution(
            [point_lio_dir, "config", "mid360.yaml"]),
        description="Path to the Point-LIO config file",
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        namespace=namespace,
        parameters=[point_lio_cfg_dir],
        remappings=remappings,
        output="screen",
    )

    # TODO: Remove soon
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0.6", "0", "0", "0", "world", "camera_init"],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_point_lio_cfg_dir)
    ld.add_action(start_point_lio_node)
    ld.add_action(static_tf_node)

    return ld
