from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag_path = "/workspaces/data/bag/520双创飙车事件/"
    play_bag = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag_path],
        output="screen",
    )

    pkg_share = get_package_share_directory("point_lio")
    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "point_lio.launch.py"])
        )
    )
    return LaunchDescription([point_lio, play_bag])
