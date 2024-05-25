from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    # launch parameters
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    use_rosbridge = LaunchConfiguration("use_rosbridge", default="True")

    # package cafeteriabot_webapp
    path_root = Path(get_package_share_directory("cafeteriabot_webapp"))

    # package rosbridge_server
    path_web2 = Path(get_package_share_directory("rosbridge_server"))

    # return launch
    return LaunchDescription(
        [
            DeclareLaunchArgument(name="use_sim_time", default_value="True"),
            DeclareLaunchArgument(name="use_rosbridge", default_value="True"),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    (path_web2 / "launch" / "rosbridge_websocket_launch.xml").as_posix()
                ),
                condition=IfCondition(LaunchConfiguration('use_rosbridge')),
            ),
            ExecuteProcess(
                cmd=["python3", "-m", "http.server", "7000"],
                cwd=path_root.as_posix(),
                output="screen",
            ),
        ]
    )
