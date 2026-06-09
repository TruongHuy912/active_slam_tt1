import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("bumperbot_active_slam")
    default_config = os.path.join(package_share, "config", "active_slam.yaml")

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_navigation = LaunchConfiguration("enable_navigation")
    enable_efficient_utility = LaunchConfiguration("enable_efficient_utility")
    scoring_mode = LaunchConfiguration("scoring_mode")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_config,
        description="Path to Active SLAM parameter file.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock.",
    )
    enable_navigation_arg = DeclareLaunchArgument(
        "enable_navigation",
        default_value="false",
        description="Allow active_slam_explorer to send Nav2 NavigateToPose goals.",
    )
    enable_efficient_utility_arg = DeclareLaunchArgument(
        "enable_efficient_utility",
        default_value="false",
        description="Enable Efficient Active SLAM utility ranking.",
    )
    scoring_mode_arg = DeclareLaunchArgument(
        "scoring_mode",
        default_value="safe_viewpoint",
        description="Candidate scoring mode.",
    )

    active_slam_node = Node(
        package="bumperbot_active_slam",
        executable="active_slam_explorer",
        name="active_slam_explorer",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
            {"enable_navigation": enable_navigation},
            {"enable_efficient_utility": enable_efficient_utility},
            {"scoring_mode": scoring_mode},
        ],
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        enable_navigation_arg,
        enable_efficient_utility_arg,
        scoring_mode_arg,
        active_slam_node,
    ])
