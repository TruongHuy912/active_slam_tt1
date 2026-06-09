import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    slam_config = LaunchConfiguration("slam_config")
    controller_params_file = LaunchConfiguration("controller_params_file")
    world_name = LaunchConfiguration("world_name")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="empty",
        description="Gazebo world name from bumperbot_description/worlds without the .world suffix"
    )
    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to the SLAM Toolbox YAML file"
    )
    controller_params_file_arg = DeclareLaunchArgument(
        "controller_params_file",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "config",
            "controller_server.yaml"
        ),
        description="Full path to the Nav2 controller_server parameter file"
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
        launch_arguments={
            "world_name": world_name,
        }.items(),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_config": slam_config,
        }.items(),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "controller_params_file": controller_params_file,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("nav2_bringup"),
                "rviz",
                "nav2_default_view.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
    return LaunchDescription([
        use_slam_arg,
        world_name_arg,
        slam_config_arg,
        controller_params_file_arg,
        gazebo,
        controller,
        joystick,
        localization,
        slam,
        navigation,
        rviz,
    ])
