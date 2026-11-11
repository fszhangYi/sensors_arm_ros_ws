"""M1: Gazebo Cell C + D405 + FOV validate node (gui optional)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory("sensors_arm_gazebo")
    fov_share = get_package_share_directory("sensors_arm_fov")
    fov_cfg = os.path.join(fov_share, "config", "cell_c_fov.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("camera_hfov_deg", default_value="87.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start Gazebo gzclient GUI (needs DISPLAY)",
        ),
        DeclareLaunchArgument(
            "start_sim_joint_driver",
            default_value="true",
            description="Use fixed-home sim_joint_driver (false when arm_control owns joints)",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, "launch", "cell_c_gazebo.launch.py")
            ),
            launch_arguments={
                "camera_hfov_deg": LaunchConfiguration("camera_hfov_deg"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "gui": LaunchConfiguration("gui"),
                "start_sim_joint_driver": LaunchConfiguration("start_sim_joint_driver"),
            }.items(),
        ),
        Node(
            package="sensors_arm_fov",
            executable="validate_node",
            name="sensors_arm_fov_validate",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "config_file": fov_cfg,
                "camera_info_topic": "/cell_c/d405/color/camera_info",
                "image_topic": "/cell_c/d405/color/image_raw",
            }],
            output="screen",
        ),
    ])
