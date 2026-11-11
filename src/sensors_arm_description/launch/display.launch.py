"""Display a cell URDF with robot_state_publisher (+ optional joint_state_publisher_gui)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("sensors_arm_description")
    default_model = os.path.join(share, "urdf", "cell_c.urdf.xacro")

    model = LaunchConfiguration("model")
    use_gui = LaunchConfiguration("use_gui")

    robot_description = ParameterValue(Command(["xacro ", model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model),
        DeclareLaunchArgument("use_gui", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(use_gui),
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
            output="screen",
        ),
    ])
