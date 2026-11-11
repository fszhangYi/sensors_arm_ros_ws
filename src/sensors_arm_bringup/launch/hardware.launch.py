"""Hardware bringup: control + hw bridge (no Gazebo) + rosbridge + web."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    hw_share = get_package_share_directory("sensors_arm_hw")
    fov_share = get_package_share_directory("sensors_arm_fov")
    arms_yaml = os.path.join(hw_share, "config", "default_arms.yaml")
    fov_cfg = os.path.join(fov_share, "config", "cell_c_fov.yaml")
    rosbridge_launch = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch",
        "rosbridge_websocket_launch.xml",
    )
    venv_python = os.path.expanduser("~/autodl-tmp/.venv/bin/python")
    if not os.path.isfile(venv_python):
        venv_python = "/usr/bin/python3"

    return LaunchDescription([
        DeclareLaunchArgument("start_web", default_value="true"),
        DeclareLaunchArgument("web_port", default_value="6008"),
        DeclareLaunchArgument("dry_run", default_value="false"),
        DeclareLaunchArgument("arm_id", default_value="arm-c"),

        Node(
            package="sensors_arm_control",
            executable="arm_control_node",
            name="arm_control_node",
            parameters=[{"mode": "hardware", "active_arm": LaunchConfiguration("arm_id")}],
            output="screen",
        ),
        Node(
            package="sensors_arm_hw",
            executable="hw_bridge_node",
            name="hw_bridge_node",
            parameters=[{
                "dry_run": LaunchConfiguration("dry_run"),
                "arm_id": LaunchConfiguration("arm_id"),
                "config_file": arms_yaml,
            }],
            output="screen",
        ),
        Node(
            package="sensors_arm_fov",
            executable="validate_node",
            name="sensors_arm_fov_validate",
            parameters=[{"config_file": fov_cfg}],
            output="screen",
        ),
        IncludeLaunchDescription(XMLLaunchDescriptionSource(rosbridge_launch)),
        ExecuteProcess(
            cmd=[
                venv_python, "-m", "sensors_arm_web.web_server",
                "--port", LaunchConfiguration("web_port"),
                "--mode", "hardware",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_web")),
        ),
    ])
