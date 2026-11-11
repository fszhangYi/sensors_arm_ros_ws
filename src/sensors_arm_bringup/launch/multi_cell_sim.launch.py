"""Sim stack: Cell C Gazebo FOV + arm_control + hw dry-run + rosbridge + web."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    bringup_share = get_package_share_directory("sensors_arm_bringup")
    hw_share = get_package_share_directory("sensors_arm_hw")
    arms_yaml = os.path.join(hw_share, "config", "default_arms.yaml")
    rosbridge_launch = os.path.join(
        get_package_share_directory("rosbridge_server"),
        "launch",
        "rosbridge_websocket_launch.xml",
    )

    # Prefer workspace venv python for FastAPI web UI
    venv_python = os.path.expanduser("~/autodl-tmp/.venv/bin/python")
    if not os.path.isfile(venv_python):
        venv_python = "/usr/bin/python3"

    return LaunchDescription([
        DeclareLaunchArgument("start_web", default_value="true"),
        DeclareLaunchArgument("web_port", default_value="6008"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("camera_hfov_deg", default_value="87.0"),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start Gazebo gzclient GUI (needs DISPLAY)",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "cell_c_gazebo_fov.launch.py")
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "camera_hfov_deg": LaunchConfiguration("camera_hfov_deg"),
                "gui": LaunchConfiguration("gui"),
                # arm_control_node owns /joint_states + Gazebo pose.
                "start_sim_joint_driver": "false",
            }.items(),
        ),

        Node(
            package="sensors_arm_control",
            executable="arm_control_node",
            name="arm_control_node",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "mode": "sim",
                "active_arm": "arm-c",
                # Gazebo kinematic plugin latches /joint_states; no gz-joint PID.
                "sync_gazebo": False,
            }],
            output="screen",
        ),

        Node(
            package="sensors_arm_hw",
            executable="hw_bridge_node",
            name="hw_bridge_node",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "dry_run": True,
                "arm_id": "arm-c",
                "config_file": arms_yaml,
                # Prevent fighting arm_control on /joint_states (Gazebo shake).
                "publish_joint_states": False,
            }],
            output="screen",
        ),

        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(rosbridge_launch),
        ),

        ExecuteProcess(
            cmd=[
                venv_python, "-m", "sensors_arm_web.web_server",
                "--port", LaunchConfiguration("web_port"),
                "--mode", "sim",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_web")),
            additional_env={
                "PYTHONPATH": os.environ.get("PYTHONPATH", ""),
            },
        ),
    ])
