"""Gazebo + Cell C + D405-like camera (gui:=true starts gzclient)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo_share = get_package_share_directory("sensors_arm_gazebo")
    desc_share = get_package_share_directory("sensors_arm_description")

    world = os.path.join(gazebo_share, "worlds", "cell_c_empty.world")
    xacro_file = os.path.join(gazebo_share, "urdf", "cell_c_gazebo.urdf.xacro")

    use_sim_time = LaunchConfiguration("use_sim_time")
    hfov = LaunchConfiguration("camera_hfov_deg")
    gui = LaunchConfiguration("gui")

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_file,
            " camera_hfov_deg:=", hfov,
        ]),
        value_type=str,
    )

    # package:// → model:// needs .../share on GAZEBO_MODEL_PATH.
    # CameraSensor also needs /usr/share/gazebo-11 on GAZEBO_RESOURCE_PATH
    # (media/rtshaderlib). Never drop the system prefix.
    def _merge_paths(*parts: str) -> str:
        seen = []
        for part in parts:
            for p in str(part or "").split(":"):
                if p and p not in seen and (os.path.isdir(p) or p.endswith("share")):
                    seen.append(p)
        return ":".join(seen)

    gazebo_model_path = _merge_paths(
        os.path.join(gazebo_share, "models"),
        os.path.dirname(desc_share),
        os.path.dirname(gazebo_share),
        "/usr/share/gazebo-11/models",
        "/usr/share/gazebo/models",
        os.environ.get("GAZEBO_MODEL_PATH", ""),
    )
    gazebo_resource_path = _merge_paths(
        "/usr/share/gazebo-11",
        "/usr/share/gazebo",
        os.environ.get("GAZEBO_RESOURCE_PATH", ""),
        gazebo_model_path,
    )
    # Workspace install/lib must be first so libsensors_arm_kinematic_joints.so resolves.
    ws_lib = ""
    try:
        from ament_index_python.packages import get_package_prefix
        ws_lib = os.path.join(get_package_prefix("sensors_arm_gazebo"), "lib")
    except Exception:
        pass
    plugin_path = _merge_paths(
        ws_lib,
        "/opt/ros/foxy/lib",
        "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",
        os.environ.get("GAZEBO_PLUGIN_PATH", ""),
    )
    ld_path = _merge_paths(
        ws_lib,
        os.environ.get("LD_LIBRARY_PATH", ""),
    )
    gz_env = {
        "GAZEBO_MODEL_PATH": gazebo_model_path,
        "GAZEBO_RESOURCE_PATH": gazebo_resource_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "LD_LIBRARY_PATH": ld_path,
        # Empty URI disables online model database lookups.
        "GAZEBO_MODEL_DATABASE_URI": os.environ.get("GAZEBO_MODEL_DATABASE_URI", ""),
        # Keep Gazebo ROS services on loopback (see scripts/env.sh).
        "GAZEBO_IP": os.environ.get("GAZEBO_IP", "127.0.0.1"),
        "GAZEBO_MASTER_URI": os.environ.get(
            "GAZEBO_MASTER_URI", "http://127.0.0.1:11345"
        ),
    }

    # Home joints from sensors-arm default.yaml arm-c (+ gripper_opening)
    # FOV markers are parented to fingertip links so they track opening.
    home = {
        "joint1": 2.8695,
        "joint2": -1.9539,
        "joint3": 1.8019,
        "joint4": -1.6618,
        "joint5": 4.7475,
        "joint6": -0.0275,
        "gripper_opening": 0.0,
    }

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("camera_hfov_deg", default_value="87.0"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument(
            "start_sim_joint_driver",
            default_value="true",
            description="Publish fixed home joint_states + sync Gazebo (disable when arm_control owns joints)",
        ),

        ExecuteProcess(
            cmd=[
                "gzserver", "--verbose", world,
                "-s", "libgazebo_ros_init.so",
                "-s", "libgazebo_ros_factory.so",
            ],
            output="screen",
            additional_env=gz_env,
        ),

        # Optional classic GUI; default off for AutoDL / CI headless.
        ExecuteProcess(
            cmd=["gzclient", "--verbose"],
            output="screen",
            additional_env=gz_env,
            condition=IfCondition(gui),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }],
            output="screen",
        ),

        # Publish home pose as joint_states (drives robot_state_publisher TF for FOV).
        # multi_cell_sim turns this off so arm_control_node owns joints + Gazebo sync.
        Node(
            package="sensors_arm_control",
            executable="sim_joint_driver",
            name="sim_joint_driver",
            parameters=[{
                "use_sim_time": use_sim_time,
                "joints": list(home.keys()),
                "positions": list(home.values()),
                "rate_hz": 50.0,
            }],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_sim_joint_driver")),
        ),

        # Software GL / first boot can take >3s before GazeboRosFactory advertises.
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    name="spawn_cell_c",
                    arguments=[
                        "-entity", "cell_c",
                        "-topic", "robot_description",
                        "-x", "0", "-y", "0", "-z", "0.01",
                        "-timeout", "120.0",
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
