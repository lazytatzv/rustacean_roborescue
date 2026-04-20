"""
MoveIt Servo launch for rustacean_roborescue.

Prerequisite: start control.launch.py with arm_backend:=ros2_control first so that
controller_manager, joint_state_broadcaster, and joint_trajectory_controller are running.

This file starts:
  1. move_group   – planning scene + collision checking (required by servo_node)
  2. servo_node   – real-time Cartesian/joint servo (started after move_group is ready)
"""

import os

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers.on_process_start import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _load_yaml(package_name: str, rel_path: str):
    path = os.path.join(get_package_share_directory(package_name), rel_path)
    with open(path) as f:
        return yaml.safe_load(f)


def _load_robot_description() -> str:
    """sekirei_moveit.xacro を use_real_hw:=true で展開して返す。
    dynamixel_hardware_interface が未インストールの場合は use_real_hw:=false (mock) にフォールバック。
    """
    import subprocess

    xacro_path = os.path.join(
        get_package_share_directory("sekirei_moveit"), "urdf", "sekirei_moveit.xacro"
    )

    use_real_hw = "false"
    dxl_model_folder = ""
    try:
        dxl_model_folder = os.path.join(
            get_package_share_directory("dynamixel_hardware_interface"), "param", "dxl_model"
        )
        use_real_hw = "true"
    except PackageNotFoundError:
        print("[moveit_servo.launch] dynamixel_hardware_interface not found; using mock hardware")

    args = [
        "xacro", xacro_path,
        f"use_real_hw:={use_real_hw}",
    ]
    if dxl_model_folder:
        args.append(f"dynamixel_model_folder:={dxl_model_folder}")

    result = subprocess.run(args, capture_output=True, text=True, check=True)
    return result.stdout


def launch_setup(context, *args, **kwargs):
    moveit_cfg = get_package_share_directory("sekirei_moveit")

    robot_description = _load_robot_description()

    srdf_path = os.path.join(moveit_cfg, "config", "sekirei.srdf")
    with open(srdf_path) as f:
        robot_description_semantic = f.read()

    kinematics = _load_yaml("sekirei_moveit", "config/kinematics.yaml")
    joint_limits = _load_yaml("sekirei_moveit", "config/joint_limits.yaml")
    ompl_planning = _load_yaml("sekirei_moveit", "config/ompl_planning.yaml")
    moveit_controllers = _load_yaml("sekirei_moveit", "config/moveit_controllers.yaml")

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 3.0,
        "trajectory_execution.allowed_goal_duration_margin": 5.0,
        "trajectory_execution.allowed_start_tolerance": 0.5,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_params = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": kinematics,
        "robot_description_planning": joint_limits,
    }

    # Flatten OMPL config with ompl. prefix as move_group expects
    for key, value in ompl_planning.items():
        if isinstance(value, list):
            move_group_params[f"ompl.{key}"] = ParameterValue(value)
        elif isinstance(value, dict):
            for k2, v2 in value.items():
                if isinstance(v2, list):
                    move_group_params[f"ompl.{key}.{k2}"] = ParameterValue(v2)
                else:
                    move_group_params[f"ompl.{key}.{k2}"] = v2
        else:
            move_group_params[f"ompl.{key}"] = value

    move_group_params.update(trajectory_execution)
    move_group_params.update(planning_scene_monitor)

    if moveit_controllers:
        scm = moveit_controllers.get("moveit_simple_controller_manager", {})
        if "controller_names" in scm and isinstance(scm["controller_names"], list):
            scm["controller_names"] = ParameterValue(scm["controller_names"])
        move_group_params.update(moveit_controllers)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        output="screen",
        parameters=[
            os.path.join(moveit_cfg, "config", "moveit_servo.yaml"),
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            {"robot_description_kinematics": kinematics},
        ],
    )

    # Delay servo_node until move_group's planning scene service is up
    _guard = {"started": False}

    def _start_servo_when_ready(ctx, *a, **k):
        if _guard["started"]:
            return []
        _guard["started"] = True

        import time

        deadline = time.time() + 30.0
        while time.time() < deadline:
            try:
                services = os.popen("ros2 service list 2>/dev/null").read().splitlines()
            except Exception:
                services = []
            if "/move_group/get_planning_scene" in services:
                return [servo_node]
            time.sleep(0.5)

        print("[moveit_servo.launch] Warning: timed out waiting for move_group, starting servo anyway")
        return [servo_node]

    return [
        move_group_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=move_group_node,
                on_start=[OpaqueFunction(function=_start_servo_when_ready)],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            OpaqueFunction(function=launch_setup),
        ]
    )
