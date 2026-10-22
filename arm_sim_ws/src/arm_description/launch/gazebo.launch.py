#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gazebo 物理仿真（可选，需要额外安装依赖，见 README）：
  1) 读 config/arm_config.yaml 并强制 enable_gazebo=true 生成 URDF
  2) 启动 Gazebo 空世界 + 生成实体
  3) 启动 ros2_control 控制器管理器与控制器

用法：
  ros2 launch arm_description gazebo.launch.py

发布关节目标（位置控制）示例：
  ros2 topic pub /arm_controller/commands std_msgs/msg/Float64MultiArray \
    "{data: [0.5, -0.3, 0.2, 0.0, 0.0, 0.0]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from arm_description.urdf_generator import generate_urdf, load_config


def generate_launch_description():
    pkg_share = get_package_share_directory("arm_description")
    config_path = os.path.join(pkg_share, "config", "arm_config.yaml")
    controllers_path = os.path.join(pkg_share, "config", "controllers.yaml")

    config = load_config(config_path)
    config["enable_gazebo"] = True
    urdf = generate_urdf(config)

    gazebo_share = get_package_share_directory("gazebo_ros")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_arm"],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": urdf}, controllers_path],
        output="screen",
    )

    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    spawn_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_manager,
        spawn_joint_state_broadcaster,
        spawn_arm_controller,
    ])
