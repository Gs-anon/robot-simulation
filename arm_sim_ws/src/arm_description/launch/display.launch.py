#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RViz 运动学仿真：
  1) 读 config/arm_config.yaml
  2) 用 urdf_generator 在内存中生成 URDF
  3) 交给 robot_state_publisher 发布 TF
  4) joint_state_publisher_gui 提供关节滑杆（拖动即可动）
  5) RViz2 显示

用法（colcon build + source 后）：
  ros2 launch arm_description display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from arm_description.urdf_generator import generate_urdf, load_config


def generate_launch_description():
    pkg_share = get_package_share_directory("arm_description")
    config_path = os.path.join(pkg_share, "config", "arm_config.yaml")
    rviz_path = os.path.join(pkg_share, "rviz", "arm.rviz")

    config = load_config(config_path)
    # RViz 运动学仿真不需要 Gazebo 接口，强制关掉
    config["enable_gazebo"] = False
    urdf = generate_urdf(config)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": urdf}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
