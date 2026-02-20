#!/usr/bin/env python3
"""
颜色分割节点启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('widowx_custom_perception'),
            'config',
            'color_segmentation_params.yaml'
        ]),
        description='Path to color segmentation parameter file'
    )

    # 颜色分割节点
    color_segmentation_node = Node(
        package='widowx_custom_perception',
        executable='color_segmentation_node',
        name='color_segmentation_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,
        color_segmentation_node,
    ])

