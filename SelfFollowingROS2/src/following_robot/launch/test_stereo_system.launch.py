#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目立体视觉测试系统Launch文件
============================
同时启动双目视觉节点和测试程序

作者: AI Assistant
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
import traceback


def generate_launch_description():
    """生成完整的测试系统Launch描述
    
    参数:
        无
        
    返回值:
        LaunchDescription: Launch配置对象
    """
    try:
        # 声明Launch参数
        camera_id_arg = DeclareLaunchArgument(
            'camera_id',
            default_value='0',
            description='相机设备ID'
        )
        
        # 双目视觉节点
        stereo_vision_node = Node(
            package='following_robot',
            executable='stereo_vision_node',
            name='stereo_vision_node',
            output='screen',
            parameters=[{
                'camera_id': LaunchConfiguration('camera_id'),
            }]
        )
        
        # 延迟启动测试程序（等待双目视觉节点初始化）
        test_node = TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='following_robot',
                    executable='simple_test.py',
                    name='stereo_vision_tester',
                    output='screen'
                )
            ]
        )
        
        # 启动信息
        start_info = LogInfo(
            msg='启动双目立体视觉测试系统...'
        )
        
        instructions = LogInfo(
            msg='使用说明:\n'
                '- 双目视觉节点发布图像到 /stereo/left/image\n'
                '- 距离测量服务: /stereo/get_distance\n'
                '- 测试程序会显示图像窗口\n'
                '- 在图像上点击鼠标测量距离\n'
                '- 按 q 键退出'
        )
        
        return LaunchDescription([
            camera_id_arg,
            start_info,
            instructions,
            stereo_vision_node,
            test_node,
        ])
        
    except Exception as e:
        print(f"测试Launch文件生成错误: {e}")
        traceback.print_exc()
        return LaunchDescription([]) 