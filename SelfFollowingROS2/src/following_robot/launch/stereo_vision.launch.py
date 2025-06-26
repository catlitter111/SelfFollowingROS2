#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目立体视觉系统Launch文件
========================
启动双目视觉节点，包含以下功能：
- 双目立体视觉节点
- 图像发布
- 距离测量服务

作者: AI Assistant
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
import traceback


def generate_launch_description():
    """生成Launch描述
    
    参数:
        无
        
    返回值:
        LaunchDescription: Launch配置对象
    """
    try:
        # 声明Launch参数
        camera_id_arg = DeclareLaunchArgument(
            'camera_id',
            default_value='1',
            description='相机设备ID'
        )
        
        frame_width_arg = DeclareLaunchArgument(
            'frame_width',
            default_value='1280',
            description='相机帧宽度'
        )
        
        frame_height_arg = DeclareLaunchArgument(
            'frame_height',
            default_value='480',
            description='相机帧高度'
        )
        
        fps_limit_arg = DeclareLaunchArgument(
            'fps_limit',
            default_value='30',
            description='FPS限制'
        )
        
        # 双目视觉节点
        stereo_vision_node = Node(
            package='following_robot',
            executable='stereo_vision_node',
            name='stereo_vision_node',
            output='screen',
            parameters=[{
                'camera_id': LaunchConfiguration('camera_id'),
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
                'fps_limit': LaunchConfiguration('fps_limit'),
            }],
            remappings=[
                ('/stereo/left/image', '/camera/left/image'),
                ('/stereo/get_distance', '/stereo/get_distance')
            ]
        )
        
        # 启动信息
        start_info = LogInfo(
            msg='启动双目立体视觉系统...'
        )
        
        return LaunchDescription([
            camera_id_arg,
            frame_width_arg,
            frame_height_arg,
            fps_limit_arg,
            start_info,
            stereo_vision_node,
        ])
        
    except Exception as e:
        print(f"Launch文件生成错误: {e}")
        traceback.print_exc()
        return LaunchDescription([]) 