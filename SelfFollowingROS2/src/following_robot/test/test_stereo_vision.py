#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目立体视觉测试程序
=================
功能包括：
- 订阅左目图像并显示
- 通过控制台打印画面中心物体距离
- 支持键盘交互

作者: AI Assistant
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.srv import GetDistance

import cv2
import numpy as np
import traceback
import threading
import time


class StereoVisionTester(Node):
    """双目视觉测试节点类
    
    参数:
        无
        
    返回值:
        节点实例
    """
    
    def __init__(self):
        try:
            super().__init__('stereo_vision_tester')
            
            # CV Bridge for image conversion
            self.bridge = CvBridge()
            
            # 当前图像数据
            self.current_image = None
            self.image_width = 0
            self.image_height = 0
            self.image_lock = threading.Lock()
            
            # 创建订阅者
            self.image_sub = self.create_subscription(
                Image,
                '/stereo/left/image',
                self.image_callback,
                10
            )
            
            # 创建服务客户端
            self.distance_client = self.create_client(
                GetDistance,
                '/stereo/get_distance'
            )
            
            # 等待服务
            self.wait_for_service()
            
            # 启动显示线程
            self.running = True
            self.display_thread = threading.Thread(target=self.display_loop)
            self.display_thread.daemon = True
            self.display_thread.start()
            
            # 启动距离测量线程
            self.distance_thread = threading.Thread(target=self.distance_measurement_loop)
            self.distance_thread.daemon = True
            self.distance_thread.start()
            
            self.get_logger().info('双目视觉测试节点已启动')
            self.get_logger().info('键盘控制:')
            self.get_logger().info('- 按 "q" 退出')
            self.get_logger().info('- 鼠标点击图像测量距离')
            
        except Exception as e:
            self.get_logger().error(f"测试节点初始化错误: {e}")
            traceback.print_exc()

    def wait_for_service(self):
        """等待距离测量服务可用
        
        参数:
            无
            
        返回值:
            无
        """
        try:
            self.get_logger().info('等待距离测量服务...')
            while not self.distance_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('距离测量服务不可用，等待中...')
            self.get_logger().info('距离测量服务已连接')
            
        except Exception as e:
            self.get_logger().error(f"等待服务错误: {e}")
            traceback.print_exc()

    def image_callback(self, msg):
        """图像订阅回调函数
        
        参数:
            msg (Image): ROS图像消息
            
        返回值:
            无
        """
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 更新当前图像
            with self.image_lock:
                self.current_image = cv_image.copy()
                self.image_height, self.image_width = cv_image.shape[:2]
                
        except Exception as e:
            self.get_logger().error(f"图像回调错误: {e}")
            traceback.print_exc()

    def display_loop(self):
        """图像显示循环
        
        参数:
            无
            
        返回值:
            无
        """
        # 创建窗口
        cv2.namedWindow('Stereo Left Image', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('Stereo Left Image', self.mouse_callback)
        
        while self.running:
            try:
                with self.image_lock:
                    if self.current_image is not None:
                        display_image = self.current_image.copy()
                        
                        # 绘制中心点
                        if self.image_width > 0 and self.image_height > 0:
                            center_x = self.image_width // 2
                            center_y = self.image_height // 2
                            
                            # 绘制十字线
                            cv2.line(display_image, (center_x - 20, center_y), 
                                    (center_x + 20, center_y), (0, 255, 0), 2)
                            cv2.line(display_image, (center_x, center_y - 20), 
                                    (center_x, center_y + 20), (0, 255, 0), 2)
                            
                            # 添加文本
                            cv2.putText(display_image, 'Center', 
                                      (center_x - 30, center_y - 30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        # 显示图像
                        cv2.imshow('Stereo Left Image', display_image)
                
                # 检查键盘输入
                key = cv2.waitKey(30) & 0xFF
                if key == ord('q'):
                    self.get_logger().info('用户按下q键，退出程序')
                    self.running = False
                    break
                    
            except Exception as e:
                self.get_logger().error(f"显示循环错误: {e}")
                traceback.print_exc()
                time.sleep(0.1)
        
        cv2.destroyAllWindows()

    def mouse_callback(self, event, x, y, flags, param):
        """鼠标点击回调函数
        
        参数:
            event: 鼠标事件类型
            x (int): 鼠标x坐标
            y (int): 鼠标y坐标
            flags: 鼠标标志
            param: 额外参数
            
        返回值:
            无
        """
        try:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.get_logger().info(f'鼠标点击位置: ({x}, {y})')
                self.measure_distance_at_point(x, y)
                
        except Exception as e:
            self.get_logger().error(f"鼠标回调错误: {e}")
            traceback.print_exc()

    def distance_measurement_loop(self):
        """距离测量循环 - 自动测量画面中心距离
        
        参数:
            无
            
        返回值:
            无
        """
        while self.running:
            try:
                # 每2秒测量一次中心距离
                time.sleep(2.0)
                
                with self.image_lock:
                    if self.image_width > 0 and self.image_height > 0:
                        center_x = self.image_width // 2
                        center_y = self.image_height // 2
                        
                        # 测量中心点距离
                        self.measure_distance_at_point(center_x, center_y, is_center=True)
                        
            except Exception as e:
                self.get_logger().error(f"距离测量循环错误: {e}")
                traceback.print_exc()
                time.sleep(1.0)

    def measure_distance_at_point(self, x, y, is_center=False):
        """测量指定点的距离
        
        参数:
            x (int): 像素x坐标
            y (int): 像素y坐标
            is_center (bool): 是否为中心点测量
            
        返回值:
            无
        """
        try:
            if not self.distance_client.service_is_ready():
                self.get_logger().warn('距离测量服务不可用')
                return
            
            # 创建服务请求
            request = GetDistance.Request()
            request.x = x
            request.y = y
            
            # 异步调用服务
            future = self.distance_client.call_async(request)
            
            def distance_callback(future):
                try:
                    response = future.result()
                    if response.success:
                        prefix = "[中心点]" if is_center else f"[点击点({x},{y})]"
                        self.get_logger().info(f'{prefix} 距离: {response.distance:.2f}m')
                    else:
                        prefix = "[中心点]" if is_center else f"[点击点({x},{y})]"
                        self.get_logger().warn(f'{prefix} 测量失败: {response.message}')
                        
                except Exception as e:
                    self.get_logger().error(f"距离服务回调错误: {e}")
                    traceback.print_exc()
            
            future.add_done_callback(distance_callback)
            
        except Exception as e:
            self.get_logger().error(f"距离测量错误: {e}")
            traceback.print_exc()

    def destroy_node(self):
        """节点销毁时的清理工作
        
        参数:
            无
            
        返回值:
            无
        """
        try:
            self.running = False
            self.get_logger().info('双目视觉测试节点已关闭')
            super().destroy_node()
            
        except Exception as e:
            self.get_logger().error(f"节点销毁错误: {e}")
            traceback.print_exc()


def main(args=None):
    """主函数
    
    参数:
        args: 命令行参数
        
    返回值:
        无
    """
    try:
        rclpy.init(args=args)
        node = StereoVisionTester()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
            
    except Exception as e:
        print(f"主函数错误: {e}")
        traceback.print_exc()


if __name__ == '__main__':
    main() 