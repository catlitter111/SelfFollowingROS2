#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双目立体视觉ROS2节点
===================
基于OpenCV的双目立体视觉节点，支持：
- 发布左目图像
- 提供距离测量服务
- 实时立体匹配

作者: AI Assistant
优化: 添加完整的错误处理和日志记录
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
from functools import lru_cache


class StereoConfig:
    """立体视觉系统配置类
    
    参数:
        无
        
    返回值:
        配置对象实例
    """
    
    def __init__(self):
        try:
            # 相机内参和外参
            self.baseline = 25.100  # 基线距离
            self.focal_length = 663  # 焦距
            self.cx = 317  # 光心x坐标
            self.cy = 210  # 光心y坐标

            # SGBM算法参数
            self.minDisparity = 3
            self.numDisparities = 16  # 必须是16的倍数
            self.blockSize = 7
            self.P1 = 1176
            self.P2 = 4704
            self.disp12MaxDiff = 4
            self.preFilterCap = 31
            self.uniquenessRatio = 10
            self.speckleWindowSize = 100
            self.speckleRange = 32
            self.mode = cv2.STEREO_SGBM_MODE_SGBM_3WAY

            # WLS滤波器参数
            self.wls_lambda = 8000.0  # 滤波强度
            self.wls_sigma = 1.5  # 颜色相似性敏感度

            # 相机参数
            self.camera_id = 1
            self.frame_width = 1280
            self.frame_height = 480
            self.fps_limit = 30

            # 距离测量范围
            self.min_distance_mm = 100.0
            self.max_distance_mm = 10000.0
            
        except Exception as e:
            print(f"配置初始化错误: {e}")
            traceback.print_exc()


class StereoCamera:
    """双目相机参数类
    
    参数:
        无
        
    返回值:
        相机参数对象实例
    """
    
    def __init__(self):
        try:
            # 左相机内参
            self.cam_matrix_left = np.array([[660.1946, 0, 326.3185], 
                                           [0, 660.8720, 207.1556], 
                                           [0, 0, 1]])

            # 右相机内参
            self.cam_matrix_right = np.array([[665.1635, 0, 319.9729], 
                                            [0, 665.7919, 212.9630], 
                                            [0, 0, 1]])

            # 左右相机畸变系数:[k1, k2, p1, p2, k3]
            self.distortion_l = np.array([[-0.0682, 0.1546, 0, 0, 0]])
            self.distortion_r = np.array([[-0.0749, 0.1684, 0, 0, 0]])

            # 旋转矩阵
            self.R = np.array([[1.0, 6.140854786327222e-04, -0.0022],
                              [-6.240288417695294e-04, 1, -0.0046],
                              [0.0022, 0.0046, 1]])

            # 平移矩阵
            self.T = np.array([[-25.0961], [-0.0869], [-0.1893]])

            # 焦距和基线距离
            self.focal_length = 663
            self.baseline = abs(self.T[0][0])

            # Q矩阵（视差到深度的映射矩阵）
            self.Q = None  # 在getRectifyTransform中计算
            
        except Exception as e:
            print(f"相机参数初始化错误: {e}")
            traceback.print_exc()


class StereoVisionNode(Node):
    """双目立体视觉ROS2节点类
    
    参数:
        无
        
    返回值:
        节点实例
    """
    
    def __init__(self):
        try:
            super().__init__('stereo_vision_node')
            
            # 初始化配置
            self.config = StereoConfig()
            self.stereo_config = StereoCamera()
            
            # CV Bridge for image conversion
            self.bridge = CvBridge()
            
            # 相机对象
            self.cap = None
            self.running = False
            
            # 当前帧数据
            self.current_left_image = None
            self.current_points_3d = None
            self.frame_lock = threading.Lock()
            
            # 预先计算校正变换矩阵
            self.setup_stereo_rectification()
            
            # 初始化相机
            self.init_camera()
            
            # 创建发布者
            self.left_image_pub = self.create_publisher(
                Image, 
                '/stereo/left/image', 
                10
            )
            
            # 创建服务
            self.distance_service = self.create_service(
                GetDistance,
                '/stereo/get_distance',
                self.get_distance_callback
            )
            
            # 启动图像捕获线程
            self.start_capture_thread()
            
            self.get_logger().info('双目立体视觉节点已启动')
            
        except Exception as e:
            self.get_logger().error(f"节点初始化错误: {e}")
            traceback.print_exc()

    def setup_stereo_rectification(self):
        """设置立体校正参数
        
        参数:
            无
            
        返回值:
            无
        """
        try:
            height, width = self.config.frame_height, self.config.frame_width // 2
            self.map1x, self.map1y, self.map2x, self.map2y, self.Q = self.get_rectify_transform(
                height, width, self.stereo_config
            )
            self.get_logger().info('立体校正参数设置完成')
            
        except Exception as e:
            self.get_logger().error(f"立体校正设置错误: {e}")
            traceback.print_exc()

    def get_rectify_transform(self, height, width, stereo_config):
        """获取畸变校正和立体校正的映射变换矩阵
        
        参数:
            height (int): 图像高度
            width (int): 图像宽度  
            stereo_config (StereoCamera): 立体相机配置
            
        返回值:
            tuple: (map1x, map1y, map2x, map2y, Q) 校正映射矩阵和重投影矩阵
        """
        try:
            left_K = stereo_config.cam_matrix_left
            right_K = stereo_config.cam_matrix_right
            left_dist = stereo_config.distortion_l
            right_dist = stereo_config.distortion_r
            R = stereo_config.R
            T = stereo_config.T

            # 计算立体校正参数
            R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
                left_K, left_dist, right_K, right_dist,
                (width, height), R, T,
                flags=cv2.CALIB_ZERO_DISPARITY,
                alpha=0.5
            )

            # 保存Q矩阵
            stereo_config.Q = Q

            # 生成映射矩阵
            map1x, map1y = cv2.initUndistortRectifyMap(
                left_K, left_dist, R1, P1, (width, height), cv2.CV_32FC1
            )
            map2x, map2y = cv2.initUndistortRectifyMap(
                right_K, right_dist, R2, P2, (width, height), cv2.CV_32FC1
            )

            return map1x, map1y, map2x, map2y, Q
            
        except Exception as e:
            self.get_logger().error(f"校正变换计算错误: {e}")
            traceback.print_exc()
            return None, None, None, None, None

    def init_camera(self):
        """初始化相机
        
        参数:
            无
            
        返回值:
            无
        """
        try:
            self.cap = cv2.VideoCapture(self.config.camera_id)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)

            if not self.cap.isOpened():
                self.get_logger().warn(f"无法打开相机ID {self.config.camera_id}，尝试使用默认相机...")
                self.config.camera_id = 0
                self.cap = cv2.VideoCapture(self.config.camera_id)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)

                if not self.cap.isOpened():
                    raise RuntimeError("错误：无法打开相机！")
                    
            self.get_logger().info(f'相机初始化成功，ID: {self.config.camera_id}')
            
        except Exception as e:
            self.get_logger().error(f"相机初始化错误: {e}")
            traceback.print_exc()

    def start_capture_thread(self):
        """启动图像捕获线程
        
        参数:
            无
            
        返回值:
            无
        """
        try:
            self.running = True
            self.capture_thread = threading.Thread(target=self.capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            self.get_logger().info('图像捕获线程已启动')
            
        except Exception as e:
            self.get_logger().error(f"捕获线程启动错误: {e}")
            traceback.print_exc()

    def capture_loop(self):
        """图像捕获循环
        
        参数:
            无
            
        返回值:
            无
        """
        while self.running:
            try:
                if self.cap is None:
                    time.sleep(0.1)
                    continue
                    
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn("无法获取图像")
                    time.sleep(0.5)
                    continue

                # 调整图像大小
                if frame.shape[1] != self.config.frame_width or frame.shape[0] != self.config.frame_height:
                    frame = cv2.resize(frame, (self.config.frame_width, self.config.frame_height))

                # 分割左右图像
                mid_x = frame.shape[1] // 2
                left_half = frame[:, :mid_x]
                right_half = frame[:, mid_x:]

                # 处理立体视觉
                left_image, points_3d = self.process_stereo_frame(left_half, right_half)
                
                if left_image is not None:
                    # 更新当前帧数据
                    with self.frame_lock:
                        self.current_left_image = left_image
                        self.current_points_3d = points_3d

                    # 发布左目图像
                    self.publish_left_image(left_image)

                # 限制帧率
                time.sleep(1.0 / self.config.fps_limit)
                
            except Exception as e:
                self.get_logger().error(f"图像捕获循环错误: {e}")
                traceback.print_exc()
                time.sleep(0.1)

    def process_stereo_frame(self, left_image, right_image):
        """处理立体视觉帧
        
        参数:
            left_image (ndarray): 左目图像
            right_image (ndarray): 右目图像
            
        返回值:
            tuple: (处理后的左目图像, 3D点云数据)
        """
        try:
            # 消除畸变
            iml = self.undistortion(left_image, self.stereo_config.cam_matrix_left, 
                                  self.stereo_config.distortion_l)
            imr = self.undistortion(right_image, self.stereo_config.cam_matrix_right, 
                                  self.stereo_config.distortion_r)

            # 预处理图像
            iml_processed, imr_processed = self.preprocess(iml, imr)

            # 图像校正
            iml_rectified, imr_rectified = self.rectify_image(
                iml_processed, imr_processed, 
                self.map1x, self.map1y, self.map2x, self.map2y
            )

            # 计算视差图
            disparity = self.stereo_match_sgbm(iml_rectified, imr_rectified)

            # 计算3D点云
            points_3d = self.reproject_to_3d(disparity, self.Q)

            return iml_rectified, points_3d
            
        except Exception as e:
            self.get_logger().error(f"立体视觉帧处理错误: {e}")
            traceback.print_exc()
            return None, None

    def undistortion(self, image, camera_matrix, dist_coeff):
        """消除图像畸变
        
        参数:
            image (ndarray): 输入图像
            camera_matrix (ndarray): 相机内参矩阵
            dist_coeff (ndarray): 畸变系数
            
        返回值:
            ndarray: 校正后的图像
        """
        try:
            h, w = image.shape[:2]
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                camera_matrix, dist_coeff, (w, h), 1, (w, h)
            )
            undistorted = cv2.undistort(image, camera_matrix, dist_coeff, None, new_camera_matrix)

            x, y, w, h = roi
            if w > 0 and h > 0:
                undistorted = undistorted[y:y + h, x:x + w]

            return undistorted
            
        except Exception as e:
            self.get_logger().error(f"畸变校正错误: {e}")
            traceback.print_exc()
            return image

    def preprocess(self, img1, img2):
        """图像预处理
        
        参数:
            img1 (ndarray): 左目图像
            img2 (ndarray): 右目图像
            
        返回值:
            tuple: (处理后的左目图像, 处理后的右目图像)
        """
        try:
            # 转换为灰度图
            if img1.ndim == 3:
                img1_gray = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            else:
                img1_gray = img1.copy()

            if img2.ndim == 3:
                img2_gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            else:
                img2_gray = img2.copy()

            # 应用CLAHE增强对比度
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            img1_eq = clahe.apply(img1_gray)
            img2_eq = clahe.apply(img2_gray)

            return img1_eq, img2_eq
            
        except Exception as e:
            self.get_logger().error(f"图像预处理错误: {e}")
            traceback.print_exc()
            return img1, img2

    def rectify_image(self, image1, image2, map1x, map1y, map2x, map2y):
        """对图像应用畸变校正和立体校正
        
        参数:
            image1 (ndarray): 左目图像
            image2 (ndarray): 右目图像
            map1x, map1y, map2x, map2y: 校正映射矩阵
            
        返回值:
            tuple: (校正后的左目图像, 校正后的右目图像)
        """
        try:
            rectified_img1 = cv2.remap(image1, map1x, map1y, cv2.INTER_AREA)
            rectified_img2 = cv2.remap(image2, map2x, map2y, cv2.INTER_AREA)
            return rectified_img1, rectified_img2
            
        except Exception as e:
            self.get_logger().error(f"图像校正错误: {e}")
            traceback.print_exc()
            return image1, image2

    def stereo_match_sgbm(self, left_image, right_image):
        """使用SGBM算法计算视差图
        
        参数:
            left_image (ndarray): 左目图像
            right_image (ndarray): 右目图像
            
        返回值:
            ndarray: 视差图
        """
        try:
            # 确保输入图像是灰度图
            if left_image.ndim != 2:
                left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            if right_image.ndim != 2:
                right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

            # 创建SGBM匹配器
            left_matcher = cv2.StereoSGBM_create(
                minDisparity=self.config.minDisparity,
                numDisparities=self.config.numDisparities,
                blockSize=self.config.blockSize,
                P1=self.config.P1,
                P2=self.config.P2,
                disp12MaxDiff=self.config.disp12MaxDiff,
                preFilterCap=self.config.preFilterCap,
                uniquenessRatio=self.config.uniquenessRatio,
                speckleWindowSize=self.config.speckleWindowSize,
                speckleRange=self.config.speckleRange,
                mode=self.config.mode
            )

            # 计算视差图
            disparity = left_matcher.compute(left_image, right_image)
            disparity = disparity.astype(np.float32) / 16.0

            # 过滤小视差值
            min_valid_disp = 1.0
            disparity[disparity < min_valid_disp] = 0

            return disparity
            
        except Exception as e:
            self.get_logger().error(f"立体匹配错误: {e}")
            traceback.print_exc()
            return None

    def reproject_to_3d(self, disparity, Q):
        """将视差图转换为3D点云
        
        参数:
            disparity (ndarray): 视差图
            Q (ndarray): 重投影矩阵
            
        返回值:
            ndarray: 3D点云数据
        """
        try:
            if disparity is None or Q is None:
                return None
                
            # 过滤太小的视差值
            filtered_disp = disparity.copy()
            min_disparity = 1.0
            filtered_disp[filtered_disp < min_disparity] = 0

            # 使用OpenCV的reprojectImageTo3D进行重投影
            points_3d = cv2.reprojectImageTo3D(filtered_disp, Q)

            # 过滤深度值异常的点
            max_depth = self.config.max_distance_mm
            mask = (points_3d[:, :, 2] > 0) & (points_3d[:, :, 2] < max_depth)
            points_3d[~mask] = [0, 0, 0]

            return points_3d
            
        except Exception as e:
            self.get_logger().error(f"3D重投影错误: {e}")
            traceback.print_exc()
            return None

    def publish_left_image(self, image):
        """发布左目图像
        
        参数:
            image (ndarray): 左目图像
            
        返回值:
            无
        """
        try:
            if image is not None:
                # 确保图像是彩色的，用于ROS消息
                if image.ndim == 2:
                    image_color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                else:
                    image_color = image
                    
                msg = self.bridge.cv2_to_imgmsg(image_color, "bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "stereo_left"
                self.left_image_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"图像发布错误: {e}")
            traceback.print_exc()

    def get_distance_callback(self, request, response):
        """距离测量服务回调函数
        
        参数:
            request: 服务请求，包含x, y坐标
            response: 服务响应对象
            
        返回值:
            response: 包含距离信息的响应
        """
        try:
            with self.frame_lock:
                points_3d = self.current_points_3d

            if points_3d is None:
                response.success = False
                response.distance = 0.0
                response.message = "3D点云数据不可用"
                return response

            # 测量指定点的距离
            distance = self.measure_distance(points_3d, request.x, request.y)

            if distance is not None:
                response.success = True
                response.distance = distance
                response.message = f"成功测量距离: {distance:.2f}米"
                self.get_logger().info(f"测量点({request.x}, {request.y})距离: {distance:.2f}米")
            else:
                response.success = False
                response.distance = 0.0
                response.message = "无法测量该点距离，可能是无效点"

            return response
            
        except Exception as e:
            self.get_logger().error(f"距离服务回调错误: {e}")
            traceback.print_exc()
            response.success = False
            response.distance = 0.0
            response.message = f"服务处理错误: {str(e)}"
            return response

    def measure_distance(self, points_3d, x, y):
        """测量指定像素点到相机的距离
        
        参数:
            points_3d (ndarray): 3D点云数据
            x (int): 像素x坐标
            y (int): 像素y坐标
            
        返回值:
            float: 距离值（米），如果无效则返回None
        """
        try:
            h, w = points_3d.shape[:2]

            # 检查坐标是否在有效范围内
            if not (0 <= x < w and 0 <= y < h):
                return None

            # 获取点的3D坐标
            point_3d = points_3d[y, x]

            # 检查点的有效性
            if np.all(np.isfinite(point_3d)) and not np.all(point_3d == 0):
                # 计算欧几里得距离
                distance = np.sqrt(np.sum(point_3d ** 2))
                return distance / 1000.0  # 转换为米

            return None
            
        except Exception as e:
            self.get_logger().error(f"距离测量错误: {e}")
            traceback.print_exc()
            return None

    def destroy_node(self):
        """节点销毁时的清理工作
        
        参数:
            无
            
        返回值:
            无
        """
        try:
            self.running = False
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
            self.get_logger().info('双目立体视觉节点已关闭')
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
        node = StereoVisionNode()
        
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