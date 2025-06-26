# 双目立体视觉机器人系统

## 概述
本系统基于ROS2开发，实现了双目立体视觉功能，包括：
- 实时双目图像处理
- 立体匹配和深度估计
- 距离测量服务
- 图像发布功能

## 功能特性
- 发布左目图像到 `/stereo/left/image` 话题
- 提供距离测量服务 `/stereo/get_distance`
- 支持实时立体匹配算法（SGBM）
- 完整的错误处理和日志记录
- 鼠标点击测距功能

## 系统要求
- ROS2 (推荐Humble)
- OpenCV
- Python 3.8+
- cv_bridge
- 双目相机设备

## 安装和编译

### 1. 编译自定义消息包
```bash
cd ~/SelfFollowingROS2/SelfFollowingROS2
colcon build --packages-select custom_msgs
source install/setup.bash
```

### 2. 编译following_robot包
```bash
colcon build --packages-select following_robot
source install/setup.bash
```

### 3. 编译所有包
```bash
colcon build
source install/setup.bash
```

## 使用方法

### 启动双目视觉节点
```bash
# 使用launch文件启动
ros2 launch following_robot stereo_vision.launch.py

# 或者直接运行节点
ros2 run following_robot stereo_vision_node
```

### 启动测试程序
```bash
# 在另一个终端运行测试程序
ros2 run following_robot simple_test.py
```

### 服务调用示例
```bash
# 测量点(320, 240)的距离
ros2 service call /stereo/get_distance custom_msgs/srv/GetDistance "{x: 320, y: 240}"
```

### 查看话题和服务
```bash
# 查看图像话题
ros2 topic echo /stereo/left/image

# 查看可用服务
ros2 service list

# 查看服务接口
ros2 service type /stereo/get_distance
```

## 配置参数

### Launch文件参数
- `camera_id`: 相机设备ID (默认: 1)
- `frame_width`: 图像宽度 (默认: 1280)
- `frame_height`: 图像高度 (默认: 480)  
- `fps_limit`: FPS限制 (默认: 30)

### 相机校准参数
系统使用预设的相机内参和外参，如需修改请编辑 `stereo_vision_node.py` 中的 `StereoCamera` 类。

## API说明

### 发布的话题
- `/stereo/left/image` (sensor_msgs/Image): 左目相机图像

### 提供的服务
- `/stereo/get_distance` (custom_msgs/GetDistance): 距离测量服务
  - 请求: x (int32), y (int32) - 像素坐标
  - 响应: success (bool), distance (float64), message (string)

## 测试程序功能
- 显示实时左目图像
- 画面中心显示绿色十字线
- 鼠标点击任意位置测量距离
- 每2秒自动测量中心点距离
- 按'q'键退出程序

## 故障排除

### 相机无法打开
1. 检查相机连接
2. 确认相机设备ID正确
3. 检查相机权限设置

### 服务调用失败
1. 确认双目视觉节点正在运行
2. 检查服务是否注册成功: `ros2 service list`
3. 查看节点日志输出

### 距离测量不准确
1. 检查相机校准参数
2. 确认双目相机基线距离设置正确
3. 验证立体匹配算法参数

## 开发说明
- 所有函数都包含完整的错误处理和traceback
- 代码遵循PEP8规范
- 使用中文注释和日志输出
- 支持多线程处理

## 许可证
TODO: 添加许可证信息 