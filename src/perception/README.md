# 感知模块 (perception)

## 模块简介

感知模块负责处理激光雷达、摄像头等传感器数据，实现井下环境的障碍物检测与识别。

## 功能组件

### 1. 点云预处理 (pointcloud_preprocessing)
- 点云去噪与滤波
- 地面点分割
- 点云配准与融合

### 2. 激光雷达障碍物检测 (lidar_obstacle_detection)
- 基于聚类的障碍物检测
- 障碍物位置与尺寸估计
- 动态障碍物追踪

### 3. 摄像头障碍物检测 (camera_obstacle_detection)
- 图像目标检测
- 语义分割
- 深度估计

## ROS话题

### 订阅话题
| 话题名称 | 类型 | 说明 |
|---------|------|------|
| /points_raw | sensor_msgs/PointCloud2 | 原始激光点云 |
| /image_raw | sensor_msgs/Image | 原始图像数据 |

### 发布话题
| 话题名称 | 类型 | 说明 |
|---------|------|------|
| /points_filtered | sensor_msgs/PointCloud2 | 滤波后点云 |
| /obstacles | perception/ObstacleArray | 障碍物检测结果 |
| /obstacle_markers | visualization_msgs/MarkerArray | 障碍物可视化标记 |

## 配置文件

```
config/
├── lidar_config.yaml      # 激光雷达参数配置
├── camera_config.yaml     # 摄像头参数配置
└── obstacle_config.yaml   # 障碍物检测参数配置
```

## 启动命令

```bash
# 启动点云预处理
roslaunch perception pointcloud_preprocessing.launch

# 启动激光雷达障碍物检测
roslaunch perception lidar_obstacle_detection.launch

# 启动摄像头障碍物检测
roslaunch perception camera_obstacle_detection.launch

# 启动所有感知节点
roslaunch perception perception.launch
```
