# 仿真模块 (simulation)

## 模块简介

仿真模块基于Gazebo构建井下环境与车辆仿真系统，用于算法验证和测试。

## 功能组件

### 1. 仿真控制器 (simulation_control)
- Gazebo仿真管理
- 仿真场景加载
- 仿真数据采集

## 文件结构

```
urdf/                        # URDF模型文件
└── vehicle.urdf             # 车辆URDF模型

worlds/                      # Gazebo世界文件
└── mine_world.world         # 井下环境世界模型

models/                      # Gazebo模型文件
├── mine_tunnel/             # 矿井隧道模型
└── obstacles/               # 障碍物模型

launch/                      # 启动文件
├── mine_environment.launch  # 井下环境启动
└── vehicle_model.launch     # 车辆模型启动
```

## ROS话题

### 订阅话题
| 话题名称 | 类型 | 说明 |
|---------|------|------|
| /cmd_vel | geometry_msgs/Twist | 控制指令 |

### 发布话题
| 话题名称 | 类型 | 说明 |
|---------|------|------|
| /scan | sensor_msgs/LaserScan | 仿真激光雷达数据 |
| /points_raw | sensor_msgs/PointCloud2 | 仿真点云数据 |
| /image_raw | sensor_msgs/Image | 仿真图像数据 |
| /odom | nav_msgs/Odometry | 仿真里程计数据 |

## 配置文件

```
config/
├── gazebo_config.yaml      # Gazebo仿真参数配置
├── sensor_config.yaml      # 传感器仿真参数配置
└── vehicle_config.yaml     # 车辆仿真参数配置
```

## 启动命令

```bash
# 启动井下环境仿真
roslaunch simulation mine_environment.launch

# 启动车辆模型
roslaunch simulation vehicle_model.launch

# 启动完整仿真系统
roslaunch simulation simulation.launch
```
