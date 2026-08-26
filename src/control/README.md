# 控制模块 (control)

## 模块简介

控制模块负责将规划模块生成的路径和速度指令转换为车辆执行器的控制信号，实现车辆的精确运动控制。

## 功能组件

### 1. 运动控制器 (motion_controller)
- 横向控制器
- 纵向控制器
- 车辆动力学模型

### 2. 转向控制 (steering_control)
- 转向角计算
- 转向执行器驱动
- 转向限位保护

### 3. 速度控制 (speed_control)
- 速度闭环控制
- 加速度限制
- 速度平滑处理

## ROS话题

### 订阅话题
| 话题名称 | 类型 | 说明 |
|---------|------|------|
| /local_path | nav_msgs/Path | 局部路径 |
| /cmd_vel | geometry_msgs/Twist | 速度指令 |
| /odom | nav_msgs/Odometry | 里程计反馈 |

### 发布话题
| 话题名称 | 类型 | 说明 |
|---------|------|------|
| /steering_angle | std_msgs/Float64 | 转向角指令 |
| /speed_command | std_msgs/Float64 | 速度指令 |
| /control_status | control/ControlStatus | 控制状态 |

## 配置文件

```
config/
├── motion_controller.yaml  # 运动控制器参数配置
├── steering_control.yaml   # 转向控制参数配置
└── speed_control.yaml      # 速度控制参数配置
```

## 启动命令

```bash
# 启动运动控制器
roslaunch control motion_controller.launch

# 启动转向控制
roslaunch control steering_control.launch

# 启动速度控制
roslaunch control speed_control.launch

# 启动所有控制节点
roslaunch control control.launch
```
