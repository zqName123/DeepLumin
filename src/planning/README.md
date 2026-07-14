# 规划模块 (planning)

## 模块简介

规划模块负责生成井下车辆的行驶路径，包括全局路径规划和局部避障规划。

## 功能组件

### 1. 全局规划器 (global_planner)
- 基于A*算法的路径搜索
- 地图预处理
- 路径可行性检查

### 2. 局部规划器 (local_planner)
- 基于DWA的局部避障
- 动态窗口采样
- 轨迹评价与选择

### 3. 路径平滑 (path_smoother)
- B样条路径平滑
- 曲率约束优化
- 速度规划

## ROS话题

### 订阅话题
| 话题名称 | 类型 | 说明 |
|---------|------|------|
| /map | nav_msgs/OccupancyGrid | 地图数据 |
| /amcl_pose | geometry_msgs/PoseWithCovarianceStamped | 定位结果 |
| /obstacles | perception/ObstacleArray | 障碍物信息 |

### 发布话题
| 话题名称 | 类型 | 说明 |
|---------|------|------|
| /global_path | nav_msgs/Path | 全局路径 |
| /local_path | nav_msgs/Path | 局部路径 |
| /cmd_vel | geometry_msgs/Twist | 速度控制指令 |

## 配置文件

```
config/
├── global_planner.yaml    # 全局规划参数配置
├── local_planner.yaml     # 局部规划参数配置
└── path_smoother.yaml     # 路径平滑参数配置
```

## 启动命令

```bash
# 启动全局规划器
roslaunch planning global_planner.launch

# 启动局部规划器
roslaunch planning local_planner.launch

# 启动路径平滑
roslaunch planning path_smoother.launch

# 启动所有规划节点
roslaunch planning planning.launch
```
