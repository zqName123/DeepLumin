# simple_odom_fusion

> 多源里程计融合包 —— 融合 DR（航位推算）、SLAM 里程计与全局定位结果，输出平滑且全局一致的 `map->base_link` 位姿。

---

## 一、功能概述

本包提供两种融合节点，用于解决自动驾驶/机器人定位中的核心问题：

| 问题 | 解决方案 |
|---|---|
| DR 高频但长期漂移 | 用 SLAM 增量修正 DR 预测 |
| SLAM 局部一致但会漂移 | 用全局定位（Global Matcher）修正 `map->odom` |
| 全局定位低频且有跳变 | 通过增益系数平滑修正，避免位姿突变 |
| 多源数据时间不同步 | 内置时间同步队列，自动配对最近时刻数据 |

**输出**：高频（可配置至 100Hz+）、平滑、全局一致的 `map->base_link` 位姿。

---

## 二、两个节点的对比

| 特性 | `simple_odom_fusion_node` | `ysw_style_fusion_node` |
|---|---|---|
| **输入里程计** | DR + SLAM（双输入） | 仅 SLAM（单输入） |
| **融合策略** | DR 预测 + SLAM 增量修正 | 直接 `map_odom = map_base ⊕ odom_base⁻¹` |
| **时间同步** | ✅ 内置 `TimeSyncQueue` | ❌ 直接取最新 SLAM 数据 |
| **复杂度** | 较高（支持多种融合模式） | 简单（类似 Cartographer 风格） |
| **适用场景** | 需要 DR 高频平滑 + SLAM 修正 | 已有高质量 SLAM，仅需全局定位修正 |
| **初始化方式** | 全局定位 / 局部自动 / 初始位姿 | 全局定位 / 初始位姿 |

---

## 三、坐标系定义

```
map (全局地图坐标系，不漂移)
  └── odom (融合后的局部坐标系，通过 map->odom 修正)
       └── base_link (机器人本体坐标系)
            └── lidar_link (激光雷达坐标系)
```

**关键位姿**：
- `odom_base`：`odom -> base_link`（来自 DR 或 SLAM 的局部里程计）
- `map_odom`：`map -> odom`（全局定位修正量，低频更新）
- `map_base`：`map -> base_link`（最终融合输出）

**融合公式**：
```
map_base = map_odom ⊕ odom_base
```

---

## 四、算法原理

### 4.1 simple_odom_fusion_node 核心算法

#### 时间同步（TimeSyncQueue）

```cpp
// DR 队列和 SLAM 队列按时间戳配对
// 容差 tolerance 内（默认 0.05s）认为可配对
// 时间差过大的样本会被丢弃
```

**流程**：
1. DR 和 SLAM 数据分别入队
2. 每次 `popSynced()` 取队首比较时间戳
3. `|t_dr - t_slam| <= tolerance` → 配对成功，返回 `SyncedPair`
4. 否则丢弃时间较早的一侧，继续比较

#### 局部传播（propagateLocal）

```cpp
// 1. 计算 DR 增量
dr_delta = between(last_dr.pose, current_dr.pose)

// 2. DR 预测（基于上一时刻的 odom_base）
dr_pred = blend(previous_odom_base, compose(previous_odom_base, dr_delta), dr_prediction_weight)

// 3. 若启用 SLAM 增量模式
if (use_slam_increment) {
    slam_delta = between(last_slam.pose, current_slam.pose)
    slam_pred = compose(previous_odom_base, slam_delta)
    fused_pred = blend(dr_pred, slam_pred, slam_correction_gain)
} else {
    fused_pred = dr_pred
}

// 4. 更新状态
odom_base = fused_pred
```

**增量模式 vs 绝对模式**：
- `use_slam_increment = true`：使用 SLAM 的相对增量修正 DR 预测（推荐，对 SLAM 跳变更鲁棒）
- `use_slam_increment = false`：直接使用 SLAM 绝对位姿（SLAM 质量高时可用）

#### 全局定位修正（correctWithGlobalMatcher）

```cpp
// 观测到的 map->odom
observed_map_odom = compose(map_base_observed, inverse(odom_base))

// 平滑修正（一阶低通滤波）
map_odom = blend(map_odom, observed_map_odom, correction_gain)
```

**修正增益 `correction_gain`**：
- 值越小（如 0.05）：修正越平滑，但收敛慢
- 值越大（如 0.3）：修正越快，但可能有跳变
- 默认 0.15，兼顾平滑和收敛速度

#### 异常检测（validDelta）

```cpp
// 检查两次更新间的位移和旋转是否异常
if (delta_translation > max_delta_translation)  → 丢弃（默认 8m）
if (delta_rotation > max_delta_rotation_rad)    → 丢弃（默认 1.2rad ≈ 68°）
```

用于过滤传感器故障或数据异常导致的位姿跳变。

### 4.2 ysw_style_fusion_node 核心算法

```cpp
// 直接计算 map->odom 修正量
map_odom = map_base_global ⊕ inverse(odom_base_slam)

// 融合输出
map_base = compose(map_odom, odom_base_slam)
```

**特点**：
- 无 DR 输入，直接依赖 SLAM 的 `odom->base_link`
- 全局定位更新 `map->odom` 时，SLAM 继续输出平滑的局部位姿
- 即使全局定位暂时失效，SLAM 仍可继续输出相对位姿

---

## 五、文件结构

```
simple_odom_fusion/
├── CMakeLists.txt                          # 构建配置
├── package.xml                             # ROS 包配置
├── config/
│   └── simple_odom_fusion.yaml             # 主配置文件
├── include/simple_odom_fusion/
│   └── core/
│       ├── math_utils.hpp                  # 位姿数学工具（compose/inverse/between/interpolate/blend）
│       ├── types.hpp                       # 数据结构（Pose3d/OdomSample/SyncedPair/FusionConfig/...）
│       ├── time_sync_queue.hpp             # 时间同步队列（DR/SLAM 配对）
│       └── simple_fusion_core.hpp          # 融合核心接口
├── src/
│   ├── core/
│   │   └── simple_fusion_core.cpp          # 融合核心实现（初始化/同步/传播/修正/输出）
│   └── ros/
│       ├── simple_odom_fusion_node.cpp     # DR+SLAM 融合节点（复杂模式）
│       └── ysw_style_fusion_node.cpp       # SLAM-only 融合节点（简单模式）
├── launch/
│   ├── simple_odom_fusion.launch           # 复杂模式启动文件
│   └── ysw_style_fusion.launch             # 简单模式启动文件
└── rviz/
    └── simple_odom_fusion.rviz             # RViz 可视化配置
```

---

## 六、配置参数详解

### 6.1 坐标系配置（frames）

```yaml
frames:
  map: map                    # 全局地图坐标系
  odom: odom                  # 融合后的局部坐标系
  base_link: base_link        # 机器人本体坐标系
  lidar: lidar_link           # 激光雷达坐标系
```

### 6.2 话题配置（topics）

```yaml
topics:
  dr_odom: /localization/dr_odom                          # DR 里程计输入
  slam_odom: /localization/slam_odom                      # SLAM 里程计输入
  global_matcher_result: /localization/global_matcher/result  # 全局定位结果
  initial_pose: /initialpose                              # RViz 初始位姿
  source_cloud: /localization/cloud_registered            # 源点云（用于转发）
  fused_odom: /localization/simple_fused_odom             # 融合里程计输出
  fused_pose: /localization/simple_fused_pose             # 融合位姿输出
  fused_path: /localization/simple_fused_path             # 融合轨迹输出
  current_scan_in_map: /localization/ysw_style/current_scan_in_map  # 当前点云在map系（可视化）
```

### 6.3 运行时配置（runtime）

```yaml
runtime:
  output_rate_hz: 100.0       # 输出频率（Hz），定时器周期 = 1/rate
  publish_tf: true            # 是否发布 TF 变换
  publish_map_to_odom_tf: true    # 是否发布 map->odom 的 TF
  publish_odom_to_base_tf: true   # 是否发布 odom->base_link 的 TF
  publish_path: true          # 是否发布历史轨迹
  max_path_size: 5000         # 轨迹最大点数（滑动窗口，超限时移除旧点）
```

### 6.4 时间同步配置（sync）

```yaml
sync:
  max_queue_size: 300         # DR/SLAM 队列最大长度
  tolerance: 0.05             # 时间同步容差（秒），在此窗口内配对
  max_pair_dt: 0.20           # 配对的最大允许时间差，超过则丢弃
```

### 6.5 融合算法配置（fusion）

```yaml
fusion:
  max_delta_translation: 8.0      # 异常检测：最大允许位移（米）
  max_delta_rotation_rad: 1.2     # 异常检测：最大允许旋转（弧度）
  dr_prediction_weight: 1.0       # DR 预测权重（0-1）
  slam_correction_gain: 0.25      # SLAM 修正增益（0-1）
  use_slam_increment: true        # 是否使用 SLAM 增量模式
```

**参数调优建议**：

| 场景 | dr_prediction_weight | slam_correction_gain | use_slam_increment |
|---|---|---|---|
| DR 质量高，SLAM 偶尔跳变 | 0.9 | 0.1 | true |
| SLAM 质量高，DR 漂移大 | 0.3 | 0.7 | false |
| 平衡模式（默认） | 1.0 | 0.25 | true |

### 6.6 初始化配置（initialization）

```yaml
initialization:
  allow_initialpose: true                   # 允许通过 /initialpose 重置
  allow_global_matcher: true                # 允许通过全局定位结果初始化
  allow_local_auto: true                    # 允许局部自动初始化
```

**初始化优先级**：
1. 全局定位结果（若 `allow_global_matcher = true`）
2. 局部自动初始化（若 `allow_local_auto = true`，静止检测）
3. 初始位姿（若 `allow_initialpose = true`，人工设置）

### 6.7 全局定位配置（global_matcher）

```yaml
global_matcher:
  enable_correction: true           # 是否启用全局定位修正
  correction_gain: 0.15             # 修正增益（0-1，越小越平滑）
  min_interval: 1.0                 # 两次修正的最小间隔（秒）
  publish_current_scan_in_map: true   # 是否发布当前点云在map系的可视化
  cloud_use_fused_pose: true        # 转发时是否使用融合位姿变换
```

---

## 七、启动方式

### 7.1 简单模式（YSL 风格，仅 SLAM）

```bash
roslaunch simple_odom_fusion ysw_style_fusion.launch
```

**输入**：
- `/localization/slam_odom`：SLAM 里程计（`odom->base_link`）
- `/localization/global_matcher/result`：全局定位结果（`map->base_link`）

**输出**：
- `/localization/ysw_style_odom`：融合里程计（`map->base_link`）
- `/tf`：`map->odom` 和 `odom->base_link`

### 7.2 复杂模式（DR + SLAM）

```bash
roslaunch simple_odom_fusion simple_odom_fusion.launch
```

**输入**：
- `/localization/dr_odom`：DR 里程计（高频，如轮速计）
- `/localization/slam_odom`：SLAM 里程计（中频）
- `/localization/global_matcher/result`：全局定位结果

**输出**：
- `/localization/simple_fused_odom`：融合里程计
- `/tf`：`map->odom` 和 `odom->base_link`

---

## 八、数据流图

### simple_odom_fusion_node

```
┌─────────────┐     ┌─────────────┐     ┌─────────────────────┐
│  DR Odometry │────→│             │     │                     │
│  (100-500Hz) │     │  TimeSync   │────→│  SimpleFusionCore   │
└─────────────┘     │   Queue     │     │                     │
┌─────────────┐     │  (配对 DR   │     │  - propagateLocal() │
│ SLAM Odometry│────→│   + SLAM)  │     │  - correctWithGM()  │
│   (10-50Hz)  │     │             │     │  - output()         │
└─────────────┘     └─────────────┘     └─────────────────────┘
                                                 │
┌─────────────────────┐                          │
│ GlobalMatcherResult │─────────────────────────→│  (修正 map_odom)
│     (0.1-1Hz)       │                          │
└─────────────────────┘                          ▼
                                        ┌─────────────────────┐
                                        │   FusionOutput      │
                                        │  (map_base/odom_base│
                                        │   /map_odom/velocity)│
                                        └─────────────────────┘
                                                 │
                    ┌────────────────────────────┼────────────────────────────┐
                    ▼                            ▼                            ▼
            ┌──────────────┐           ┌──────────────┐              ┌──────────────┐
            │  fused_odom  │           │     TF       │              │    path      │
            │  fused_pose  │           │ map->odom    │              │   (RViz)     │
            └──────────────┘           │ odom->base   │              └──────────────┘
                                       └──────────────┘
```

### ysw_style_fusion_node

```
┌─────────────────┐
│  SLAM Odometry   │────────┐
│   (odom->base)   │        │
└─────────────────┘        │     ┌─────────────────────────┐
                           ├────→│   YswStyleFusionNode    │
┌─────────────────┐        │     │                         │
│ GlobalMatcher   │────────┘     │  map_odom = map_base    │
│  (map->base)    │              │           ⊕ inv(odom_base)│
└─────────────────┘              │                         │
                                 │  map_base = map_odom    │
                                 │           ⊕ odom_base   │
                                 └─────────────────────────┘
                                              │
                    ┌─────────────────────────┼─────────────────────────┐
                    ▼                         ▼                         ▼
            ┌──────────────┐          ┌──────────────┐          ┌──────────────┐
            │  fused_odom  │          │     TF       │          │    path      │
            │  map_to_odom │          │ map->odom    │          │   (RViz)     │
            └──────────────┘          │ odom->base   │          └──────────────┘
                                      └──────────────┘
```

---

## 九、调试与可视化

### 9.1 查看 TF 树

```bash
rosrun tf tf_echo map base_link      # 查看 map->base_link 变换
rosrun tf tf_echo map odom           # 查看 map->odom 修正量
rosrun tf tf_echo odom base_link     # 查看 odom->base_link 局部里程计
```

### 9.2 查看融合输出

```bash
rostopic echo /localization/simple_fused_odom    # 查看融合里程计
rostopic echo /localization/ysw_style_map_to_odom # 查看 map->odom 修正量
```

### 9.3 RViz 可视化

```bash
roslaunch simple_odom_fusion simple_odom_fusion.launch
rosrun rviz rviz -d $(rospack find simple_odom_fusion)/rviz/simple_odom_fusion.rviz
```

**显示内容**：
- 融合轨迹（`/localization/simple_fused_path`）
- 当前位姿（TF：`map->base_link`）
- 点云（若配置了转发）

### 9.4 常见问题排查

| 现象 | 可能原因 | 解决方案 |
|---|---|---|
| 无融合输出 | DR/SLAM 未同步 | 检查时间戳是否对齐，调整 `sync/tolerance` |
| 位姿跳变 | 全局定位修正过快 | 减小 `global_matcher/correction_gain` |
| 轨迹漂移 | SLAM 增量模式未启用 | 设置 `fusion/use_slam_increment: true` |
| TF 缺失 | `publish_tf` 未启用 | 检查 `runtime/publish_tf` 参数 |
| 初始化失败 | 全局定位未成功 | 检查全局定位模块是否正常运行 |

---

## 十、扩展与定制

### 10.1 添加新的里程计源

1. 在 `types.hpp` 中定义新的 `OdomSample` 类型（或复用现有类型）
2. 在 `TimeSyncQueue` 中添加新的队列和配对逻辑
3. 在 `SimpleFusionCore` 中添加新的 `feedXxx()` 和 `propagateXxx()` 方法
4. 在 ROS 节点中添加订阅器和回调函数

### 10.2 替换融合算法

`SimpleFusionCore` 是纯算法类，不依赖 ROS：
- 修改 `propagateLocal()` 实现自定义融合策略
- 修改 `correctWithGlobalMatcher()` 实现自定义修正策略
- 保持 `FusionOutput` 输出格式不变即可兼容现有 ROS 节点

### 10.3 集成到其他框架

本包的核心算法（`SimpleFusionCore` + `TimeSyncQueue` + `math_utils`）可轻松移植到非 ROS 环境：
- 移除 ROS 消息转换层
- 保留纯 C++ 算法核心
- 适配新的通信框架（如 DDS、ZeroMQ 等）

---

## 十一、依赖

- **ROS1**：`roscpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `tf2_ros`
- **PCL**：点云处理与坐标变换
- **Eigen**：位姿数学运算
- **deeplumin_msgs**：全局定位结果消息类型

---

## 十二、参考

- [Cartographer 坐标系设计](https://google-cartographer.readthedocs.io/en/latest/)
- [ROS REP 105: Coordinate Frames for Mobile Platforms](https://www.ros.org/reps/rep-0105.html)
- [Gao et al., "LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping"](https://arxiv.org/abs/2007.00258)