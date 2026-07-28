# relocalization

基于 Scan Context 和 GICP 的全局重定位模块，为 DeepLumin 定位系统提供全局位姿估计。

## 功能特性

- **Scan Context 描述子**: 旋转不变的点云位置识别
- **GICP 精配准**: 基于 Generalized ICP 的精确位姿估计
- **子图匹配**: 支持基于半径或索引的局部子图构建
- **在线重定位**: 实时接收查询点云，输出全局位姿
- **离线评估**: 支持 bag 文件批量评估和可视化
- **多模式建库**: 支持单帧/子图模式构建数据库

## 依赖

- ROS Noetic
- PCL (Point Cloud Library)
- Eigen3
- OpenMP (可选，用于并行加速)
- deeplumin_msgs

## 编译

```bash
cd /home/hhy/2004ros/DeepLumin
catkin build relocalization
```

## 运行

### 1. 在线重定位

```bash
roslaunch relocalization global_relocalization.launch
```

### 2. Bag 评估

```bash
roslaunch relocalization bag_eval_viz.launch
```

### 3. 构建 Scan Context 数据库

```bash
# 单帧模式
rosrun relocalization build_sc_database

# 子图模式
roslaunch relocalization build_sc_submap_database.launch
```

## 参数配置

主要配置文件: `config/relocalization.yaml`

### 核心参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `descriptor/type` | 描述子类型 | `scan_context` |
| `matcher/type` | 匹配器类型 | `gicp` |
| `keyframe_dir` | 关键帧目录 | - |
| `pose_file` | 位姿文件 (TUM 格式) | - |
| `global_map_pcd` | 全局地图 PCD | - |
| `scan_context_db` | Scan Context 数据库 | - |

### Scan Context 参数

```yaml
scan_context:
  num_rings: 20          # 径向环数
  num_sectors: 60        # 角度扇区数
  max_radius: 100.0      # 最大半径 (m)
  min_range: 3.0         # 最小距离 (m)
  ring_key_candidates: 100  # 候选帧数量
```

### GICP 参数

```yaml
gicp:
  max_iterations: 100
  max_correspondence_distance: 10.5
  transformation_epsilon: 0.00001
  accept_fitness: 0.5
  min_inlier_ratio: 0.7
  max_inlier_rmse: 0.3
```

### 子图参数

```yaml
submap:
  mode: radius           # radius/index/hybrid
  radius: 20.0           # 子图半径 (m)
  neighbor_keyframes: 20 # 邻居关键帧数
  before_frames: 5       # 前方帧数
  after_frames: 5        # 后方帧数
```

## 输入输出

### 输入话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/localization/points` | `sensor_msgs/PointCloud2` | 查询点云 |

### 输出话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/localization/relocalization/result` | `deeplumin_msgs/RelocalizationResult` | 重定位结果 |

### 结果消息格式

```yaml
header:
  stamp: ...
  frame_id: "map"
child_frame_id: "base_link"
pose:                    # 估计位姿 (协方差表示不确定性)
  pose: ...
  covariance: ...
success: true/false      # 算法是否成功
accepted: true/false     # 是否通过质量门限
map_id: "default"
method: "scan_context_gicp"
score: 0.85              # 匹配得分
fitness_score: 0.85
inlier_ratio: 0.75
inlier_rmse: 0.15
```

## 节点说明

### global_relocalization_node

在线重定位主节点，实时处理查询点云并输出全局位姿。

```bash
rosrun relocalization global_relocalization_node
```

### bag_eval_node

Bag 文件评估节点，用于批量测试和评估重定位性能。

```bash
rosrun relocalization bag_eval_node
```

### relocalization_node

通用重定位节点，支持多种评估模式。

```bash
rosrun relocalization relocalization_node
```

## 架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    relocalization_node                           │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    RelocalizationCore                     │  │
│  │  - 关键帧数据库 (KeyframeDatabase)                         │  │
│  │  - Scan Context 描述子 (ScanContextDescriptor)             │  │
│  │  - GICP 匹配器 (GICPGlobalMatcher)                         │  │
│  └──────────────────────────────────────────────────────────┘  │
│                           │                                      │
│         ┌─────────────────┼─────────────────┐                   │
│         ▼                 ▼                 ▼                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │   Stage 1    │  │   Stage 2    │  │   Stage 3    │          │
│  │ Scan Context │→ │  Ring Key    │→ │    GICP      │          │
│  │  粗定位      │  │  候选筛选    │  │  精配准      │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
│                                                           │      │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │              GlobalRelocalizationNode                     │  │
│  │  - ROS 接口封装                                           │  │
│  │  - 结果发布                                               │  │
│  │  - 可视化                                                 │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## 工作流程

### 在线重定位流程

1. **接收查询点云** → 预处理和降采样
2. **Scan Context 描述** → 生成查询描述子
3. **Ring Key 匹配** → 从数据库找出候选帧
4. **子图构建** → 拼接候选帧附近点云
5. **GICP 精配准** → 计算精确变换
6. **质量验证** → 检查内点率和 RMSE
7. **发布结果** → 输出位姿和置信度

### 离线评估流程

1. 加载 bag 文件和真值位姿
2. 按时间戳采样查询帧
3. 执行重定位算法
4. 计算与真值的误差
5. 生成评估报告 (CSV)

## 数据库构建

### 1. 准备数据

```bash
# 关键帧目录结构
key_point_frame/
  ├── 000000.pcd
  ├── 000001.pcd
  └── ...

# 位姿文件 (TUM 格式)
# timestamp x y z qx qy qz qw
optimized_poses_tum.txt
```

### 2. 构建数据库

```bash
# 子图模式 (推荐)
roslaunch relocalization build_sc_submap_database.launch

# 参数配置
sc_submap:
  mode: index
  before_frames: 3
  after_frames: 10
```

### 3. 验证数据库

```bash
# 检查生成的 .bin 文件
ls -lh scan_context_submap_db_*.bin
```

## 调试

### RViz 配置

```bash
roslaunch relocalization bag_eval_viz.launch
```

显示内容:
- 查询点云 (红色)
- 配准后点云 (绿色)
- 目标子图 (蓝色)
- 真值位姿 (黄色箭头)

### 常见问题

1. **重定位失败率高**
   - 检查建库参数与查询参数是否一致
   - 调整 `min_inlier_ratio` 和 `max_inlier_rmse`
   - 增大 `top_k` 候选帧数量

2. **GICP 不收敛**
   - 检查初始位姿是否偏差过大
   - 调整 `max_correspondence_distance`
   - 启用/禁用 `z_filter_enable`

3. **数据库加载失败**
   - 检查 `scan_context_db` 路径
   - 确认参数与数据库一致 (`num_rings`, `num_sectors`, `max_radius`)

### 性能优化

1. **启用 OpenMP**
   ```cmake
   # CMakeLists.txt
   find_package(OpenMP)
   if(OpenMP_FOUND)
     target_compile_options(... PRIVATE ${OpenMP_CXX_FLAGS})
   endif()
   ```

2. **调整候选帧数量**
   - 减小 `ring_key_candidates` 提高速度
   - 增大 `ring_key_candidates` 提高召回率

3. **子图参数调优**
   - 减小 `neighbor_keyframes` 降低内存占用
   - 使用 `hybrid` 模式平衡精度和速度

## 评估指标

| 指标 | 说明 | 目标值 |
|------|------|--------|
| Success Rate | 重定位成功率 | > 95% |
| Translation Error | 平移误差 (m) | < 0.5m |
| Yaw Error | 航向角误差 (deg) | < 3° |
| Inlier Ratio | 内点比例 | > 0.7 |
| RMSE | 均方根误差 | < 0.3m |

## 维护者

- DeepLumin Team

## 许可证

Apache-2.0

## 参考

- [Scan Context](https://github.com/irapkaist/scancontext): 原始 Scan Context 实现
- [GICP](https://pointclouds.org/documentation/classpcl_1_1_generalized_iterative_closest_point.html): PCL GICP 文档