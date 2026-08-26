# localization_v2

新一代定位融合节点：基于 `ysw_style_fusion` 的 `map->odom` 修正思路，
融合 `slam_odometry_v2` 局部里程计与 `global_matcher` 全局匹配结果，
输出平滑且全局一致的 `map->base_link` 位姿，并预留 `relocalization`
重定位初始化接口。

## 一、数据流与坐标关系

```
map (全局地图坐标系)
  └── odom (SLAM 局部坐标系，经 map->odom 修正)
       └── base_link (车体坐标系)
```

```text
slam_odometry_v2 ── /localization/slam_odom (odom->base_link)
global_matcher ──── /localization/global_matcher/result (map->odom)
relocalization ──── /localization/relocalization/result (map->base_link，启动初值)
/initialpose ────── 人工初始位姿
        │
        ▼
   localization_v2
        │
        ├── /localization_v2/odom        (map->base_link 融合里程计)
        ├── /localization_v2/map_to_odom (修正量)
        ├── /localization_v2/path
        ├── /localization_v2/status
        └── TF map->odom
```

融合公式（与 ysw_style_fusion 一致）：

```text
map_odom = map_base ⊕ (odom_base)⁻¹   （GM / 人工初值 / 重定位更新）
map_base = map_odom ⊕ odom_base        （高频输出）
```

## 二、启动初始化状态机（含重定位接口）

状态：`UNINITIALIZED → RELOCALIZING → TRACKING`

- 算法开始、收到第一条 slam 里程计后，若未初始化且
  `relocalization.enabled && auto_start`，自动发布 `start` 命令请求重定位；
- 满足以下任一条件即完成初始化并发布 `stop` 命令停止重定位：
  1. 重定位结果成功且通过质量门限（`max_fitness_score` /
     `min_inlier_ratio` / `max_inlier_rmse`，可选 `require_accepted`）；
  2. 人工给出 `/initialpose`；
  3. `global_matcher` 自身匹配成功（`use_global_matcher_init`）。

`initialization.require_initial_pose=false` 时保持 ysw 原始行为：
无初始位姿也能以单位 `map->odom` 输出，首个 GM 结果到达后自动修正；
置 `true` 时初始化完成前不发布融合里程计。

### 重定位接口（解耦点）

`IRelocalizationClient`（`include/localization_v2/interface/`）是节点与重定位
模块的抽象接口，节点只依赖该接口：

```text
IRelocalizationClient
  ├── start() / stop()
  └── setResultCallback(RelocalizationSignal)
```

默认实现 `RelocalizationRosAdapter`：

- 订阅 `/localization/relocalization/result`
  （`deeplumin_msgs/RelocalizationResult`，`map->base_link`）；
- 发布 `/localization_v2/relocalization/command`
  （`std_msgs/String`，`"start"` / `"stop"`）。

后续若重定位模块改为 service / action 协议，只需新增一个
`IRelocalizationClient` 实现并在节点构造处替换注入，
融合逻辑与状态机无需任何改动。

## 三、运行

```bash
# 完整链路（slam_odometry_v2 + global_matcher + localization_v2）
roslaunch localization_v2 localization_v2.launch

# 带重定位初值来源（需先按实际传感器校准 relocalization.yaml）
roslaunch localization_v2 localization_v2.launch start_relocalization:=true
```

## 四、目录结构

```text
localization_v2/
├── CMakeLists.txt
├── package.xml
├── config/localization_v2.yaml      # 唯一配置文件
├── include/localization_v2/
│   ├── core/                        # 纯 C++：融合核心 + 初始化状态机
│   │   ├── types.hpp
│   │   ├── fusion_core.hpp
│   │   └── initialization.hpp
│   ├── interface/
│   │   └── relocalization_interface.hpp   # 重定位抽象接口
│   └── ros/
│       ├── localization_v2_config.hpp
│       └── relocalization_ros_adapter.hpp # 重定位默认 ROS 实现
├── src/
│   ├── core/                        # 核心实现（不依赖 ROS 消息）
│   └── ros/
│       ├── localization_v2_config.cpp
│       ├── relocalization_ros_adapter.cpp
│       └── localization_v2_node.cpp # 节点入口
└── launch/localization_v2.launch
```

## 五、设计说明

- **解耦**：core（融合/状态机）不依赖 ROS 消息，ROS 只在 `ros/` 层；
- **无冗余话题**：`odom->base` TF 由 `slam_odometry_v2` 发布，本节点只发
  `map->odom`；全局地图可视化由 `global_matcher` 发布，本节点不再重复加载；
- **配置单一来源**：所有可调参数集中在 `config/localization_v2.yaml`，
  launch 只做话题/路径等部署级覆盖。
