# localization_v3

定位融合节点：融合 `slam_odometry_v2` 局部里程计与 `global_matcher_v2` 全局匹配
（`map->odom` 修正思路），输出平滑且全局一致的 `map->base_link` 位姿，
支持重定位初始化与跟踪丢失恢复。

设计文档：`docs/localization_v3_optimization_plan.md`（问题清单、机制设计、参数说明）。

## 一、数据流与坐标关系

```
map (全局地图坐标系)
  └── odom (SLAM 局部坐标系，经 map->odom 修正)
       └── base_link (车体坐标系)
```

```text
slam_odometry_v2 ── /localization/slam_odom (odom->base_link)
global_matcher_v2 ── /localization/global_matcher/result (map->odom)
relocalization ──── /localization/relocalization/result (map->base_link)
/initialpose ────── 人工初始位姿（最高优先级）
        │
        ▼
   localization_v3
        │
        ├── /localization_v2/odom        (map->base_link 融合里程计)
        ├── /localization_v2/map_to_odom (修正量)
        ├── /localization_v2/status      (FusionStatus)
        ├── /localization_v2/loc_point   (规划位姿)
        ├── /localization_v2/gm_anchor   (通知 GM 重锚)
        ├── /localization_v2/relocalization/command (start/stop)
        └── TF map->odom, map->base_link
```

融合公式：
`map_odom = map_base ⊕ (odom_base)⁻¹`，`map_base = map_odom ⊕ odom_base`。

## 二、机制要点（一个阶段一个机制）

| 阶段 | 机制 |
|---|---|
| 人工位姿 | 无条件写入（**不经 jump guard**）→ settle 0.3s 滤过期结果 → 首个质量 good 的匹配结果验证通过并锚定 GM → 从该位姿继续跟踪 |
| 跟踪（TRACKING） | 质量三级分级 + **jump guard**（小跳直过 / 大跳 3 帧一致确认 / 超限拒绝）——防几何相似误匹配跳变 |
| 恢复（TRACKING_LOST / FAILURE） | **不用 jump guard**（恢复即大跳变）：质量门限 + 恢复确认窗口（GM 3 次 / reloc 2 次） |
| 失效判定 | 快计数 3 帧（≈0.3s）→ TRACKING_LOST；3s 未恢复 → LOCALIZATION_FAILURE；出图快路径 / 无修正看门狗 / SLAM 断流 |

状态机（五态）：`UNINITIALIZED → RELOCALIZING → TRACKING ⇄ TRACKING_LOST → LOCALIZATION_FAILURE`
人工位姿在任意状态生效并强制回 TRACKING。

## 三、模块结构

```
core/   types · fusion_core(位姿写入策略) · jump_guard · quality_gate
        failure_detector · state_machine · init_arbiter        （纯 C++，可单测）
ros/    localization_v3_node(薄编排) · config · status_publisher · relocalization_ros_adapter
config/localization_v3.yaml   （各参数含义见文件内注释）
test/   test_v3_fusion_core / test_v3_jump_guard / test_v3_state_machine / test_v3_failure_detector
```

## 四、构建与运行

```bash
cd /home/hhy/2004ros/DeepLumin_zq
catkin_make -DCMAKE_BUILD_TYPE=Release          # 或 --pkg localization_v3
source devel/setup.bash

# 运行整条链路
roslaunch localization_v3 localization_v3.launch
# 只构建/运行单元测试
catkin_make run_tests_localization_v3
```
