# slam_mapping_v2 重构方案：从“复制移植”到“自有结构”

## 一、现状问题

当前 slam_mapping_v2 是“三套命名空间 + 纯复制”的混合体：

| 问题 | 表现 |
|------|------|
| 命名空间混杂 | `slam_mapping_v2`（自有 runtime/node）+ `slam_mapping`（复制的核心）+ `slam_mapping_ros`（复制的可视化/转换）+ `localization`（复制的 slam_odometry 类型） |
| 类型冗余 | `localization/common/config.hpp` 携带 DrConfig/FusionConfig/RelocConfig/PoseGraphConfig 等大量 v2 不用的字段；`SlamConfig` 是 faster-lio 前端配置，v2 不使用 |
| 配置体系分裂 | 复制的 `param_loader` 读 `slam/type`、`slam/faster_lio/*`、`sensors/*` 等旧键，v2 自己的键在 `v2/*` 下 → 一份 yaml 两套读取来源 |
| 转换链冗余 | `slam_mapping_ros::fromRos → localization_ros::fromRos` 再包一层 |
| 多余抽象 | `i_loop_closure_backend` 接口 + `LoopClosureBackend` 包装 + type 字符串分发：v2 只有一个实现，全部多余 |
| 未消化残留 | 复制的文件仍保留旧命名空间与旧结构，等于“外部包塞进内部” |

## 二、目标结构（只含自己的内容和定义）

单一命名空间 `slam_mapping_v2`（ros 层用子命名空间 `slam_mapping_v2::ros`），
删除 `include/slam_mapping/**`、`include/localization/**`、`src/vendored/**`。
外部依赖仅保留系统库：ROS、PCL、GTSAM（这些是库依赖，不是“外部包函数”）。

```
slam_mapping_v2/
├── include/slam_mapping_v2/
│   ├── common/types.hpp           # 自有类型：Pose/FrameInput/Keyframe/GnssObservation/State/Status
│   ├── common/config.hpp          # 自有配置：仅 v2 需要的键，全部由 v2 yaml 直接加载
│   ├── core/input_buffer.hpp      # odom/scan 时间配对（保留现有实现）
│   ├── core/keyframe_manager.hpp  # 关键帧判定：纯函数式 shouldAdd(prev, odom, scan)，无内部存储、天然线程安全
│   ├── core/global_map.hpp        # 全局地图：关键帧点云按 pose_map 体素累积（增量维护，替代每次全量重建）
│   ├── core/gnss_optimizer.hpp    # GNSS ENU 对齐 + 因子门控（适配 v2 Keyframe/GnssObservation）
│   ├── core/loop_closure.hpp      # 回环：kdtree 候选 + 子图 GICP + GTSAM ISAM2（单实现，无接口包装）
│   ├── core/map_saver.hpp         # 保存：PCD / CSV（适配 v2 Keyframe）
│   ├── core/mapping_runtime.hpp   # 多线程运行时（线程模型不变，类型换成 v2）
│   └── ros/
│       ├── converters.hpp         # 自有转换：odom msg→FrameInput、scan msg→FrameInput、Gpchc→GnssObservation
│       ├── config_loader.hpp      # 读取 v2 yaml 全部键（无 slam_/sensors 旧键）
│       ├── publishers.hpp         # 轻量发布器（或由 node 直接发布）
│       └── node.hpp               # 节点：订阅/服务/线程编排
└── src/（对应实现，每个模块一个文件）
```

## 三、旧模块 → 新模块的适配方式（不是复制，是重写适配）

| 旧（复制进来的） | 新（自有） | 适配要点 |
|------------------|-----------|----------|
| `slam_mapping::MappingKeyframe` 等类型 | `slam_mapping_v2::Keyframe` | 只保留 v2 需要的字段（id/timestamp/pose_odom/pose_map/cloud/point_count/source_frame/gnss/标志） |
| `slam_mapping::KeyframeManager`（内部存 vector） | `slam_mapping_v2::KeyframeManager` | 去掉内部存储：`bool shouldAdd(const Keyframe& prev, const FrameInput& f)`，判定逻辑保留（距离/旋转/帧间隔/点数） |
| `slam_mapping::LoopClosureBackend` + `i_loop_closure_backend` + fastlio2 实现 | `slam_mapping_v2::LoopClosure` | 删掉接口包装与 type 分发；保留 ISAM2 增量因子、kdtree 候选、GICP 子图验证，全部改用 v2 Keyframe |
| `slam_mapping::GnssOptimizer` | `slam_mapping_v2::GnssOptimizer` | ENU 对齐/status 门控/Huber/z 覆盖逻辑保留，类型换成 v2 GnssObservation/Keyframe |
| `slam_mapping::MapSaver` | `slam_mapping_v2::MapSaver` | 保留 PCD/CSV 保存，输入换成 v2 Keyframe |
| `slam_mapping_ros::VisualizationPublisher` | `slam_mapping_v2::ros::Publishers`（或 node 直发） | 话题列表按 v2 outputs 配置，去掉对 slam_mapping 类型的依赖 |
| `slam_mapping_ros::param_loader`（读 slam_/sensors 旧键） | `slam_mapping_v2::ros::loadConfig` | 只读 v2 自有键：`input/`、`outputs/`、`services/`、`threading/`、`keyframe/`、`loop_closure/`（含 gnss）、`map_saver/`、`visualization/` |
| `slam_mapping_ros::ros_converters` + `localization_ros` | `slam_mapping_v2::ros::converters` | 直接实现：odom/scan/Gpchc → v2 类型；status → MappingStatus；无中转层 |
| `localization/common/*` | `slam_mapping_v2/common/*` | 只定义 v2 用到的别名（PointType=pcl::PointXYZINormal 等）与 Pose/矩阵别名 |

## 四、分阶段实施（每阶段有验证门）

> 前置：先在 DeepLumin 仓库提交当前可运行版本作为基线（便于回退与对比）。

**Phase 1 — 自有类型与运行时改造**
1. 新建 `slam_mapping_v2/common/types.hpp`、`common/config.hpp`（只含 v2 内容）。
2. 把 `mapping_runtime_v2` 与 `node` 中的 `slam_mapping::/localization::` 引用全部替换为 v2 类型。
3. 删除 `include/localization/**`。
4. 验证门：编译通过，功能不变（先不删核心模块，可临时用类型别名过渡）。

**Phase 2 — 核心模块重写适配**
1. 重写 `keyframe_manager`（纯判定）、`gnss_optimizer`、`loop_closure`、`map_saver`、
   `global_map` 为 v2 类型/命名空间（算法逻辑保持：ENU 公式、ISAM2 因子、GICP、体素累积）。
2. 删除 `include/slam_mapping/**` 与 `src/vendored/**`（除已迁移的）。
3. 验证门：编译通过；用同一段 bag 回归（关键帧数/回环数/保存产物与重构前一致量级）。

**Phase 3 — ROS 层与配置精简**
1. 重写 converters / config_loader / publishers。
2. `slam_mapping_v2.yaml` 去掉 `slam:`、`sensors:` 旧键，只留 v2 键。
3. 验证门：编译通过；25s bag 端到端（odom 订阅 → 关键帧 → 回环/GNSS → 保存）。

**Phase 4 — 收尾**
1. mapping_manager 对接回归（backend 指向 v2 服务）。
2. 更新 README/DESIGN，删除过渡别名与死代码。

## 五、风险与说明

- **数值行为不变**：GNSS ENU 对齐、ISAM2 因子、GICP 是 GTSAM/PCL 库内的确定性算法，
  重写只改类型/命名空间/接口，不改公式与调用次序；对比回归即可确认。
- **GTSAM/PCL/ROS 是系统库依赖**：不属于“外部包的函数库”，保留。
- **收益**：单一命名空间、配置单一来源、无冗余类型、无中转层、每个模块职责清晰，
  后续维护与扩展（如增加描述子回环）都在 v2 自己的结构内进行。
> 四条补充决策（回环接口 / iVox 建图 / 话题裁剪 / 数据结构审计）见 [REFACTOR_SUPPLEMENT.md](REFACTOR_SUPPLEMENT.md)。
