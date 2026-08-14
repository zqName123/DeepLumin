# slam_mapping_v2 重构补充决策（四条）

> 配合 [REFACTOR.md](REFACTOR.md) 使用；四条结论直接修正目标结构与实施范围。

## 一、回环接口：保留，但重构为“可插拔检测器 + 独立图优化”

**结论：接口包装有用，保留** —— 后续换 scan_context 等描述子回环时，只需新增一个检测器实现。
现有拷贝有两个问题：① 接口把“候选检测 + 验证 + 图优化 + GNSS”全绑在 `update()` 里，
换检测器等于换整个后端；② 是 slam_mapping 的拷贝，类型是别人家的。

v2 拆成两层：

```cpp
namespace slam_mapping_v2::core {

// 可插拔检测器：只负责“给最新关键帧找候选 + 精配准验证”，产出相对位姿与得分
class ILoopDetector {
 public:
  virtual ~ILoopDetector() = default;
  virtual void configure(const LoopClosureConfig& cfg) = 0;
  virtual LoopCandidate detect(const std::vector<Keyframe>& kfs, int current_idx) = 0;
};

class KdtreeGicpLoopDetector : public ILoopDetector { /* 默认实现：kdtree 候选 + 子图 GICP */ };
// 未来：class ScanContextLoopDetector : public ILoopDetector { /* 复用 relocalization_framework 的 csc */ };

// 图优化（具体类，不绑定检测器）：增量添加里程计/回环/GNSS 因子，ISAM2 更新
class PoseGraphOptimizer {
 public:
  void addOdomFactors(const std::vector<Keyframe>& kfs);
  void addLoopFactor(const LoopCandidate& c);
  void addGnssFactors(const std::vector<Keyframe>& kfs, GnssOptimizer* gnss);
  void optimize(std::vector<Keyframe>* kfs);
};
}  // namespace slam_mapping_v2::core
```

运行时只依赖 `ILoopDetector`；换检测器不动图优化与线程模型。GNSS 优化独立为
`GnssOptimizer`，不进回环接口。

## 二、建图引擎：采用 faster_lio_ws 的 iVox + MapIncremental

**现状问题**：map_preview / 保存每次都从全部关键帧全量重建地图（O(N)），
与 faster-lio 的“增量体素建图”相比浪费且不实时。

**目标**：v2 的 `GlobalMap` 用 faster-lio 的 iVox 增量逻辑：

- 每帧把订阅到的 **odom 系 registered_scan** 直接按 faster-lio `MapIncremental`
  的体素去重规则插入 iVox（O(1)/帧，无需关键帧全量重建）；
- 地图随时可导出（`GetAllPoints`）→ 可视化/保存直接取 iVox 点云（降采样后发布）；
- 关键帧仍保留（回环/GNSS/轨迹需要），但地图本体不再依赖关键帧重建。

落地方式：

- `thirdparty/ivox3d/` 放入 faster_lio_ws 的四个头文件
  （eigen_types / hilbert / ivox3d / ivox3d_node，约 1600 行，仅依赖 pcl/glog/execution），
  保留 `faster_lio` 命名空间，视为第三方库依赖（与 GTSAM 同级）；
- v2 用自有 `GlobalMap` 封装（增量插入、世界系变换、全图导出），业务层不直接见 ivox；
- 新增 `map/` 配置段（体素分辨率、去重 filter_size_map、可视化解像度）。

## 三、发布话题审计（保留必要、裁剪冗余）

| 话题 | 内容 | 判定 | 理由 |
|------|------|------|------|
| `/slam_mapping_v2/status` | MappingStatus | **保留** | mapping_manager 必需 |
| `/slam_mapping_v2/path` | map 系优化轨迹 | **保留** | 回环修正后轨迹，rviz 常用 |
| `/slam_mapping_v2/map_preview` | 全局地图（iVox 增量导出） | **保留** | 主地图输出 |
| TF map→odom | 回环后修正 | **保留**（publish_tf 开关） | 下游/rviz 必需 |
| `/slam_mapping_v2/odom` | 转发 odom | **默认关（冗余）** | 输入 `/localization/slam_odom` 已存在 |
| `/slam_mapping_v2/current_scan` | 转发 registered_scan | **默认关（冗余）** | 输入 `/localization/cloud_registered` 已存在 |
| `/slam_mapping_v2/keyframes` | 关键帧 marker | 默认关（调试） | 开发期开 |
| `/slam_mapping_v2/loop_markers` | 回环 marker | 默认关（调试） | 开发期开 |
| `/slam_mapping_v2/gnss_markers`、`gnss_factor_edges` | GNSS 调试 | 默认关（调试） | 已有 flag |
| `/slam_mapping_v2/loop_current_submap`、`loop_history_submap` | 回环子图 | 默认关（调试） | 回环验证用 |
| `/slam_mapping_v2/local_map` | 前端局部地图 | **不发布** | v2 无前端局部地图 |

原则：只保留“必要”（status/path/全局地图/map→odom TF）与“调试可选”（flag 控制）两类；
转发类（odom/current_scan）默认关闭，需要统一出口时再开。

## 四、数据结构审计与解耦

**实际使用 vs 冗余（v2 视角）**：

| 数据结构 | 结论 |
|----------|------|
| `localization/config.hpp` 的 SlamConfig/ExtrinsicConfig/Dr/Fusion/Reloc/Descriptor 等 | **全部不用**：v2 只用到 FrameConfig（map/odom/base_link）→ 自有 config 只留需要字段 |
| `RuntimeStartRequest.config_path`、`enable_visualization` | v2 未使用（服务消息保留字段以兼容 mapping_manager，但自有请求结构不保存）→ 裁剪 |
| `MappingKeyframe` 各字段 | 全部被用（回环/GNSS/保存/可视化）→ 保留为 v2 Keyframe |
| `MappingRuntimeStatus` | 大部分被 toRosStatus/mapping_manager 使用；`accumulated_point_count`/预热字段 v2 不填 → 自有 Status 去掉未用字段 |
| `LoopClosureConfig` / `GnssOptimizationConfig` | 字段全部被后端使用 → 保留为自有配置 |
| `LoopData`（v2 自有） | 线程间快照，保留 |

**解耦原则**：

1. `core` 不包含 ROS 消息类型（只依赖 pcl/Eigen/GTSAM/ivox3d），消息转换全部在 `ros/converters`；
2. 模块间只依赖 v2 自有类型，单一命名空间 `slam_mapping_v2`；
3. 线程边界不变：pending / committed / LoopData 三份数据 + 显式锁，主线程不碰后端关键帧；
4. 外部库（ROS/PCL/GTSAM/ivox3d）只作为底层依赖，不进入业务层接口。

## 对重构方案的影响

- 目标结构树中的 `loop_closure` 一项替换为：`i_loop_detector` + `loop_detector_kdtree_gicp` + `pose_graph_optimizer`；
- 目标结构树中的 `global_map` 改为基于 `thirdparty/ivox3d` 的增量建图；
- 话题与配置按“必要 + 调试可选”裁剪（odom/current_scan 默认关）；
- 自有 `types/config` 按上表精简，删除未用字段。
