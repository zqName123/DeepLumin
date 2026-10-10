# localization_v2 模块总结文档

> 基于代码逆向分析与详细设计文档整理
> 最后更新：2026-09-21

---

## 目录

1. [模块概述](#1-模块概述)
2. [项目结构](#2-项目结构)
3. [核心数据结构定义](#3-核心数据结构定义)
4. [状态机定义](#4-状态机定义)
5. [融合核心（FusionCore）](#5-融合核心fusioncore)
6. [重定位接口层](#6-重定位接口层)
7. [ROS 节点配置](#7-ros-节点配置)
8. [话题接口总览](#8-话题接口总览)
9. [初始化仲裁机制](#9-初始化仲裁机制)
10. [跟踪失效检测与恢复](#10-跟踪失效检测与恢复)
11. [核心算法公式](#11-核心算法公式)
12. [配置参数完整清单](#12-配置参数完整清单)
13. [依赖关系](#13-依赖关系)
14. [测试覆盖](#14-测试覆盖)

---

## 1. 模块概述

`localization_v2` 是一个基于 ROS 的新一代定位融合节点。其核心思想来源于 `ysw_style_fusion`，通过**高频局部里程计 + 低频全局修正**的方式，输出平滑且全局一致的 `map->base_link` 位姿。

### 1.1 设计目标

| 目标 | 说明 |
|------|------|
| **高频平滑输出** | 100Hz 输出融合位姿，满足控制闭环需求 |
| **全局一致性** | 通过低频全局匹配结果修正 `map->odom`，消除里程计累积漂移 |
| **多源初始化** | 支持重定位、人工初值、全局匹配三种初始化来源，带仲裁机制 |
| **跟踪失效检测** | GM 连续失败自动触发跟踪丢失，重定位恢复失败进入定位失败 |
| **降级输出** | 跟踪丢失期间可选择继续输出，保障安全 |
| **解耦可测试** | 核心融合逻辑纯 C++，不依赖 ROS，便于单元测试与跨平台复用 |

### 1.2 坐标系约定

```
map (全局地图坐标系)
  └── odom (SLAM 局部坐标系，经 map->odom 修正)
       └── base_link (车体坐标系)
```

### 1.3 术语表

| 术语 | 定义 |
|------|------|
| **map 系** | 全局地图坐标系，通常为地图原点 |
| **odom 系** | SLAM 局部坐标系，由 slam_odometry_v2 维护 |
| **base 系** | 车辆本体坐标系（base_link） |
| **map->odom** | 全局到局部坐标系的修正变换，由本节点维护 |
| **map->base** | 车辆在全局地图中的位姿，本节点最终输出 |
| **odom->base** | 局部里程计位姿，由 slam_odometry_v2 高频输出 |
| **Jump Guard** | 跳变门限，防止单次全局匹配结果导致位姿突变 |
| **Tracking Lost** | 跟踪丢失，GM 连续失败达到阈值 |
| **Localization Failure** | 定位失败，重定位恢复超时或连续失败 |

---

## 2. 项目结构

```
localization_v2/
├── CMakeLists.txt                          # 构建配置（3 个库 + 1 节点 + 1 测试）
├── package.xml                             # 包依赖声明
├── README.md                               # 快速开始与设计概览
│
├── config/
│   └── localization_v2.yaml                # 唯一配置文件（全部可调参数）
│
├── launch/
│   └── localization_v2.launch              # 完整定位链路启动文件
│
├── rviz/
│   └── localization_v2.rviz                # RViz 可视化配置
│
├── docs/
│   ├── detailed_design.md                  # 详细设计文档
│   ├── implementation_guide.md             # 实现指南
│   ├── localizaiton_v2系统介绍文档.md       # 系统介绍
│   ├── state_machine_design.md             # 状态机设计文档
│   └── localization_v2_summary.md          # 本总结文档
│
├── include/localization_v2/
│   ├── core/
│   │   ├── types.hpp                       # 纯 C++ 位姿类型与数学工具
│   │   ├── fusion_core.hpp                 # 融合核心接口
│   │   └── initialization.hpp              # 初始化状态机接口
│   ├── interface/
│   │   └── relocalization_interface.hpp    # 重定位抽象接口
│   └── ros/
│       ├── localization_v2_config.hpp      # 节点配置结构体
│       └── relocalization_ros_adapter.hpp  # 重定位 ROS 默认实现
│
├── src/
│   ├── core/
│   │   ├── fusion_core.cpp                 # 融合核心实现
│   │   └── initialization.cpp              # 状态机实现
│   └── ros/
│       ├── localization_v2_config.cpp      # 配置加载实现
│       ├── relocalization_ros_adapter.cpp  # ROS 适配层实现
│       └── localization_v2_node.cpp        # 主节点入口（~1100 行）
│
└── test/
    └── test_state_machine.cpp              # 状态机单元测试（13 个用例）
```

### 2.1 构建目标（CMakeLists.txt）

| 目标名 | 类型 | 源文件 | 链接依赖 |
|--------|------|--------|----------|
| `localization_v2_core` | 静态库 | `src/core/fusion_core.cpp`, `src/core/initialization.cpp` | Eigen3 |
| `localization_v2_ros` | 静态库 | `src/ros/localization_v2_config.cpp`, `src/ros/relocalization_ros_adapter.cpp` | core 库 + ROS + PCL |
| `localization_v2_node` | 可执行文件 | `src/ros/localization_v2_node.cpp` | core 库 + ros 库 + ROS + PCL |
| `test_state_machine` | 测试 | `test/test_state_machine.cpp` | core 库 + GTest |

---

## 3. 核心数据结构定义

### 3.1 位姿基础类型 — [types.hpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/localization_v2/include/localization_v2/core/types.hpp)

```cpp
namespace localization_v2 {

using Vec3d = Eigen::Vector3d;
using Quatd = Eigen::Quaterniond;

/// 位姿结构：表示 "parent -> child" 的刚体变换
struct Pose3d {
  Vec3d position = Vec3d::Zero();       // 平移分量（m）
  Quatd orientation = Quatd::Identity(); // 旋转分量（四元数）
};

/// 数值合法性检查
bool isFinite(const Vec3d& v);
bool isFinite(const Quatd& q);

/// 位姿求逆：out = a⁻¹
Pose3d inverse(const Pose3d& a);

/// 位姿复合：out = a ⊕ b（先 b 后 a）
Pose3d compose(const Pose3d& a, const Pose3d& b);

/// 相对位姿：out = from⁻¹ ⊕ to
Pose3d between(const Pose3d& from, const Pose3d& to);

/// 偏航角差（度，范围 [-180, 180]）
double yawDiffDeg(const Pose3d& a, const Pose3d& b);

/// 平移距离（米）
double translationDiff(const Pose3d& a, const Pose3d& b);

} // namespace localization_v2
```

### 3.2 跳变门限配置 — [fusion_core.hpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/localization_v2/include/localization_v2/core/fusion_core.hpp)

```cpp
struct JumpGuardConfig {
  bool enabled = false;       ///< 是否启用跳变门限（默认关 = ysw_loc 原始行为）
  double max_translation = 5.0;  ///< 单次修正最大平移跳变（米）
  double max_yaw_deg = 50.0;     ///< 单次修正最大偏航跳变（度）
};
```

### 3.3 全局匹配更新 — [fusion_core.hpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/localization_v2/include/localization_v2/core/fusion_core.hpp)

```cpp
struct GlobalMatchUpdate {
  bool success = false;            ///< 匹配是否成功
  bool has_pose = false;           ///< 消息是否携带可用位姿
  Pose3d pose = Pose3d();          ///< 位姿：map->odom 或 map->base
  bool pose_is_map_base = false;   ///< true: pose 为 map->base；false: pose 为 map->odom
  std::string map_id;              ///< 地图标识
  double fitness_score = 0.0;      ///< 匹配 fitness 分数
  double inlier_ratio = 0.0;       ///< 内点比率
};
```

### 3.4 重定位更新 — [fusion_core.hpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/localization_v2/include/localization_v2/core/fusion_core.hpp)

```cpp
struct RelocalizationUpdate {
  bool success = false;              ///< 重定位是否成功
  bool accepted = false;             ///< 是否被外部接受
  Pose3d map_base = Pose3d();        ///< 重定位给出的 map->base_link 位姿
  double fitness_score = 0.0;        ///< 匹配 fitness 分数
  double inlier_ratio = 0.0;         ///< 内点比率
  double inlier_rmse = 0.0;          ///< 内点 RMSE
  std::string map_id;                ///< 地图标识
  std::string method;                ///< 重定位方法名
};
```

### 3.5 初始化配置 — [initialization.hpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/localization_v2/include/localization_v2/core/initialization.hpp)

```cpp
/// 初始化策略配置
struct InitConfig {
  bool require_initial_pose = false;        ///< 是否必须获得初始位姿后才输出定位
  bool use_relocalization_init = true;      ///< 是否允许使用重定位作为初始位姿来源
  bool use_manual_initialpose = true;       ///< 是否允许人工 initial_pose 完成初始化/重置
  bool use_global_matcher_init = true;      ///< 是否允许 GM 自身成功即视为初始化完成
};

/// 重定位启停策略
struct RelocalizationPolicy {
  bool enabled = true;                      ///< 重定位功能总开关
  bool auto_start = true;                   ///< 未初始化且需要初值时，是否自动启动重定位
  bool stop_on_success = true;              ///< 重定位成功（过门限）后停止
  bool stop_on_manual_pose = true;          ///< 收到人工 initial_pose 后停止
  bool stop_on_global_match = true;         ///< GM 自身成功匹配后停止
};

/// 跟踪失效检测与恢复防抖配置
struct TrackingFailureConfig {
  int gm_failure_threshold = 10;            ///< GM 连续失败阈值，达到此值触发跟踪丢失
  int reloc_failure_threshold = 5;          ///< 重定位连续失败阈值，达到此值触发定位失败
  double reloc_timeout_sec = 30.0;          ///< 重定位超时时间（秒）
  int gm_recovery_threshold = 3;            ///< GM 恢复确认窗口：连续成功 N 次才恢复
  int reloc_recovery_threshold = 2;         ///< 重定位恢复确认窗口：连续成功 N 次才恢复
  double tracking_min_dwell_sec = 2.0;      ///< TRACKING 最小驻留时间（秒）
  double tracking_lost_min_dwell_sec = 1.0; ///< TRACKING_LOST 最小驻留时间（秒）
  int gm_degraded_threshold = 20;           ///< 跟踪退化阈值：连续退化帧数达到此值触发跟踪丢失
};
```

### 3.6 状态机枚举 — [initialization.hpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/localization_v2/include/localization_v2/core/initialization.hpp)

```cpp
enum class RelocCommand {
  kNone = 0,    ///< 无动作
  kStart,       ///< 请求启动重定位
  kStop,        ///< 请求停止重定位
};

class LocalizationStateMachine {
 public:
  enum class State {
    kUninitialized = 0,       ///< 未初始化，等待初始位姿
    kRelocalizing,            ///< 重定位初始化中
    kTracking,                ///< 正常跟踪，定位有效
    kTrackingLost,            ///< 跟踪丢失，尝试恢复中
    kLocalizationFailure,     ///< 定位失败，需要人工干预
  };
  // ... （见第 4 节接口详情）
};
```

### 3.7 重定位信号 — [relocalization_interface.hpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/localization_v2/include/localization_v2/interface/relocalization_interface.hpp)

```cpp
struct RelocalizationSignal {
  bool success = false;              ///< 重定位是否成功
  bool accepted = false;             ///< 是否被接受
  Pose3d map_base;                   ///< map->base_link 位姿
  double fitness_score = 0.0;        ///< 匹配 fitness 分数
  double inlier_ratio = 0.0;         ///< 内点比率
  double inlier_rmse = 0.0;          ///< 内点 RMSE
  std::string map_id;                ///< 地图标识
  std::string method;                ///< 重定位方法名
};

/// 重定位结果回调函数类型
using RelocalizationResultCallback = std::function<void(const RelocalizationSignal&)>;
```

### 3.8 重定位接口 — [relocalization_interface.hpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/localization_v2/include/localization_v2/interface/relocalization_interface.hpp)

```cpp
class IRelocalizationClient {
 public:
  virtual ~IRelocalizationClient() = default;
  virtual void start() = 0;                                          ///< 请求启动重定位（幂等）
  virtual void stop() = 0;                                           ///< 请求停止重定位（幂等）
  virtual void setResultCallback(RelocalizationResultCallback) = 0;  ///< 注册结果回调
};
```

### 3.9 节点运行时内部缓存状态

以下为 `LocalizationV2Node` 类中用于维护运行时状态的核心成员变量：

```cpp
// ---- 内部缓存 ----
sensor_msgs::PointCloud2::ConstPtr latest_cloud_;   // 最新点云（odom 系）
geometry_msgs::TwistWithCovariance latest_twist_;    // 最新速度（来自 slam）
nav_msgs::Path path_;                                 // 融合轨迹缓存
pcl::PointCloud<pcl::PointXYZI>::Ptr init_z_map_;     // 人工初值 z 补全用地图
std::string latest_map_id_;                           // 最新地图标识
ros::Time relocalizing_since_;                        // 进入重定位状态的时刻

// ---- 重定位候选缓存（初始化仲裁用） ----
bool reloc_result_pending_ = false;
Pose3d reloc_result_map_base_;
double reloc_result_inlier_ratio_ = 0.0;
double reloc_result_inlier_rmse_ = std::numeric_limits<double>::infinity();
std::string reloc_result_map_id_;
std::string reloc_result_method_;
ros::Time reloc_result_stamp_;

// ---- GM 候选缓存（初始化仲裁用） ----
bool have_gm_candidate_ = false;
double gm_candidate_inlier_ratio_ = 0.0;
double gm_candidate_inlier_rmse_ = std::numeric_limits<double>::infinity();
Pose3d gm_candidate_map_odom_;
std::string gm_candidate_map_id_;

// ---- 跟踪丢失计时 ----
ros::Time tracking_lost_since_;
```

---

## 4. 状态机定义

### 4.1 状态转换图

```
                    ┌──────────────────────────────────────────────────────┐
                    │                                                      │
                    ▼                                                      │
┌──────────────┐   onSlamOdom()     ┌──────────────┐                      │
│ UNINITIALIZED │ ─────────────────→ │ RELOCALIZING │                     │
│              │  (auto_start=true)  │              │                     │
└──────┬───────┘                     └──────┬───────┘                     │
       │                                    │                             │
       │  ┌──────────────────────────────────┘                            │
       │  │  ┌──────────────────────────────────────┐                    │
       │  │  │                                      │                    │
       │  │  onRelocalizationResult(accepted)       │                    │
       │  │  onManualPose()                         │                    │
       │  │  onGlobalMatchSuccess()                 │                    │
       │  ▼                                        ▼                    │
       │  ┌──────────────────────────────────────────┐                  │
       │  │                TRACKING                  │                  │
       │  │            （正常运行/输出）               │ ◄────────────────┤
       │  └──────────────────┬───────────────────────┘                  │
       │                     │                                          │
       │                     │ onTrackingFailure()                      │
       │                     │ (GM连续失败达阈值)                        │
       │                     ▼                                          │
       │  ┌──────────────────────────────────────────┐                  │
       │  │             TRACKING_LOST                 │ ────────────────┘
       │  │            （跟踪丢失/恢复中）              │
       │  └──────────────────┬───────────────────────┘
       │                     │
       │                     │ onRecoveryFailed()
       │                     │ (重定位超时/连续失败达阈值)
       │                     ▼
       │  ┌──────────────────────────────────────────┐
       │  │         LOCALIZATION_FAILURE              │
       └──│            （定位失败/需人工干预）           │
          └──────────────────────────────────────────┘
```

### 4.2 状态转换表

| 当前状态 | 事件 | 条件 | 下一状态 | 动作 |
|----------|------|------|----------|------|
| UNINITIALIZED | `onSlamOdom` | `auto_start && use_relocalization_init` | RELOCALIZING | 下发 `start` 命令 |
| UNINITIALIZED | `onManualPose` | `use_manual_initialpose` | TRACKING | 下发 `stop` 命令（若启用），绕过驻留检查 |
| UNINITIALIZED | `onGlobalMatchSuccess` | `use_global_matcher_init` | TRACKING | 下发 `stop` 命令（若启用） |
| RELOCALIZING | `onManualPose` | `use_manual_initialpose` | TRACKING | 下发 `stop` 命令（若启用），绕过驻留检查 |
| RELOCALIZING | `onRelocalizationResult` | `accepted=true` | TRACKING | 下发 `stop` 命令（若启用），绕过驻留检查 |
| RELOCALIZING | `onGlobalMatchSuccess` | 单源GM模式 | TRACKING | 下发 `stop` 命令（若启用） |
| RELOCALIZING | `onGlobalMatchSuccess` | 双源模式 | RELOCALIZING | 缓存候选，等待仲裁 |
| TRACKING | `onTrackingFailure` | `gm_failures >= threshold` | TRACKING_LOST | 下发 `start` 命令，检查驻留时间 |
| TRACKING | `onTrackingDegraded` | `gm_degraded >= threshold` | TRACKING_LOST | 下发 `start` 命令，检查驻留时间 |
| TRACKING_LOST | `onRelocalizationResult` | `accepted=true` | TRACKING | 下发 `stop` 命令（若启用），需检查驻留时间 |
| TRACKING_LOST | `onGlobalMatchSuccess` | quality_good | TRACKING | 下发 `stop` 命令（若启用），需检查驻留时间 |
| TRACKING_LOST | `onManualPose` | `use_manual_initialpose` | TRACKING | 下发 `stop` 命令（若启用），绕过驻留检查 |
| TRACKING_LOST | `onRecoveryFailed` | `reloc_failures >= threshold` 或超时 | LOCALIZATION_FAILURE | 下发 `stop` 命令 |
| LOCALIZATION_FAILURE | `onGlobalMatchSuccess` | quality_good | TRACKING | 下发 `stop` 命令（若启用） |
| LOCALIZATION_FAILURE | `onManualPose` | `use_manual_initialpose` | TRACKING | 不下发命令（从失败恢复不做 stop） |

### 4.3 LocalizationStateMachine 完整接口

```cpp
// === 配置 ===
void configure(const InitConfig& init,
               const RelocalizationPolicy& policy,
               const TrackingFailureConfig& tracking_config);

// === 状态查询 ===
State state() const;
bool initialized() const;          // TRACKING / TRACKING_LOST / LOCALIZATION_FAILURE
bool relocalizationRunning() const;// 是否在 RELOCALIZING 状态
bool trackingLost() const;         // 是否在 TRACKING_LOST 状态
bool localizationFailure() const;  // 是否在 LOCALIZATION_FAILURE 状态
const std::string& stateName() const;

// === 命令取走（每周期调用一次，取走后自动清零）===
RelocCommand takeCommand();

// === 事件处理 ===
void onSlamOdom();                          // slam 数据到达
void onManualPose();                        // 人工 initial_pose
void onRelocalizationResult(bool accepted); // 重定位结果
void onGlobalMatchSuccess();                // GM 自身匹配成功
void onTrackingFailure();                   // GM 连续失败达阈值
void onTrackingDegraded();                  // GM 连续退化达阈值
void onRecoveryFailed();                    // 重定位连续失败达阈值
void onRecoveryAttemptFailed();             // 恢复尝试失败（重置确认计数器）
void forceLocalizationFailure();            // 强制切换为定位失败（如超时）

// === 计数器查询与重置 ===
int getGmConsecutiveFailures() const;
int getGmDegradedCount() const;
int getRelocConsecutiveFailures() const;
void resetGmCounter();
void resetDegradedCounter();
void resetRelocCounter();
```

---

## 5. 融合核心（FusionCore）

### 5.1 FusionCore 完整接口

```cpp
class FusionCore {
 public:
  // ---- 配置 ----
  void setJumpGuard(const JumpGuardConfig& config);
  void requestJumpGuardBypass();  // 请求下次跳过跳变门限（一次性生效）

  // ---- slam 局部里程计 ----
  void setOdomBase(const Pose3d& odom_base);
  bool hasSlam() const;
  const Pose3d& odomBase() const;

  // ---- 全局修正量（map->odom） ----
  bool hasMapOdom() const;
  const Pose3d& mapOdom() const;

  // ---- 更新方法 ----
  bool updateFromGlobalMatch(const GlobalMatchUpdate& update);
  bool setMapOdom(const Pose3d& map_odom);
  bool updateMapOdomFromMapBase(const Pose3d& map_base, bool require_slam);

  // ---- 输出 ----
  Pose3d fusedPose() const;  // map->base_link = map_odom ⊕ odom_base
  bool isInitialized() const; // 等价于 have_map_odom_
};
```

### 5.2 跳变门限逻辑

```cpp
bool FusionCore::passesJumpGuard(const Pose3d& candidate_map_odom) const {
  if (!jump_guard_.enabled || !have_map_odom_) {
    return true;  // 不启用或尚无 map_odom 时直接通过
  }
  // 比较 fused_pose 变化量（而非 map_odom 的绝对值）
  const Pose3d old_fused = compose(map_odom_, odom_base_);
  const Pose3d new_fused = compose(candidate_map_odom, odom_base_);
  return translationDiff(old_fused, new_fused) <= jump_guard_.max_translation &&
         std::abs(yawDiffDeg(old_fused, new_fused)) <= jump_guard_.max_yaw_deg;
}
```

**设计要点**：
- 比较的是 `fused_pose`（即 `map->base_link`）的变化量，而非 `map_odom` 的绝对值差异
- 人工初值/重定位后可通过 `requestJumpGuardBypass()` 跳过下一次门限检查
- 无 slam 数据时退化到 `map_odom` 绝对值比较

---

## 6. 重定位接口层

### 6.1 接口架构

```
IRelocalizationClient（抽象接口）
        ▲
        │ 继承
        │
RelocalizationRosAdapter（默认ROS实现）
```

### 6.2 RelocalizationRosAdapter 实现

```cpp
class RelocalizationRosAdapter : public IRelocalizationClient {
 public:
  RelocalizationRosAdapter(ros::NodeHandle& nh,
                           const std::string& result_topic,
                           const std::string& command_topic);

  void start() override;   // 发布 "start" 命令
  void stop() override;    // 发布 "stop" 命令
  void setResultCallback(RelocalizationResultCallback callback) override;

 private:
  void onResult(const deeplumin_msgs::RelocalizationResult::ConstPtr& msg);
  void publishCommand(const std::string& command);

  ros::Subscriber result_sub_;    // 订阅结果话题
  ros::Publisher command_pub_;    // 发布命令话题
  RelocalizationResultCallback callback_;
};
```

### 6.3 通信协议

| 方向 | 话题 | 类型 | 频率 |
|------|------|------|------|
| 输入 | 可配置 | `deeplumin_msgs/RelocalizationResult` | 事件触发 |
| 输出 | 可配置 | `std_msgs/String`（"start"/"stop"） | 事件触发 |

---

## 7. ROS 节点配置

### 7.1 节点配置结构体 — [localization_v2_config.hpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/localization_v2/include/localization_v2/ros/localization_v2_config.hpp)

```cpp
struct LocalizationV2Config {
  // ---- 坐标系 ----
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  std::string base_frame = "base_link";

  // ---- 话题（输入） ----
  std::string slam_odom_topic = "/localization/slam_odom";
  std::string global_matcher_result_topic = "/localization/global_matcher/result";
  std::string initial_pose_topic = "/initialpose";
  std::string source_cloud_topic = "/localization/cloud_registered_effect";

  // ---- 话题（输出） ----
  std::string fused_odom_topic = "/localization_v2/odom";
  std::string map_to_odom_topic = "/localization_v2/map_to_odom";
  std::string fused_path_topic = "/localization_v2/path";
  std::string status_topic = "/localization_v2/status";
  std::string current_scan_in_map_topic = "/localization_v2/current_scan_in_map";
  std::string relocalization_result_topic = "/localization/relocalization/result";
  std::string relocalization_command_topic = "/localization_v2/relocalization/command";
  std::string gm_anchor_topic = "/localization_v2/gm_anchor";
  std::string loc_point_topic = "/localization_v2/loc_point";

  // ---- 运行时 ----
  double output_rate_hz = 100.0;         // 输出频率
  int max_path_size = 50000;             // 轨迹最大保留点数
  bool publish_tf = true;                // 是否发布 TF
  bool publish_map_to_odom_tf = true;    // 是否发布 map->odom TF
  bool publish_odom_to_base_tf = false;  // 是否发布 odom->base TF
  bool publish_path = true;              // 是否发布轨迹
  bool publish_current_scan_in_map = true; // 是否发布 map 系点云
  double base_yaw_offset = 0.0;          // base_link yaw 偏移（度）

  // ---- global_matcher 结果语义 ----
  std::string gm_result_pose_frame = "odom"; // "odom" 或 "base"
  bool enable_jump_guard = false;
  double max_jump_translation = 5.0;
  double max_jump_yaw_deg = 50.0;

  // ---- 初始化策略 ----
  bool require_initial_pose = false;
  bool use_relocalization_init = true;
  bool use_manual_initialpose = true;
  bool use_global_matcher_init = true;

  // ---- 重定位策略 ----
  bool relocalization_enabled = true;
  bool relocalization_auto_start = true;
  bool relocalization_stop_on_success = true;
  bool relocalization_stop_on_manual_pose = true;
  bool relocalization_stop_on_global_match = true;
  bool relocalization_require_accepted = false;
  double relocalization_max_fitness_score = 1.0;
  double relocalization_min_inlier_ratio = 0.5;
  double relocalization_max_inlier_rmse = 0.8;
  double relocalization_fallback_timeout = 8.0; // 双源初始化仲裁超时（秒）

  // ---- 人工初始位姿 z 补全 ----
  bool initial_pose_z_completion = true;
  double initial_pose_z_search_radius = 5.0;
  std::string initial_pose_map_pcd_path;

  // ---- 跟踪失效检测 ----
  int gm_failure_threshold = 10;
  double gm_max_inlier_rmse = 0.8;

  // ---- 重定位恢复 ----
  int reloc_failure_threshold = 5;
  double reloc_timeout_sec = 30.0;

  // ---- 恢复防抖 ----
  int gm_recovery_threshold = 3;
  int reloc_recovery_threshold = 2;
  double tracking_min_dwell_sec = 2.0;
  double tracking_lost_min_dwell_sec = 1.0;

  // ---- 跟踪质量等级 ----
  double gm_good_min_inlier_ratio = 0.69;
  double gm_degraded_min_inlier_ratio = 0.4;
  int gm_degraded_threshold = 20;

  // ---- 降级输出 ----
  bool allow_degraded_output = true;
};
```

---

## 8. 话题接口总览

### 8.1 输入话题（订阅）

| 话题 | 类型 | 来源 | 含义 | 频率 |
|------|------|------|------|------|
| `slam_odom_topic` | `nav_msgs/Odometry` | lio_odometry / slam_odometry_v2 | 局部里程计 `odom->base_link` | ~100Hz |
| `global_matcher_result_topic` | `deeplumin_msgs/GlobalMatchResult` | global_matcher_v2 | 全局匹配结果 | ~1-10Hz |
| `initial_pose_topic` | `geometry_msgs/PoseWithCovarianceStamped` | RViz / 外部 | 人工初始位姿 | 事件触发 |
| `relocalization_result_topic` | `deeplumin_msgs/RelocalizationResult` | relocalization | 重定位结果（map->base） | 事件触发 |
| `source_cloud_topic` | `sensor_msgs/PointCloud2` | lio_odometry | 当前帧点云（odom 系，可视化用） | 与 lidar 同频 |

### 8.2 输出话题（发布）

| 话题 | 类型 | 订阅方 | 含义 | 频率 |
|------|------|--------|------|------|
| `fused_odom_topic` | `nav_msgs/Odometry` | 控制/规划 | 融合位姿 `map->base_link`（含速度） | 100Hz |
| `map_to_odom_topic` | `nav_msgs/Odometry` | 调试 | 修正量 `map->odom` | 100Hz |
| `fused_path_topic` | `nav_msgs/Path` | RViz | 融合轨迹历史 | 100Hz（可选） |
| `status_topic` | `deeplumin_msgs/FusionStatus` | 监控 | 定位状态（含数据源可用性、失败原因） | 100Hz |
| `current_scan_in_map_topic` | `sensor_msgs/PointCloud2` | RViz | 当前帧点云在 map 系 | 100Hz（可选） |
| `gm_anchor_topic` | `geometry_msgs/PoseWithCovarianceStamped` | global_matcher | 初始化仲裁重定位胜出时通知 GM 重锚 | 仅初始化阶段（一次性） |
| `relocalization_command_topic` | `std_msgs/String`("start"/"stop") | relocalization | 重定位启停命令 | 事件触发 |
| `loc_point_topic` | `deeplumin_msgs/LocPoint` | 规划 | 规划订阅的位姿数据（含 x, y, yaw, pitch, vec） | 100Hz |

### 8.3 TF 发布

| TF | parent → child | 条件 | 说明 |
|----|----------------|------|------|
| `map` → `odom` | `publish_map_to_odom_tf=true` | 修正量，100Hz |
| `odom` → `base_link` | `publish_odom_to_base_tf=true` | 默认关闭（slam_odometry_v2 已发布） |
| `map` → `base_link` | 始终发布 | 融合位姿，带 `base_yaw_offset` 适配 |

### 8.4 FusionStatus 消息字段详解

```cpp
// 来自 deeplumin_msgs/FusionStatus
msg.header            // stamp + frame_id
msg.level             // NORMAL=0 / WARNING=1 / DEGRADED=2 / ERROR=3 / FAILURE=4
msg.state             // "UNINITIALIZED" / "RELOCALIZING" / "TRACKING" / "TRACKING_LOST" / "LOCALIZATION_FAILURE"
msg.initialized       // 是否已完成初始化
msg.output_valid      // 当前输出是否有效
msg.current_map_id    // 当前地图标识

// 数据源年龄（秒）
msg.last_slam_age         // 0.0（有 slam 数据）或 1e9
msg.last_global_matcher_age  // 0.0（有 GM 候选）或 1e9
msg.last_relocalization_age  // RELOCALIZING/TRACKING_LOST 状态下为距 relocalizing_since_ 的秒数，否则 1e9
msg.last_dr_age / last_gnss_age  // 未使用，固定 1e9
msg.last_can_age / last_imu_age  // 固定 0.0（始终可用）
msg.last_lidar_age        // 与 last_slam_age 相同

// 各维度标准差（根据状态动态赋值）
msg.position_std / msg.velocity_std / msg.roll_std / msg.pitch_std / msg.yaw_std

// 数据源使用与拒绝信息
msg.used_sources       // 当前使用的数据源列表
msg.rejected_sources   // 被拒绝的数据源列表
msg.reject_reasons     // 拒绝原因列表

// 数据源可用性
msg.is_slam_available / msg.is_global_matcher_available / msg.is_relocalization_available
msg.is_dr_available / msg.is_gnss_available / msg.is_can_available / msg.is_imu_available / msg.is_lidar_available
```

**不同状态下的 level 与 output_valid：**

| 状态 | level | output_valid | position_std |
|------|-------|-------------|--------------|
| UNINITIALIZED | WARNING | false | 100.0 |
| RELOCALIZING | WARNING | false | 50.0 |
| TRACKING（正常） | NORMAL | true | 0.1 |
| TRACKING（退化） | WARNING | true | 0.1 |
| TRACKING_LOST | DEGRADED | `allow_degraded_output` | 5.0 |
| LOCALIZATION_FAILURE | FAILURE | false | 100.0 |

### 8.5 LocPoint 消息结构

```cpp
// 来自 deeplumin_msgs/LocPoint
msg.header    // stamp + frame_id = "map"
msg.x         // map 系 x 坐标（米）
msg.y         // map 系 y 坐标（米）
msg.yaw       // 偏航角（度，范围 [-180, 180]）
msg.pitch     // 俯仰角（度）
msg.vec       // 前进速度（m/s），取自 slam 里程计 twist.linear.x
```

---

## 9. 初始化仲裁机制

### 9.1 数据流

```
收到 slam 数据（首次）
  │
  ▼
UNINITIALIZED → RELOCALIZING（自动启动重定位）
  │
  ├── 重定位结果到达 ──→ 质量门限检查 ──→ 缓存结果 ──→ 等待 GM 或超时
  └── GM 结果到达 ────→ 缓存候选 ──────→ 等待重定位或超时
                                            │
                                            ▼
                                      tryCompleteInitialization()
                                            │
                              ┌─────────────┴─────────────┐
                              ▼                           ▼
                       双源都已就绪                   超时（fallback_timeout）
                              │                           │
                              ▼                           ▼
                     arbitrateAndComplete()      取已就绪的一源
                              │
                    ┌─────────┴─────────┐
                    ▼                   ▼
           completeWithReloc()   completeWithGm()
                    │                   │
                    └─────────┬─────────┘
                              ▼
                          TRACKING
```

### 9.2 仲裁评分逻辑

```cpp
void arbitrateAndComplete() {
  // 综合评分 = inlier_ratio - 0.3 * inlier_rmse
  const double reloc_score = reloc_result_inlier_ratio_ - 0.3 * reloc_result_inlier_rmse_;
  const double gm_score    = gm_candidate_inlier_ratio_ - 0.3 * gm_candidate_inlier_rmse_;
  const bool reloc_better = reloc_score >= gm_score;

  if (reloc_better)
    completeWithReloc();   // 发布 gm_anchor 通知 GM 重锚
  else
    completeWithGm();
}
```

### 9.3 初始化完成处理

```cpp
completeWithReloc():
  ├── fusion_.updateMapOdomFromMapBase(reloc_result_map_base_, require_slam=false)
  ├── publishGmAnchor(reloc_result_map_base_)  // 通知 GM 重锚（一次性，仅初始化阶段）
  ├── state_machine_.onRelocalizationResult(true)  // → TRACKING
  ├── dispatchRelocalizationCommand()              // → 下发 stop
  └── publishMapToOdom()

completeWithGm():
  ├── fusion_.setMapOdom(gm_candidate_map_odom_)
  ├── state_machine_.onRelocalizationResult(true)  // → TRACKING
  ├── dispatchRelocalizationCommand()              // → 下发 stop
  └── publishMapToOdom()
```

**关于 `completeWithGm()` 的说明**：代码中使用 `state_machine_.onRelocalizationResult(true)` 而非 `state_machine_.onGlobalMatchSuccess()` 来触发 TRACKING 切换，这是因为 `onGlobalMatchSuccess()` 在 RELOCALIZING 状态下没有预期行为（仅设计在 `kUninitialized` 和 `kRelocalizing` 状态下处理——但 `onGlobalMatchSuccess` 的代码检查的是 `state_ == kUninitialized || state_ == kRelocalizing`，会处理它。然而这里用 `onRelocalizationResult(true)` 是为了通过 `stop_on_success` 策略触发 stop 命令下发。

---

## 10. 跟踪失效检测与恢复

### 10.1 GM 质量三级分级

GM 结果在 TRACKING 状态下分三个等级处理：

| 等级 | inlier_ratio 条件 | inlier_rmse 条件 | 处理方式 |
|------|-------------------|-------------------|----------|
| **好（good）** | `>= gm_good_min_inlier_ratio` (0.69) | `<= gm_max_inlier_rmse` (0.8) | 重置失败/退化计数器，正常应用位姿 |
| **退化（degraded）** | `>= gm_degraded_min_inlier_ratio` (0.4) | `<= gm_max_inlier_rmse` (0.8) | 增加退化计数，仍应用位姿（level=WARNING），退化帧数达 `gm_degraded_threshold` 触发 TRACKING_LOST |
| **差（bad）** | `< gm_degraded_min_inlier_ratio` (0.4) | 或 `> gm_max_inlier_rmse` (0.8) | 增加失败计数，复位退化计数，不应用位姿，失败数达 `gm_failure_threshold` 触发 TRACKING_LOST |

### 10.2 恢复确认窗口（防抖）

为防止偶发成功导致状态抖动，恢复使用确认窗口机制：

| 恢复来源 | 连续成功次数要求 | 配置参数 |
|----------|-----------------|----------|
| GM（从 TRACKING_LOST 恢复） | `gm_recovery_threshold`（默认 3） | `recovery_anti_jitter/gm_recovery_threshold` |
| 重定位（从 TRACKING_LOST 恢复） | `reloc_recovery_threshold`（默认 2） | `recovery_anti_jitter/reloc_recovery_threshold` |

### 10.3 驻留时间检查（防抖）

为防止 TRACKING ↔ TRACKING_LOST 快速切换：

| 切换方向 | 最小驻留时间 | 配置参数 |
|----------|-------------|----------|
| TRACKING → TRACKING_LOST | `tracking_min_dwell_sec`（默认 2.0s） | `recovery_anti_jitter/tracking_min_dwell_sec` |
| TRACKING_LOST → TRACKING | `tracking_lost_min_dwell_sec`（默认 1.0s） | `recovery_anti_jitter/tracking_lost_min_dwell_sec` |

### 10.4 超时检测

```cpp
// 在 timer 回调中执行
if (state == TRACKING_LOST) {
  const double elapsed = (ros::Time::now() - tracking_lost_since_).toSec();
  if (elapsed >= reloc_timeout_sec) {
    state_machine_.forceLocalizationFailure();
    // 强制切换为 LOCALIZATION_FAILURE
  }
}
```

---

## 11. 核心算法公式

### 11.1 位姿融合

```cpp
// 全局修正（低频，GM / 人工初值 / 重定位 更新）
map_odom = map_base ⊕ (odom_base)⁻¹
// 其中 ⊕ 表示 compose(a, b) = a ⊕ b（先 b 后 a）

// 高频输出（100Hz，每次 slam_odom 或定时器触发）
map_base = map_odom ⊕ odom_base
```

### 11.2 坐标系转换（GM 结果）

```cpp
if (pose_is_map_base) {
  // GM 输出为 map->base：需要有 slam 数据才能反推
  candidate_map_odom = compose(update.pose, inverse(odom_base_));
} else {
  // GM 输出为 map->odom：直接使用
  candidate_map_odom = update.pose;
}
```

### 11.3 人工初值 / 重定位反推

```cpp
// map_odom = map_base ⊕ (odom_base)⁻¹
map_odom_ = compose(map_base, inverse(odom_base_));
```

### 11.4 跳变门限（fused_pose 变化量）

```cpp
// 比较 fused_pose（map->base）的变化量，而非 map_odom 的绝对值
const Pose3d old_fused = compose(current_map_odom, odom_base);
const Pose3d new_fused = compose(candidate_map_odom, odom_base);
const bool passed = translationDiff(old_fused, new_fused) <= max_translation &&
                    std::abs(yawDiffDeg(old_fused, new_fused)) <= max_yaw_deg;
```

### 11.5 点云变换（发布 current_scan_in_map）

```cpp
// 将 odom 系点云变换到 map 系
pcl::transformPointCloud(in, out, toMatrix4f(map_odom));
```

### 11.6 人工初值 z 补全

```cpp
// 1. 以 (x, y) 为中心，z_search_radius 为半径搜索地图 PCD
// 2. 取搜索范围内 z 的最小值和最大值
// 3. z_out = 0.5 * (z_min + z_max)
```

---

## 12. 配置参数完整清单

### 12.1 按 YAML 结构分组

| 配置分组 | 参数路径 | 类型 | 默认值 | 说明 |
|----------|---------|------|--------|------|
| **frames** | `frames/map` | string | "map" | 全局地图坐标系 |
| | `frames/odom` | string | "odom" | SLAM 局部坐标系 |
| | `frames/base_link` | string | "base_link" | 车辆本体坐标系 |
| **topics** | `topics/slam_odom` | string | "/localization/slam_odom" | slam 里程计输入 |
| | `topics/global_matcher_result` | string | "/localization/global_matcher/result" | GM 结果输入 |
| | `topics/initial_pose` | string | "/initialpose" | 人工初值输入 |
| | `topics/source_cloud` | string | "/localization/cloud_registered_effect" | 点云输入（可视化） |
| | `topics/fused_odom` | string | "/localization_v2/odom" | 融合里程计输出 |
| | `topics/map_to_odom` | string | "/localization_v2/map_to_odom" | 修正量输出 |
| | `topics/fused_path` | string | "/localization_v2/path" | 轨迹输出 |
| | `topics/status` | string | "/localization_v2/status" | 状态输出 |
| | `topics/current_scan_in_map` | string | "/localization_v2/current_scan_in_map" | map 系点云输出 |
| | `topics/gm_anchor` | string | "/localization_v2/gm_anchor" | GM 锚点通知（仅初始化阶段） |
| | `topics/loc_point` | string | "/localization_v2/loc_point" | 规划订阅位姿数据 |
| **runtime** | `runtime/output_rate_hz` | double | 100.0 | 输出频率 |
| | `runtime/max_path_size` | int | 50000 | 轨迹最大保留点数 |
| | `runtime/publish_tf` | bool | true | 是否发布 TF |
| | `runtime/publish_map_to_odom_tf` | bool | true | 是否发布 map->odom TF |
| | `runtime/publish_odom_to_base_tf` | bool | false | 是否发布 odom->base TF |
| | `runtime/publish_path` | bool | true | 是否发布轨迹 |
| | `runtime/publish_current_scan_in_map` | bool | true | 是否发布 map 系点云 |
| | `runtime/base_yaw_offset` | double | 0.0 | base_link yaw 偏移 |
| **global_matcher** | `global_matcher/result_pose_frame` | string | "odom" | GM 结果坐标系 |
| | `global_matcher/enable_jump_guard` | bool | false | 是否启用跳变门限 |
| | `global_matcher/max_jump_translation` | double | 5.0 | 最大平移跳变（米） |
| | `global_matcher/max_jump_yaw_deg` | double | 50.0 | 最大偏航跳变（度） |
| | `global_matcher/max_inlier_rmse` | double | 0.8 | GM inlier RMSE 上限 |
| **initialization** | `initialization/require_initial_pose` | bool | false | 是否必须初始位姿后才输出 |
| | `initialization/use_relocalization_init` | bool | true | 允许重定位初始化 |
| | `initialization/use_manual_initialpose` | bool | true | 允许人工初值初始化 |
| | `initialization/use_global_matcher_init` | bool | true | 允许 GM 自身成功初始化 |
| **relocalization** | `relocalization/result_topic` | string | "/localization/relocalization/result" | 重定位结果话题 |
| | `relocalization/command_topic` | string | "/localization_v2/relocalization/command" | 重定位命令话题 |
| | `relocalization/enabled` | bool | true | 重定位功能总开关 |
| | `relocalization/auto_start` | bool | true | 未初始化时自动启动重定位 |
| | `relocalization/stop_on_success` | bool | true | 重定位成功后停止 |
| | `relocalization/stop_on_manual_pose` | bool | true | 人工初值后停止 |
| | `relocalization/stop_on_global_match` | bool | true | GM 成功后停止 |
| | `relocalization/require_accepted` | bool | false | 是否要求 accepted=true |
| | `relocalization/max_fitness_score` | double | 1.0 | 最大 fitness 分数门限 |
| | `relocalization/min_inlier_ratio` | double | 0.5 | 最小内点比率门限 |
| | `relocalization/max_inlier_rmse` | double | 0.8 | 最大内点 RMSE 门限 |
| | `relocalization/fallback_timeout` | double | 8.0 | 双源初始化仲裁超时（秒） |
| **initial_pose** | `initial_pose/z_completion_enabled` | bool | true | 是否启用 z 补全 |
| | `initial_pose/z_search_radius` | double | 5.0 | z 补全搜索半径（米） |
| | `initial_pose/map_pcd_path` | string | "" | 地图 PCD 路径（留空关闭） |
| **tracking_failure** | `tracking_failure/gm_failure_threshold` | int | 10 | GM 连续失败阈值 |
| **recovery** | `recovery/reloc_failure_threshold` | int | 5 | 重定位连续失败阈值 |
| | `recovery/reloc_timeout_sec` | double | 30.0 | 重定位超时时间（秒） |
| **recovery_anti_jitter** | `recovery_anti_jitter/gm_recovery_threshold` | int | 3 | GM 恢复确认窗口 |
| | `recovery_anti_jitter/reloc_recovery_threshold` | int | 2 | 重定位恢复确认窗口 |
| | `recovery_anti_jitter/tracking_min_dwell_sec` | double | 2.0 | TRACKING 最小驻留时间 |
| | `recovery_anti_jitter/tracking_lost_min_dwell_sec` | double | 1.0 | TRACKING_LOST 最小驻留时间 |
| **tracking_quality** | `tracking_quality/good_min_inlier_ratio` | double | 0.69 | 好/退化分界 |
| | `tracking_quality/degraded_min_inlier_ratio` | double | 0.4 | 退化/差分界 |
| | `tracking_quality/degraded_frame_threshold` | int | 20 | 连续退化帧数阈值 |
| **degraded** | `degraded/allow_degraded_output` | bool | true | TRACKING_LOST 时是否继续输出 |

---

## 13. 依赖关系

### 13.1 编译依赖

| 包/库 | 用途 |
|--------|------|
| `roscpp` | ROS C++ 客户端 |
| `std_msgs` / `geometry_msgs` / `nav_msgs` / `sensor_msgs` | 标准消息类型 |
| `tf2_ros` | TF 广播 |
| `pcl_ros` / `pcl_conversions` | 点云 ROS 消息转换 |
| `deeplumin_msgs` | 自定义消息（FusionStatus, GlobalMatchResult, RelocalizationResult, LocPoint） |
| `deeplumin_health_checker` | 节点健康监测 |
| `syslog_ros_wrap` | 日志系统 |
| `Eigen3` | 矩阵/位姿运算 |
| `PCL` (common, io, filters) | 点云加载与变换 |

### 13.2 运行依赖（完整定位链路）

```
slam_odometry_v2 / lio_odometry
    │  /slam_odometry/odom (nav_msgs/Odometry)
    │  /slam_odometry/cloud_effect (sensor_msgs/PointCloud2)
    ▼
localization_v2 (本节点)
    │  /initialpose (geometry_msgs/PoseWithCovarianceStamped) ← RViz
    │  /localization/global_matcher/result (deeplumin_msgs/GlobalMatchResult) ← global_matcher_v2
    │  /localization/relocalization/result (deeplumin_msgs/RelocalizationResult) ← relocalization
    ▼
输出:
    /localization_v2/odom → 控制/规划
    /localization_v2/status → 监控
    /localization_v2/loc_point → 规划
    TF: map→odom, map→base_link → TF 树
    /localization_v2/relocalization/command → relocalization（start/stop）
    /localization_v2/gm_anchor → global_matcher_v2（一次性锚点）
```

---

## 14. 测试覆盖

### 14.1 状态机单元测试（13 个用例）

测试文件： [test_state_machine.cpp](file:///home/hhy/2004ros/DeepLumin_zq/src/localization/localization_v2/test/test_state_machine.cpp)

| 测试用例 | 验证内容 |
|----------|----------|
| `InitialState` | 初始状态为 UNINITIALIZED，未初始化 |
| `SlameOdomTriggersRelocalizing` | 收到 slam odom 后进入 RELOCALIZING 并下发 start 命令 |
| `ManualPoseAlwaysWorkInUninitialized` | UNINITIALIZED 下人工初值直接进入 TRACKING |
| `ManualPoseWorkInRelocalizing` | RELOCALIZING 下人工初值进入 TRACKING 并下发 stop |
| `RelocalizationSuccessInRelocalizing` | 重定位成功进入 TRACKING 并下发 stop |
| `RelocalizationFailureInRelocalizing` | 重定位失败保持 RELOCALIZING，不下发命令 |
| `GlobalMatchSuccessInRelocalizing` | GM 成功在 RELOCALIZING 下进入 TRACKING |
| `TrackingToTrackingLost` | GM 连续失败达阈值后进入 TRACKING_LOST |
| `TrackingLostToTrackingByRelocSuccess` | TRACKING_LOST 下重定位成功恢复至 TRACKING |
| `TrackingLostToTrackingByManualPose` | TRACKING_LOST 下人工初值恢复至 TRACKING |
| `TrackingLostToLocalizationFailure` | 重定位连续失败达阈值进入 LOCALIZATION_FAILURE |
| `ManualPoseWorkInLocalizationFailure` | LOCALIZATION_FAILURE 下人工初值恢复至 TRACKING |
| `CounterResetOnTracking` | 进入 TRACKING 时计数器重置 |
| `NoActionInTrackingForSlamOdom` | TRACKING 状态下 slam odom 不触发状态变化 |
| `NoActionInTrackingForGlobalMatch` | TRACKING 状态下 GM 成功不触发状态变化（由节点回调处理） |
| `RelocFailureIgnoredInTrackingLost` | TRACKING_LOST 下重定位失败增加计数 |
| `RecoveryFailedOnlyInTrackingLost` | 非 TRACKING_LOST 状态下 recoveryFailed 不触发状态变化 |

---

## 附录

### A. 关键设计决策

1. **融合公式选择**：采用 `ysw_style_fusion` 的 `map->odom` 修正思路，区别于直接修正 `map->base` 的方式。优点是 GM 低频修正只作用于 `map->odom`，融合时用 `map_odom ⊕ odom_base` 输出，保证了 SLAM 高频数据的连续性。

2. **跳变门限比较方式**：比较 `fusedPose`（`map->base`）的变化量而非 `map_odom` 的绝对值差异。原因是 `map_odom` 是绝对状态，当 `odom_base` 累积大量漂移时，`map_odom` 的大修正是对漂移的补偿，不应被拒绝。

3. **重定位接口解耦**：通过 `IRelocalizationClient` 抽象接口将重定位协议与融合逻辑解耦，后续可替换为 Service / Action / gRPC 等通信方式。

4. **双源初始化仲裁**：重定位和 GM 同时可用时进行质量评分仲裁，重定位综合评分不低于 GM 时优先选择重定位（重定位初始化效果更好）。

5. **恢复防抖**：使用确认窗口（连续成功 N 次）和驻留时间双重机制，防止状态快速切换导致的系统抖动。

### B. 内部常量

```cpp
constexpr double kPi = 3.14159265358979323846;
constexpr double kRelocResultMaxAge = 3.0;  // 初始化仲裁中重定位结果的时效窗（秒）
```