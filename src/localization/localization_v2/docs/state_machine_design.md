# localization_v2 状态机详细设计文档

## 1. 概述

### 1.1 文档目的
本文档详细描述 localization_v2 状态机的扩展设计，解决当前系统存在的问题：
- TRACKING 状态是"死"状态，失效后无法恢复
- 缺少跟踪失败检测机制
- 未向下游发出定位失败信号
- 人工定位在某些状态下不可用

### 1.2 设计目标
- **完整生命周期管理**：覆盖初始化、正常跟踪、跟踪丢失、定位失败全阶段
- **自动恢复机制**：跟踪丢失后自动触发重定位恢复
- **人工干预兜底**：任何状态下人工定位都有效
- **降级输出**：跟踪丢失时继续输出但标记为降级
- **清晰的状态语义**：每个状态有明确的含义和转换条件

---

## 2. 状态定义

### 2.1 状态枚举

```cpp
enum class State {
  kUninitialized = 0,      ///< 未初始化，等待初始位姿
  kRelocalizing,           ///< 重定位初始化中
  kTracking,               ///< 正常跟踪，定位有效
  kTrackingLost,           ///< 跟踪丢失，尝试恢复中
  kLocalizationFailure,    ///< 定位失败，需要人工干预
};
```

### 2.2 状态语义表

| 状态 | 英文名 | 语义 | output_valid | level | 允许的恢复手段 |
|------|--------|------|-------------|-------|---------------|
| **kUninitialized** | UNINITIALIZED | 系统刚启动，无初始位姿 | false | LEVEL_WARNING | 人工初值、重定位、GM |
| **kRelocalizing** | RELOCALIZING | 正在通过重定位获取初始位姿 | false | LEVEL_WARNING | 人工初值、重定位、GM |
| **kTracking** | TRACKING | 正常跟踪，定位有效 | true | LEVEL_NORMAL | 人工纠偏 |
| **kTrackingLost** | TRACKING_LOST | 跟踪丢失，正在尝试恢复 | true（降级） | LEVEL_DEGRADED | 人工恢复、重定位恢复 |
| **kLocalizationFailure** | LOCALIZATION_FAILURE | 定位失败，需要人工干预 | false | LEVEL_FAILURE | 人工恢复 |

### 2.3 状态转移图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                        LocalizationStateMachine                              │
│                                                                             │
│  ┌─────────────┐     onSlamOdom()      ┌─────────────┐                      │
│  │ UNINITIALIZED│ ────────────────────→ │ RELOCALIZING│                      │
│  │ (未初始化)   │  (auto_start=true)    │ (重定位初始化)│                      │
│  └─────────────┘                       └──────┬──────┘                      │
│         ▲                                     │                             │
│         │ onManualPose()                      │ onRelocalizationResult(OK)  │
│         │ onGlobalMatchSuccess()              │ onGlobalMatchSuccess()      │
│         │                                     │ onManualPose()              │
│         └─────────────────────────────────────┘                             │
│                          │                                                  │
│                          ▼                                                  │
│                   ┌─────────────┐                                           │
│                   │  TRACKING   │ ←────────────────────────────┐            │
│                   │ (正常跟踪)   │                              │            │
│                   └──────┬──────┘                              │            │
│                          │                                     │            │
│                          │ onTrackingFailure()                 │            │
│                          │ (GM连续失败/质量退化)                │            │
│                          ▼                                     │            │
│                   ┌─────────────┐                              │            │
│                   │TRACKING_LOST│ ─────────────────────────────┘            │
│                   │ (跟踪丢失,   │  onRelocalizationResult(OK)              │
│                   │  尝试恢复中) │  onManualPose()                          │
│                   └──────┬──────┘                                           │
│                          │                                                  │
│                          │ onRecoveryFailed()                               │
│                          │ (重定位超时/连续失败)                              │
│                          ▼                                                  │
│                   ┌─────────────┐                                           │
│                   │ LOCALIZATION│                                           │
│                   │ _FAILURE   │ ──── onManualPose() ───→ TRACKING          │
│                   │ (定位失败)   │                                           │
│                   └─────────────┘                                           │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 3. 状态转换表

### 3.1 完整状态转换表

| 当前状态 | 事件 | 条件 | 下一状态 | 动作 |
|----------|------|------|----------|------|
| UNINITIALIZED | onSlamOdom | policy.enabled && policy.auto_start && init.use_relocalization_init | RELOCALIZING | 下发 start 命令 |
| UNINITIALIZED | onManualPose | init.use_manual_initialpose | TRACKING | 下发 stop 命令（若启用） |
| UNINITIALIZED | onGlobalMatchSuccess | init.use_global_matcher_init | TRACKING | 下发 stop 命令（若启用） |
| RELOCALIZING | onRelocalizationResult(accepted) | quality_good | TRACKING | 下发 stop 命令 |
| RELOCALIZING | onManualPose | init.use_manual_initialpose | TRACKING | 下发 stop 命令 |
| RELOCALIZING | onGlobalMatchSuccess | init.use_global_matcher_init | TRACKING | 下发 stop 命令 |
| TRACKING | onTrackingFailure | gm_consecutive_failures >= threshold | TRACKING_LOST | 下发 start 命令 |
| TRACKING | onManualPose | - | TRACKING | 更新 map_odom，重置计数 |
| TRACKING_LOST | onRelocalizationResult(accepted) | quality_good | TRACKING | 下发 stop 命令，重置计数 |
| TRACKING_LOST | onManualPose | - | TRACKING | 下发 stop 命令，重置计数 |
| TRACKING_LOST | onRecoveryFailed | timeout || reloc_failures >= threshold | LOCALIZATION_FAILURE | 下发 stop 命令 |
| LOCALIZATION_FAILURE | onManualPose | - | TRACKING | 可选下发 start 命令 |

### 3.2 人工定位优先级规则

**设计原则**：`onManualPose()` 在任何状态下都应该被处理。

| 状态 | 人工定位处理 | 说明 |
|------|-------------|------|
| UNINITIALIZED | 直接初始化，进入 TRACKING | 初始化来源之一 |
| RELOCALIZING | 跳过重定位，直接进入 TRACKING | 人工优先 |
| TRACKING | 更新 map_odom，重置失败计数 | 纠偏 |
| TRACKING_LOST | 更新 map_odom，进入 TRACKING | 救命稻草 |
| LOCALIZATION_FAILURE | 更新 map_odom，进入 TRACKING | 最后手段 |

---

## 4. 事件定义

### 4.1 事件枚举

```cpp
enum class Event {
  kSlamOdom,                ///< SLAM 数据到达
  kManualPose,              ///< 人工初值到达
  kRelocalizationResult,    ///< 重定位结果到达
  kGlobalMatchSuccess,      ///< GM 匹配成功
  kTrackingFailure,         ///< 跟踪失败检测
  kRecoveryFailed,          ///< 恢复失败检测
};
```

### 4.2 事件触发条件

| 事件 | 触发条件 | 数据来源 |
|------|----------|----------|
| kSlamOdom | 收到 SLAM 里程计消息 | slam_odom 话题 |
| kManualPose | 收到人工初值消息 | /initialpose 话题 |
| kRelocalizationResult | 收到重定位结果消息 | relocalization_result 话题 |
| kGlobalMatchSuccess | GM 匹配成功且质量达标 | global_matcher_result 话题 |
| kTrackingFailure | GM 连续失败次数 >= 阈值 | 内部计数器 |
| kRecoveryFailed | 重定位超时 或 连续失败次数 >= 阈值 | 内部计数器 |

---

## 5. FusionStatus 字段填充规范

### 5.1 字段定义

```cpp
// FusionStatus.msg
std_msgs/Header header

uint8 LEVEL_NORMAL = 0
uint8 LEVEL_WARNING = 1
uint8 LEVEL_DEGRADED = 2
uint8 LEVEL_FAILURE = 3

uint8 level                    // 状态等级
string state                   // 状态机状态名
bool initialized               // 是否已完成初始化
bool output_valid              // 当前输出是否有效
string failure_reason          // 失败原因（详细描述）

string current_map_id          // 当前地图标识
string active_scene            // 当前场景（预留）

float64 last_dr_age            // DR 最新数据年龄
float64 last_slam_age          // SLAM 最新数据年龄
float64 last_global_matcher_age // GM 最新数据年龄
float64 last_relocalization_age // 重定位最新数据年龄
float64 last_gnss_age          // GNSS 最新数据年龄

float64 position_std           // 位置标准差（置信度）
float64 velocity_std           // 速度标准差
float64 roll_std               // 横滚角标准差
float64 pitch_std              // 俯仰角标准差
float64 yaw_std                // 偏航角标准差

string[] used_sources          // 当前使用的数据源
string[] rejected_sources      // 被拒绝的数据源
string[] reject_reasons        // 拒绝原因

bool is_dr_available
bool is_slam_available
bool is_global_matcher_available
bool is_relocalization_available
bool is_gnss_available
```

### 5.2 各状态字段填充

#### 5.2.1 UNINITIALIZED

```cpp
void fillStatusUninitialized(deeplumin_msgs::FusionStatus& msg) {
  msg.level = deeplumin_msgs::FusionStatus::LEVEL_WARNING;
  msg.state = "UNINITIALIZED";
  msg.initialized = false;
  msg.output_valid = false;
  msg.failure_reason = "waiting for initial pose";
  
  // 不确定度：极大值
  msg.position_std = 100.0;
  msg.velocity_std = 10.0;
  msg.roll_std = 90.0;
  msg.pitch_std = 90.0;
  msg.yaw_std = 180.0;
  
  // 数据源
  msg.used_sources.clear();
  msg.rejected_sources.clear();
  msg.reject_reasons.clear();
  
  // 可用性
  msg.is_slam_available = fusion_.hasSlam();
  msg.is_global_matcher_available = true;
  msg.is_relocalization_available = config_.relocalization_enabled;
}
```

#### 5.2.2 RELOCALIZING

```cpp
void fillStatusRelocalizing(deeplumin_msgs::FusionStatus& msg) {
  msg.level = deeplumin_msgs::FusionStatus::LEVEL_WARNING;
  msg.state = "RELOCALIZING";
  msg.initialized = false;
  msg.output_valid = false;
  msg.failure_reason = "relocalization in progress for initial pose";
  
  // 不确定度：较大值
  msg.position_std = 50.0;
  msg.velocity_std = 5.0;
  msg.roll_std = 45.0;
  msg.pitch_std = 45.0;
  msg.yaw_std = 90.0;
  
  // 数据源
  msg.used_sources.clear();
  msg.used_sources.push_back("relocalization");
  if (have_gm_candidate_) {
    msg.used_sources.push_back("global_matcher_candidate");
  }
  msg.rejected_sources.clear();
  msg.reject_reasons.clear();
  
  // 重定位等待时间
  msg.last_relocalization_age = (ros::Time::now() - relocalizing_since_).toSec();
}
```

#### 5.2.3 TRACKING

```cpp
void fillStatusTracking(deeplumin_msgs::FusionStatus& msg) {
  msg.level = deeplumin_msgs::FusionStatus::LEVEL_NORMAL;
  msg.state = "TRACKING";
  msg.initialized = true;
  msg.output_valid = true;
  msg.failure_reason = "";
  
  // 正常精度
  msg.position_std = 0.1;
  msg.velocity_std = 0.5;
  msg.roll_std = 0.5;
  msg.pitch_std = 0.5;
  msg.yaw_std = 1.0;
  
  // 数据源
  msg.used_sources.clear();
  msg.used_sources.push_back("slam_odometry_v2");
  if (fusion_.hasMapOdom()) {
    msg.used_sources.push_back("global_matcher");
  }
  msg.rejected_sources.clear();
  msg.reject_reasons.clear();
}
```

#### 5.2.4 TRACKING_LOST

```cpp
void fillStatusTrackingLost(deeplumin_msgs::FusionStatus& msg) {
  msg.level = deeplumin_msgs::FusionStatus::LEVEL_DEGRADED;
  msg.state = "TRACKING_LOST";
  msg.initialized = true;
  msg.output_valid = true;  // 降级输出，不是完全无效
  msg.failure_reason = "tracking lost, GM consecutive failures=" + 
                       std::to_string(gm_consecutive_failures_) +
                       ", relocalization attempting recovery";
  
  // 降级精度
  msg.position_std = 5.0;
  msg.velocity_std = 2.0;
  msg.roll_std = 5.0;
  msg.pitch_std = 5.0;
  msg.yaw_std = 10.0;
  
  // 数据源
  msg.used_sources.clear();
  msg.used_sources.push_back("slam_odometry_v2");  // 只有 SLAM
  msg.rejected_sources.clear();
  msg.rejected_sources.push_back("global_matcher");
  msg.reject_reasons.clear();
  msg.reject_reasons.push_back("GM consecutive failures exceeded threshold");
  
  // 重定位恢复状态
  msg.last_relocalization_age = (ros::Time::now() - tracking_lost_since_).toSec();
}
```

#### 5.2.5 LOCALIZATION_FAILURE

```cpp
void fillStatusLocalizationFailure(deeplumin_msgs::FusionStatus& msg) {
  msg.level = deeplumin_msgs::FusionStatus::LEVEL_FAILURE;
  msg.state = "LOCALIZATION_FAILURE";
  msg.initialized = true;  // 历史上曾初始化
  msg.output_valid = false;  // 定位失败，输出无效
  msg.failure_reason = "localization failure: recovery timeout=" + 
                       std::to_string((ros::Time::now() - tracking_lost_since_).toSec()) + "s" +
                       ", reloc failures=" + std::to_string(reloc_consecutive_failures_);
  
  // 完全不确定
  msg.position_std = 100.0;
  msg.velocity_std = 10.0;
  msg.roll_std = 90.0;
  msg.pitch_std = 90.0;
  msg.yaw_std = 180.0;
  
  // 数据源
  msg.used_sources.clear();
  msg.rejected_sources.clear();
  msg.rejected_sources.push_back("global_matcher");
  msg.rejected_sources.push_back("relocalization");
  msg.reject_reasons.clear();
  msg.reject_reasons.push_back("GM tracking lost");
  msg.reject_reasons.push_back("relocalization recovery failed");
}
```

---

## 6. 质量判断逻辑

### 6.1 判断层次

```
┌─────────────────────────────────────────────────────────────────┐
│                    定位质量判断层次                                │
├─────────────────────────────────────────────────────────────────┤
│  层次 1：初始化判断                                               │
│  ├── 是否有初始位姿？                                            │
│  └── map_odom 是否已设置？                                       │
├─────────────────────────────────────────────────────────────────┤
│  层次 2：数据源可用性判断                                         │
│  ├── SLAM 数据是否超时？                                         │
│  ├── GM 数据是否超时？                                           │
│  └── 重定位是否可用？                                            │
├─────────────────────────────────────────────────────────────────┤
│  层次 3：质量指标判断                                             │
│  ├── GM 连续失败次数？                                           │
│  ├── GM 匹配质量（fitness/inlier）？                             │
│  └── 重定位恢复是否成功？                                        │
└─────────────────────────────────────────────────────────────────┘
```

### 6.2 GM 匹配质量判断

```cpp
struct GmQualityConfig {
  double max_fitness_score = 1.0;      // fitness 分数上限
  double min_inlier_ratio = 0.3;       // inlier 比率下限
  double max_jump_translation = 5.0;   // 最大平移跳变（米）
  double max_jump_yaw_deg = 50.0;      // 最大偏航跳变（度）
};

bool isGlobalMatchQualityGood(const GlobalMatchUpdate& update,
                               const Pose3d& current_map_odom,
                               const GmQualityConfig& config) {
  // 1. 基础成功判断
  if (!update.success) {
    return false;
  }
  
  // 2. fitness 分数判断
  if (update.fitness_score > config.max_fitness_score) {
    return false;
  }
  
  // 3. inlier 比率判断
  if (update.inlier_ratio < config.min_inlier_ratio) {
    return false;
  }
  
  // 4. 与当前位姿的一致性判断（跳变门限）
  if (current_map_odom.has_value()) {
    const double jump_dx = translationDiff(current_map_odom.value(), update.pose);
    const double jump_dyaw = yawDiffDeg(current_map_odom.value(), update.pose);
    if (jump_dx > config.max_jump_translation) {
      return false;
    }
    if (std::abs(jump_dyaw) > config.max_jump_yaw_deg) {
      return false;
    }
  }
  
  return true;
}
```

### 6.3 重定位质量判断

```cpp
struct RelocQualityConfig {
  double max_fitness_score = 1.0;      // fitness 分数上限
  double min_inlier_ratio = 0.5;       // inlier 比率下限
  double max_inlier_rmse = 0.8;        // inlier RMSE 上限
};

bool isRelocalizationQualityGood(const RelocalizationSignal& signal,
                                  const RelocQualityConfig& config) {
  if (!signal.success) {
    return false;
  }
  if (signal.fitness_score > config.max_fitness_score) {
    return false;
  }
  if (signal.inlier_ratio < config.min_inlier_ratio) {
    return false;
  }
  if (signal.inlier_rmse > config.max_inlier_rmse) {
    return false;
  }
  return true;
}
```

### 6.4 跟踪失败判断

```cpp
void checkTrackingFailure(bool gm_success, bool gm_quality_good) {
  if (state_ != State::kTracking) {
    return;
  }
  
  // 失败计数
  if (!gm_success || !gm_quality_good) {
    gm_consecutive_failures_++;
  } else {
    gm_consecutive_failures_ = 0;  // 成功时重置
  }
  
  // 判断是否进入跟踪丢失
  if (gm_consecutive_failures_ >= config_.gm_failure_threshold) {
    transitionTo(State::kTrackingLost);
    tracking_lost_since_ = ros::Time::now();
    reloc_consecutive_failures_ = 0;  // 重置重定位失败计数
    request(RelocCommand::kStart);    // 启动重定位恢复
  }
}
```

### 6.5 恢复失败判断

```cpp
void checkRecoveryFailed() {
  if (state_ != State::kTrackingLost) {
    return;
  }
  
  const double elapsed = (ros::Time::now() - tracking_lost_since_).toSec();
  const bool timeout = elapsed >= config_.reloc_timeout_sec;
  const bool max_failures = reloc_consecutive_failures_ >= config_.reloc_failure_threshold;
  
  if (timeout || max_failures) {
    transitionTo(State::kLocalizationFailure);
    request(RelocCommand::kStop);
  }
}
```

---

## 7. 配置参数

### 7.1 配置结构体

```cpp
struct LocalizationV2Config {
  // ... 现有配置 ...
  
  // 跟踪失效检测
  int gm_failure_threshold = 10;           // GM 连续失败阈值
  double gm_failure_check_window_sec = 5.0; // 失效检测时间窗口
  
  // 重定位恢复
  int reloc_failure_threshold = 5;         // 重定位连续失败阈值
  double reloc_timeout_sec = 30.0;         // 重定位超时时间
  
  // 降级输出
  bool allow_degraded_output = true;       // TRACKING_LOST 时是否继续输出
  
  // GM 质量门限
  double gm_max_fitness_score = 1.0;       // GM fitness 分数上限
  double gm_min_inlier_ratio = 0.3;        // GM inlier 比率下限
};
```

### 7.2 YAML 配置文件

```yaml
# localization_v2.yaml

# 跟踪失效检测
tracking_failure:
  gm_failure_threshold: 10          # GM 连续失败阈值
  gm_failure_check_window_sec: 5.0  # 失效检测时间窗口

# 重定位恢复
recovery:
  reloc_failure_threshold: 5        # 重定位连续失败阈值
  reloc_timeout_sec: 30.0           # 重定位超时时间

# 降级输出
degraded:
  allow_degraded_output: true       # TRACKING_LOST 时是否继续输出

# GM 质量门限
global_matcher:
  max_fitness_score: 1.0            # GM fitness 分数上限
  min_inlier_ratio: 0.3             # GM inlier 比率下限
```

---

## 8. 接口设计

### 8.1 状态机接口

```cpp
class LocalizationStateMachine {
 public:
  enum class State {
    kUninitialized = 0,
    kRelocalizing,
    kTracking,
    kTrackingLost,
    kLocalizationFailure,
  };
  
  enum class RelocCommand {
    kNone = 0,
    kStart,
    kStop,
  };
  
  // 配置
  void configure(const InitConfig& init, const RelocalizationPolicy& policy);
  
  // 状态查询
  State state() const;
  bool initialized() const;
  bool relocalizationRunning() const;
  bool trackingLost() const;
  bool localizationFailure() const;
  const std::string& stateName() const;
  
  // 命令取走
  RelocCommand takeCommand();
  
  // 事件处理
  void onSlamOdom();
  void onManualPose();
  void onRelocalizationResult(bool accepted);
  void onGlobalMatchSuccess();
  void onTrackingFailure();      // 新增
  void onRecoveryFailed();       // 新增
  
 private:
  void transitionTo(State state);
  void request(RelocCommand command);
  
  InitConfig init_;
  RelocalizationPolicy policy_;
  State state_ = State::kUninitialized;
  std::string state_name_ = "UNINITIALIZED";
  RelocCommand pending_command_ = RelocCommand::kNone;
};
```

### 8.2 失效检测器接口

```cpp
class TrackingFailureDetector {
 public:
  void configure(const TrackingFailureConfig& config);
  
  // 输入事件
  void onGlobalMatchResult(bool success, bool quality_good);
  void onRelocalizationResult(bool success);
  
  // 输出
  bool isTrackingFailed() const;
  bool isRecoveryFailed() const;
  int getGmConsecutiveFailures() const;
  int getRelocConsecutiveFailures() const;
  
  // 重置
  void reset();
  void resetGmFailures();
  void resetRelocFailures();
  
 private:
  TrackingFailureConfig config_;
  int gm_consecutive_failures_ = 0;
  int reloc_consecutive_failures_ = 0;
};
```

---

## 9. 实现注意事项

### 9.1 线程安全

当前 localization_v2 是单线程 ROS 节点，所有回调串行执行，无需额外锁。但如果未来改为多线程，需要注意：
- 状态机状态的原子性
- 计数器的原子性
- 配置参数的读写锁

### 9.2 日志规范

| 事件 | 日志级别 | 内容 |
|------|----------|------|
| 状态切换 | INFO | `[localization_v2] state transition: %s -> %s, reason=%s` |
| GM 失败检测 | WARN | `[localization_v2] GM consecutive failures=%d/%d` |
| 跟踪丢失 | WARN | `[localization_v2] tracking lost, starting recovery` |
| 重定位恢复成功 | INFO | `[localization_v2] recovery successful, back to TRACKING` |
| 重定位恢复失败 | WARN | `[localization_v2] recovery failed, reloc failures=%d/%d` |
| 定位失败 | ERROR | `[localization_v2] localization failure, manual intervention required` |
| 人工初值恢复 | INFO | `[localization_v2] recovered from manual pose` |

### 9.3 状态转移防抖

为避免状态抖动，建议：
- GM 连续失败计数使用滑动窗口，而非简单累加
- 重定位恢复成功后，等待 GM 连续成功 N 次再确认恢复
- 状态切换后设置最小驻留时间

### 9.4 向后兼容

- 新增状态但不删除旧状态
- 新增配置参数有默认值，不影响现有配置
- 状态名字符串保持一致（UNINITIALIZED, RELOCALIZING, TRACKING）

### 9.5 单元测试

状态机是纯 C++ 代码，应补充以下测试用例：

```cpp
// 测试用例示例
TEST(LocalizationStateMachineTest, InitialState) {
  LocalizationStateMachine sm;
  EXPECT_EQ(sm.state(), LocalizationStateMachine::State::kUninitialized);
}

TEST(LocalizationStateMachineTest, TrackingToTrackingLost) {
  LocalizationStateMachine sm;
  // 先初始化到 TRACKING 状态
  sm.onSlamOdom();
  sm.onRelocalizationResult(true);
  EXPECT_EQ(sm.state(), LocalizationStateMachine::State::kTracking);
  
  // 模拟跟踪失败
  sm.onTrackingFailure();
  EXPECT_EQ(sm.state(), LocalizationStateMachine::State::kTrackingLost);
}

TEST(LocalizationStateMachineTest, ManualPoseAlwaysWork) {
  LocalizationStateMachine sm;
  
  // 在任何状态下，人工初值都应该有效
  sm.onManualPose();
  EXPECT_EQ(sm.state(), LocalizationStateMachine::State::kTracking);
}
```

---

## 10. 实现步骤

### 10.1 第一步：扩展状态机

1. 修改 `initialization.hpp`，添加新状态和事件
2. 修改 `initialization.cpp`，实现新的状态转移逻辑
3. 补充单元测试

### 10.2 第二步：实现失效检测

1. 在节点层添加失效检测逻辑
2. 添加配置参数
3. 实现 `checkTrackingFailure()` 和 `checkRecoveryFailed()`

### 10.3 第三步：完善状态发布

1. 修改 `publishStatus()` 函数
2. 实现各状态的字段填充逻辑
3. 测试状态输出

### 10.4 第四步：集成测试

1. 测试完整流程：初始化 → 跟踪 → 跟踪丢失 → 恢复
2. 测试人工定位在各状态下的行为
3. 测试超时和失败边界条件

---

## 附录 A：状态转移决策树

```
收到 SLAM 数据
├── 状态 == UNINITIALIZED
│   ├── auto_start && use_relocalization_init
│   │   └── 状态 → RELOCALIZING, 下发 start
│   └── 否则
│       └── 保持 UNINITIALIZED
├── 状态 == TRACKING
│   └── 正常处理，发布融合位姿
└── 其他状态
    └── 正常处理

收到 GM 结果
├── 状态 == TRACKING
│   ├── 成功且质量好
│   │   ├── 更新 map_odom
│   │   ├── 重置 gm_consecutive_failures
│   │   └── 保持 TRACKING
│   └── 失败或质量差
│       ├── gm_consecutive_failures++
│       ├── 若 >= 阈值
│       │   └── 状态 → TRACKING_LOST, 下发 start
│       └── 否则
│           └── 保持 TRACKING
├── 状态 == TRACKING_LOST
│   ├── 成功时重置 gm_consecutive_failures
│   └── 但不直接恢复（需要重定位确认）
└── 其他状态
    └── 按现有逻辑处理

收到重定位结果
├── 状态 == RELOCALIZING
│   ├── 成功且质量好
│   │   └── 状态 → TRACKING, 下发 stop
│   └── 失败或质量差
│       └── 保持 RELOCALIZING
├── 状态 == TRACKING_LOST
│   ├── 成功且质量好
│   │   ├── 状态 → TRACKING, 下发 stop
│   │   └── 重置所有计数器
│   └── 失败或质量差
│       ├── reloc_consecutive_failures++
│       └── 继续等待
└── 其他状态
    └── 忽略

收到人工初值
├── 状态 == 任意
│   ├── 更新 map_odom
│   ├── 状态 → TRACKING
│   ├── 下发 stop（若在重定位中）
│   └── 重置所有计数器
└── 结束

定时器检查
├── 状态 == TRACKING_LOST
│   ├── 超时 || reloc_consecutive_failures >= 阈值
│   │   └── 状态 → LOCALIZATION_FAILURE, 下发 stop
│   └── 否则
│       └── 继续等待
└── 其他状态
    └── 正常处理
```

---

## 附录 B：术语表

| 术语 | 定义 |
|------|------|
| map 系 | 全局地图坐标系 |
| odom 系 | SLAM 局部坐标系 |
| base 系 | 车辆本体坐标系 |
| map->odom | 全局到局部坐标系的修正变换 |
| map->base | 车辆在全局地图中的位姿 |
| odom->base | 局部里程计位姿 |
| GM | Global Matcher，全局匹配器 |
| fitness_score | 匹配适应度分数，越小越好 |
| inlier_ratio | 内点比率，越大越好 |
| inlier_rmse | 内点均方根误差，越小越好 |
| 连续失败 | 连续 N 次匹配失败或质量不达标 |
| 跟踪丢失 | GM 连续失败导致无法更新 map_odom |
| 定位失败 | 重定位恢复也失败，需要人工干预 |
| 降级输出 | 定位精度下降但仍继续输出 |
