# localization_v2 状态机实现指南

## 1. 修改文件清单

### 1.1 需要修改的文件

| 文件 | 修改内容 | 优先级 |
|------|----------|--------|
| `include/localization_v2/core/initialization.hpp` | 扩展状态机类 | P0 |
| `src/core/initialization.cpp` | 实现新的状态转移逻辑 | P0 |
| `src/ros/localization_v2_node.cpp` | 添加失效检测和状态发布 | P0 |
| `include/localization_v2/ros/localization_v2_config.hpp` | 添加新配置参数 | P1 |
| `src/ros/localization_v2_config.cpp` | 加载新配置参数 | P1 |
| `config/localization_v2.yaml` | 添加新配置项 | P1 |

### 1.2 需要新建的文件（可选）

| 文件 | 用途 |
|------|------|
| `include/localization_v2/core/tracking_failure_detector.hpp` | 失效检测器头文件 |
| `src/core/tracking_failure_detector.cpp` | 失效检测器实现 |
| `test/test_state_machine.cpp` | 状态机单元测试 |

---

## 2. initialization.hpp 修改

### 2.1 修改后的完整文件

```cpp
#pragma once

/**
 * @file initialization.hpp
 * @brief 定位状态机（纯 C++，不依赖 ROS）
 *
 * 状态机管理整个定位生命周期：
 * 1. 未初始化 → 重定位初始化 → 正常跟踪
 * 2. 跟踪丢失 → 重定位恢复 → 正常跟踪
 * 3. 恢复失败 → 定位失败 → 人工干预
 *
 * 设计原则：
 * - 人工定位在任何状态下都有效
 * - 跟踪丢失时自动触发重定位恢复
 * - 降级输出比完全无输出更安全
 */

#include <string>

namespace localization_v2 {

/// 重定位命令
enum class RelocCommand {
  kNone = 0,  ///< 无动作
  kStart,     ///< 请求启动重定位
  kStop,      ///< 请求停止重定位
};

/// 初始化状态机配置
struct InitConfig {
  /// 是否必须获得初始位姿后才输出定位
  bool require_initial_pose = false;
  /// 是否允许使用 relocalization 作为初始位姿来源
  bool use_relocalization_init = true;
  /// 是否允许人工 initial pose 完成初始化 / 重置
  bool use_manual_initialpose = true;
  /// 是否允许 global_matcher 自身成功即视为初始化完成
  bool use_global_matcher_init = true;
};

/// 重定位启停策略
struct RelocalizationPolicy {
  bool enabled = true;
  /// 未初始化且需要初值时，是否自动向重定位模块发出 start 命令
  bool auto_start = true;
  /// 重定位成功（通过门限）后停止
  bool stop_on_success = true;
  /// 收到人工 initial pose 后停止
  bool stop_on_manual_pose = true;
  /// global_matcher 自身成功匹配后停止
  bool stop_on_global_match = true;
};

/// 跟踪失效检测配置
struct TrackingFailureConfig {
  /// GM 连续失败阈值，达到此值触发跟踪丢失
  int gm_failure_threshold = 10;
  /// 重定位连续失败阈值，达到此值触发定位失败
  int reloc_failure_threshold = 5;
  /// 重定位超时时间（秒）
  double reloc_timeout_sec = 30.0;
};

class LocalizationStateMachine {
 public:
  enum class State {
    kUninitialized = 0,      ///< 未初始化，等待初始位姿
    kRelocalizing,           ///< 重定位初始化中
    kTracking,               ///< 正常跟踪，定位有效
    kTrackingLost,           ///< 跟踪丢失，尝试恢复中
    kLocalizationFailure,    ///< 定位失败，需要人工干预
  };

  void configure(const InitConfig& init, 
                 const RelocalizationPolicy& policy,
                 const TrackingFailureConfig& tracking_config);

  // ========== 状态查询 ==========
  State state() const { return state_; }
  bool initialized() const { return state_ == State::kTracking || 
                                     state_ == State::kTrackingLost ||
                                     state_ == State::kLocalizationFailure; }
  bool relocalizationRunning() const { return state_ == State::kRelocalizing; }
  bool trackingLost() const { return state_ == State::kTrackingLost; }
  bool localizationFailure() const { return state_ == State::kLocalizationFailure; }
  const std::string& stateName() const { return state_name_; }

  // ========== 命令取走 ==========
  /// 每次事件处理后返回本周期应下发的重定位命令（取走后自动清零）
  RelocCommand takeCommand();

  // ========== 事件处理 ==========
  
  /// slam 数据到达：未初始化且启用自动启动时触发重定位
  void onSlamOdom();

  /// 人工 initial pose：任何状态下都有效
  void onManualPose();

  /// 重定位结果：成功（且通过外部门限）时完成初始化或恢复
  void onRelocalizationResult(bool accepted);

  /// global_matcher 自身匹配成功：完成初始化
  void onGlobalMatchSuccess();

  /// 跟踪失败检测：GM 连续失败达到阈值
  void onTrackingFailure();

  /// 恢复失败检测：重定位超时或连续失败达到阈值
  void onRecoveryFailed();

  // ========== 计数器查询（用于状态发布） ==========
  int getGmConsecutiveFailures() const { return gm_consecutive_failures_; }
  int getRelocConsecutiveFailures() const { return reloc_consecutive_failures_; }

 private:
  void transitionTo(State state);
  void request(RelocCommand command) { pending_command_ = command; }
  void resetCounters();

  InitConfig init_;
  RelocalizationPolicy policy_;
  TrackingFailureConfig tracking_config_;
  
  State state_ = State::kUninitialized;
  std::string state_name_ = "UNINITIALIZED";
  RelocCommand pending_command_ = RelocCommand::kNone;
  
  // 失效检测计数器
  int gm_consecutive_failures_ = 0;
  int reloc_consecutive_failures_ = 0;
};

}  // namespace localization_v2
```

---

## 3. initialization.cpp 修改

### 3.1 修改后的完整文件

```cpp
#include "localization_v2/core/initialization.hpp"

namespace localization_v2 {

void LocalizationStateMachine::configure(const InitConfig& init, 
                                          const RelocalizationPolicy& policy,
                                          const TrackingFailureConfig& tracking_config) {
  init_ = init;
  policy_ = policy;
  tracking_config_ = tracking_config;
  transitionTo(State::kUninitialized);
  resetCounters();
}

void LocalizationStateMachine::transitionTo(State state) {
  const State old_state = state_;
  state_ = state;
  
  switch (state) {
    case State::kUninitialized:
      state_name_ = "UNINITIALIZED";
      break;
    case State::kRelocalizing:
      state_name_ = "RELOCALIZING";
      break;
    case State::kTracking:
      state_name_ = "TRACKING";
      break;
    case State::kTrackingLost:
      state_name_ = "TRACKING_LOST";
      break;
    case State::kLocalizationFailure:
      state_name_ = "LOCALIZATION_FAILURE";
      break;
  }
  
  // 状态切换时重置相关计数器
  if (old_state != state) {
    if (state == State::kTracking) {
      // 成功恢复，重置所有计数器
      resetCounters();
    } else if (state == State::kTrackingLost) {
      // 进入跟踪丢失，重置重定位失败计数
      reloc_consecutive_failures_ = 0;
    }
  }
}

void LocalizationStateMachine::resetCounters() {
  gm_consecutive_failures_ = 0;
  reloc_consecutive_failures_ = 0;
}

RelocCommand LocalizationStateMachine::takeCommand() {
  const RelocCommand cmd = pending_command_;
  pending_command_ = RelocCommand::kNone;
  return cmd;
}

// ========== 事件处理 ==========

void LocalizationStateMachine::onSlamOdom() {
  // 只在未初始化状态下处理
  if (state_ != State::kUninitialized) {
    return;
  }
  if (!policy_.enabled || !policy_.auto_start || !init_.use_relocalization_init) {
    return;
  }
  transitionTo(State::kRelocalizing);
  request(RelocCommand::kStart);
}

void LocalizationStateMachine::onManualPose() {
  if (!init_.use_manual_initialpose) {
    return;
  }
  
  // 人工定位在任何状态下都有效
  // 根据当前状态决定是否需要下发 stop 命令
  switch (state_) {
    case State::kRelocalizing:
    case State::kTrackingLost:
      // 正在重定位中，需要停止
      if (policy_.enabled && policy_.stop_on_manual_pose) {
        request(RelocCommand::kStop);
      }
      break;
    case State::kLocalizationFailure:
      // 从失败状态恢复，可选重新启动重定位
      if (policy_.enabled) {
        request(RelocCommand::kNone);  // 不自动启动，由外部决定
      }
      break;
    default:
      break;
  }
  
  // 统一转移到 TRACKING
  transitionTo(State::kTracking);
}

void LocalizationStateMachine::onRelocalizationResult(bool accepted) {
  // 根据当前状态处理重定位结果
  switch (state_) {
    case State::kRelocalizing:
      // 初始化阶段的重定位结果
      if (!accepted) {
        return;
      }
      transitionTo(State::kTracking);
      if (policy_.enabled && policy_.stop_on_success) {
        request(RelocCommand::kStop);
      }
      break;
      
    case State::kTrackingLost:
      // 跟踪丢失阶段的重定位结果
      if (accepted) {
        // 恢复成功
        transitionTo(State::kTracking);
        if (policy_.enabled && policy_.stop_on_success) {
          request(RelocCommand::kStop);
        }
      } else {
        // 恢复失败，增加计数
        reloc_consecutive_failures_++;
      }
      break;
      
    default:
      // 其他状态忽略重定位结果
      break;
  }
}

void LocalizationStateMachine::onGlobalMatchSuccess() {
  if (!init_.use_global_matcher_init) {
    return;
  }
  
  // 只在未初始化状态下处理
  if (state_ == State::kUninitialized || state_ == State::kRelocalizing) {
    transitionTo(State::kTracking);
    if (policy_.enabled && policy_.stop_on_global_match) {
      request(RelocCommand::kStop);
    }
  }
  // TRACKING 和 TRACKING_LOST 状态下不处理 GM 成功
  // 因为 GM 可能是偶发成功，需要重定位确认
}

void LocalizationStateMachine::onTrackingFailure() {
  // 只在 TRACKING 状态下处理
  if (state_ != State::kTracking) {
    return;
  }
  
  gm_consecutive_failures_++;
  
  // 判断是否达到阈值
  if (gm_consecutive_failures_ >= tracking_config_.gm_failure_threshold) {
    transitionTo(State::kTrackingLost);
    request(RelocCommand::kStart);  // 启动重定位恢复
  }
}

void LocalizationStateMachine::onRecoveryFailed() {
  // 只在 TRACKING_LOST 状态下处理
  if (state_ != State::kTrackingLost) {
    return;
  }
  
  reloc_consecutive_failures_++;
  
  // 判断是否达到阈值
  if (reloc_consecutive_failures_ >= tracking_config_.reloc_failure_threshold) {
    transitionTo(State::kLocalizationFailure);
    request(RelocCommand::kStop);  // 停止重定位
  }
}

}  // namespace localization_v2
```

---

## 4. localization_v2_node.cpp 修改

### 4.1 添加成员变量

在 `LocalizationV2Node` 类的 private 成员中添加：

```cpp
// ========== 跟踪失效检测 ==========
int gm_consecutive_failures_ = 0;
int reloc_consecutive_failures_ = 0;
ros::Time tracking_lost_since_;
```

### 4.2 修改构造函数

```cpp
LocalizationV2Node(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), config_(loadLocalizationV2Config(pnh_)) {
  // ... 现有代码 ...
  
  // 修改状态机配置，添加 TrackingFailureConfig
  TrackingFailureConfig tracking_config;
  tracking_config.gm_failure_threshold = config_.gm_failure_threshold;
  tracking_config.reloc_failure_threshold = config_.reloc_failure_threshold;
  tracking_config.reloc_timeout_sec = config_.reloc_timeout_sec;
  
  state_machine_.configure(init, policy, tracking_config);
  
  // ... 其余代码 ...
}
```

### 4.3 修改 onGlobalMatch 函数

```cpp
void onGlobalMatch(const deeplumin_msgs::GlobalMatchResult::ConstPtr& msg) {
  // ... 现有解析代码 ...
  
  if (!update.success) {
    // ========== 新增：跟踪失败检测 ==========
    if (state_machine_.state() == LocalizationStateMachine::State::kTracking) {
      gm_consecutive_failures_++;
      ROS_WARN_THROTTLE(1.0, "[localization_v2] GM consecutive failures=%d/%d",
                        gm_consecutive_failures_, config_.gm_failure_threshold);
      
      if (gm_consecutive_failures_ >= config_.gm_failure_threshold) {
        state_machine_.onTrackingFailure();
        dispatchRelocalizationCommand();
        tracking_lost_since_ = ros::Time::now();
        ROS_WARN("[localization_v2] tracking lost, starting relocalization recovery");
      }
    }
    return;
  }
  
  // ========== 新增：质量判断 ==========
  bool quality_good = true;
  if (msg->fitness_score > config_.gm_max_fitness_score) {
    quality_good = false;
  }
  if (msg->inlier_ratio < config_.gm_min_inlier_ratio) {
    quality_good = false;
  }
  
  // ... 现有缓存 GM 候选代码 ...
  
  // TRACKING 状态下：成功时重置失败计数
  if (state_machine_.state() == LocalizationStateMachine::State::kTracking) {
    if (quality_good) {
      gm_consecutive_failures_ = 0;  // 重置计数
    } else {
      gm_consecutive_failures_++;
      if (gm_consecutive_failures_ >= config_.gm_failure_threshold) {
        state_machine_.onTrackingFailure();
        dispatchRelocalizationCommand();
        tracking_lost_since_ = ros::Time::now();
      }
      return;
    }
  }
  
  // ... 现有 TRACKING 阶段处理代码 ...
}
```

### 4.4 修改 onRelocalizationResult 函数

```cpp
void onRelocalizationResult(const RelocalizationSignal& signal) {
  // ========== 修改：允许在多个状态下处理 ==========
  const auto state = state_machine_.state();
  if (state != LocalizationStateMachine::State::kRelocalizing &&
      state != LocalizationStateMachine::State::kTrackingLost) {
    return;
  }
  
  if (!signal.success) {
    // ========== 新增：重定位失败计数 ==========
    if (state == LocalizationStateMachine::State::kTrackingLost) {
      reloc_consecutive_failures_++;
      ROS_WARN_THROTTLE(2.0, "[localization_v2] relocalization failed, consecutive=%d/%d",
                        reloc_consecutive_failures_, config_.reloc_failure_threshold);
      
      if (reloc_consecutive_failures_ >= config_.reloc_failure_threshold) {
        state_machine_.onRecoveryFailed();
        ROS_ERROR("[localization_v2] recovery failed, localization failure");
      }
    }
    return;
  }
  
  // ... 现有质量门限检查 ...
  
  // 通过门限
  reloc_result_pending_ = true;
  // ... 其余代码 ...
  
  // ========== 新增：TRACKING_LOST 状态下的处理 ==========
  if (state == LocalizationStateMachine::State::kTrackingLost) {
    reloc_consecutive_failures_ = 0;  // 重置失败计数
  }
  
  tryCompleteInitialization();
}
```

### 4.5 修改 onTimer 函数

```cpp
void onTimer(const ros::TimerEvent&) {
  if (!fusion_.hasSlam()) {
    publishIdentityMapOdom();
    ROS_WARN_THROTTLE(2.0, "[localization_v2] waiting for slam odom: %s",
                      config_.slam_odom_topic.c_str());
    return;
  }
  
  // ========== 新增：恢复超时检查 ==========
  if (state_machine_.state() == LocalizationStateMachine::State::kTrackingLost) {
    const double elapsed = (ros::Time::now() - tracking_lost_since_).toSec();
    if (elapsed >= config_.reloc_timeout_sec) {
      state_machine_.onRecoveryFailed();
      ROS_ERROR("[localization_v2] recovery timeout (%.1fs >= %.1fs), localization failure",
                elapsed, config_.reloc_timeout_sec);
    }
  }
  
  // 初始化仲裁
  tryCompleteInitialization();
  publishCurrentScanInMap();
  publishOutput();
}
```

### 4.6 修改 publishStatus 函数

```cpp
void publishStatus(const ros::Time& stamp, bool output_valid) {
  deeplumin_msgs::FusionStatus msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = config_.map_frame;
  msg.current_map_id = latest_map_id_;
  
  // ========== 修改：根据状态机设置 level 和 failure_reason ==========
  switch (state_machine_.state()) {
    case LocalizationStateMachine::State::kUninitialized:
      msg.level = deeplumin_msgs::FusionStatus::LEVEL_WARNING;
      msg.state = "UNINITIALIZED";
      msg.initialized = false;
      msg.output_valid = false;
      msg.failure_reason = "waiting for initial pose";
      msg.position_std = 100.0;
      msg.velocity_std = 10.0;
      msg.yaw_std = 180.0;
      break;
      
    case LocalizationStateMachine::State::kRelocalizing:
      msg.level = deeplumin_msgs::FusionStatus::LEVEL_WARNING;
      msg.state = "RELOCALIZING";
      msg.initialized = false;
      msg.output_valid = false;
      msg.failure_reason = "relocalization in progress for initial pose";
      msg.position_std = 50.0;
      msg.velocity_std = 5.0;
      msg.yaw_std = 90.0;
      break;
      
    case LocalizationStateMachine::State::kTracking:
      msg.level = deeplumin_msgs::FusionStatus::LEVEL_NORMAL;
      msg.state = "TRACKING";
      msg.initialized = true;
      msg.output_valid = true;
      msg.failure_reason = "";
      msg.position_std = 0.1;
      msg.velocity_std = 0.5;
      msg.yaw_std = 1.0;
      break;
      
    case LocalizationStateMachine::State::kTrackingLost:
      msg.level = deeplumin_msgs::FusionStatus::LEVEL_DEGRADED;
      msg.state = "TRACKING_LOST";
      msg.initialized = true;
      msg.output_valid = config_.allow_degraded_output;
      msg.failure_reason = "tracking lost, GM consecutive failures=" + 
                           std::to_string(gm_consecutive_failures_) +
                           ", relocalization attempting recovery";
      msg.position_std = 5.0;
      msg.velocity_std = 2.0;
      msg.yaw_std = 10.0;
      break;
      
    case LocalizationStateMachine::State::kLocalizationFailure:
      msg.level = deeplumin_msgs::FusionStatus::LEVEL_FAILURE;
      msg.state = "LOCALIZATION_FAILURE";
      msg.initialized = true;
      msg.output_valid = false;
      msg.failure_reason = "localization failure: recovery timeout" +
                           ", reloc failures=" + std::to_string(reloc_consecutive_failures_);
      msg.position_std = 100.0;
      msg.velocity_std = 10.0;
      msg.yaw_std = 180.0;
      break;
  }
  
  // ========== 修改：设置 used_sources ==========
  msg.used_sources.clear();
  msg.rejected_sources.clear();
  msg.reject_reasons.clear();
  
  if (fusion_.hasSlam()) {
    msg.used_sources.push_back("slam_odometry_v2");
  }
  
  switch (state_machine_.state()) {
    case LocalizationStateMachine::State::kTracking:
      if (fusion_.hasMapOdom()) {
        msg.used_sources.push_back("global_matcher");
      }
      break;
      
    case LocalizationStateMachine::State::kTrackingLost:
      msg.used_sources.push_back("slam_odometry_v2");
      msg.rejected_sources.push_back("global_matcher");
      msg.reject_reasons.push_back("GM consecutive failures exceeded threshold");
      if (config_.relocalization_enabled) {
        msg.used_sources.push_back("relocalization_attempting");
      }
      break;
      
    case LocalizationStateMachine::State::kLocalizationFailure:
      msg.rejected_sources.push_back("global_matcher");
      msg.rejected_sources.push_back("relocalization");
      msg.reject_reasons.push_back("GM tracking lost");
      msg.reject_reasons.push_back("relocalization recovery failed");
      break;
      
    default:
      break;
  }
  
  // 数据源可用性
  msg.is_slam_available = fusion_.hasSlam();
  msg.is_global_matcher_available = true;
  msg.is_relocalization_available = config_.relocalization_enabled;
  
  status_pub_.publish(msg);
}
```

---

## 5. localization_v2_config 修改

### 5.1 在 LocalizationV2Config 中添加新字段

```cpp
struct LocalizationV2Config {
  // ... 现有配置 ...
  
  // ========== 新增：跟踪失效检测 ==========
  int gm_failure_threshold = 10;           // GM 连续失败阈值
  double gm_max_fitness_score = 1.0;       // GM fitness 分数上限
  double gm_min_inlier_ratio = 0.3;        // GM inlier 比率下限
  
  // ========== 新增：重定位恢复 ==========
  int reloc_failure_threshold = 5;         // 重定位连续失败阈值
  double reloc_timeout_sec = 30.0;         // 重定位超时时间
  
  // ========== 新增：降级输出 ==========
  bool allow_degraded_output = true;       // TRACKING_LOST 时是否继续输出
};
```

### 5.2 在 loadLocalizationV2Config 中加载新配置

```cpp
LocalizationV2Config loadLocalizationV2Config(ros::NodeHandle& pnh) {
  LocalizationV2Config config;
  // ... 现有配置加载 ...
  
  // ========== 新增：跟踪失效检测配置 ==========
  pnh.param<int>("tracking_failure/gm_failure_threshold", 
                  config.gm_failure_threshold, 10);
  pnh.param<double>("global_matcher/max_fitness_score", 
                     config.gm_max_fitness_score, 1.0);
  pnh.param<double>("global_matcher/min_inlier_ratio", 
                     config.gm_min_inlier_ratio, 0.3);
  
  // ========== 新增：重定位恢复配置 ==========
  pnh.param<int>("recovery/reloc_failure_threshold", 
                  config.reloc_failure_threshold, 5);
  pnh.param<double>("recovery/reloc_timeout_sec", 
                     config.reloc_timeout_sec, 30.0);
  
  // ========== 新增：降级输出配置 ==========
  pnh.param<bool>("degraded/allow_degraded_output", 
                   config.allow_degraded_output, true);
  
  return config;
}
```

---

## 6. config/localization_v2.yaml 修改

在现有配置文件中添加：

```yaml
# ========== 跟踪失效检测 ==========
tracking_failure:
  gm_failure_threshold: 10          # GM 连续失败阈值

# ========== GM 质量门限 ==========
global_matcher:
  result_pose_frame: "odom"
  enable_jump_guard: true
  max_jump_translation: 5.0
  max_jump_yaw_deg: 50.0
  max_fitness_score: 1.0            # 新增：GM fitness 分数上限
  min_inlier_ratio: 0.3             # 新增：GM inlier 比率下限

# ========== 重定位恢复 ==========
recovery:
  reloc_failure_threshold: 5        # 重定位连续失败阈值
  reloc_timeout_sec: 30.0           # 重定位超时时间

# ========== 降级输出 ==========
degraded:
  allow_degraded_output: true       # TRACKING_LOST 时是否继续输出
```

---

## 7. 测试用例

### 7.1 状态机单元测试

```cpp
// test/test_state_machine.cpp
#include <gtest/gtest.h>
#include "localization_v2/core/initialization.hpp"

using namespace localization_v2;

class LocalizationStateMachineTest : public ::testing::Test {
 protected:
  void SetUp() override {
    InitConfig init;
    init.use_relocalization_init = true;
    init.use_manual_initialpose = true;
    init.use_global_matcher_init = true;
    
    RelocalizationPolicy policy;
    policy.enabled = true;
    policy.auto_start = true;
    policy.stop_on_success = true;
    policy.stop_on_manual_pose = true;
    policy.stop_on_global_match = true;
    
    TrackingFailureConfig tracking_config;
    tracking_config.gm_failure_threshold = 3;
    tracking_config.reloc_failure_threshold = 2;
    tracking_config.reloc_timeout_sec = 10.0;
    
    sm_.configure(init, policy, tracking_config);
  }
  
  LocalizationStateMachine sm_;
};

TEST_F(LocalizationStateMachineTest, InitialState) {
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kUninitialized);
  EXPECT_FALSE(sm_.initialized());
  EXPECT_FALSE(sm_.trackingLost());
  EXPECT_FALSE(sm_.localizationFailure());
}

TEST_F(LocalizationStateMachineTest, SlameOdomTriggersRelocalizing) {
  sm_.onSlamOdom();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kRelocalizing);
  EXPECT_EQ(sm_.takeCommand(), LocalizationStateMachine::RelocCommand::kStart);
}

TEST_F(LocalizationStateMachineTest, ManualPoseAlwaysWork) {
  // 在 UNINITIALIZED 状态
  sm_.onManualPose();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_TRUE(sm_.initialized());
}

TEST_F(LocalizationStateMachineTest, ManualPoseWorkInRelocalizing) {
  sm_.onSlamOdom();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kRelocalizing);
  
  sm_.onManualPose();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), LocalizationStateMachine::RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, TrackingToTrackingLost) {
  // 先进入 TRACKING
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  
  // 模拟 GM 连续失败
  for (int i = 0; i < 3; ++i) {
    sm_.onTrackingFailure();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
  EXPECT_EQ(sm_.takeCommand(), LocalizationStateMachine::RelocCommand::kStart);
}

TEST_F(LocalizationStateMachineTest, TrackingLostToTracking) {
  // 进入 TRACKING_LOST
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  for (int i = 0; i < 3; ++i) {
    sm_.onTrackingFailure();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
  
  // 重定位成功恢复
  sm_.onRelocalizationResult(true);
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
  EXPECT_EQ(sm_.takeCommand(), LocalizationStateMachine::RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, TrackingLostToLocalizationFailure) {
  // 进入 TRACKING_LOST
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  for (int i = 0; i < 3; ++i) {
    sm_.onTrackingFailure();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTrackingLost);
  
  // 重定位连续失败
  for (int i = 0; i < 2; ++i) {
    sm_.onRecoveryFailed();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kLocalizationFailure);
  EXPECT_EQ(sm_.takeCommand(), LocalizationStateMachine::RelocCommand::kStop);
}

TEST_F(LocalizationStateMachineTest, ManualPoseWorkInLocalizationFailure) {
  // 进入 LOCALIZATION_FAILURE
  sm_.onSlamOdom();
  sm_.onRelocalizationResult(true);
  for (int i = 0; i < 3; ++i) {
    sm_.onTrackingFailure();
  }
  for (int i = 0; i < 2; ++i) {
    sm_.onRecoveryFailed();
  }
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kLocalizationFailure);
  
  // 人工初值恢复
  sm_.onManualPose();
  EXPECT_EQ(sm_.state(), LocalizationStateMachine::State::kTracking);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

---

## 8. 编译和测试命令

```bash
# 编译
cd /home/hhy/2004ros/DeepLumin
catkin_make --pkg localization_v2

# 运行测试
source devel/setup.bash
rosrun localization_v2 test_state_machine
```

---

## 9. 注意事项

### 9.1 向后兼容性

- 新增状态（kTrackingLost, kLocalizationFailure）不影响现有逻辑
- 新增配置参数有默认值，不破坏现有配置
- 状态名字符串保持一致

### 9.2 调试建议

- 使用 `rostopic echo /localization_v2/status` 观察状态变化
- 关注 `level` 和 `failure_reason` 字段
- 使用 `rqt_console` 查看日志输出

### 9.3 性能考虑

- 失效检测是轻量级计数器操作，不影响性能
- 状态转移是 O(1) 操作
- 日志使用 `ROS_WARN_THROTTLE` 避免刷屏
