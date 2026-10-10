# localization_v2 syslog 日志替换方案

## 概述

将 localization_v2 中现有的 ROS 日志（`ROS_INFO`/`ROS_WARN`/`ROS_ERROR`/`ROS_DEBUG` 等）替换为 syslog 库日志，实现按模块分文件存储、持久化、集中排错的日志体系。

---

## 1. syslog 库介绍

### 1.1 仓库位置

```
src/system/syslog_ros_wrap/
├── syslog/                # 核心库（纯 C++，独立 CMake 工程）
│   ├── include/syslog/    # 头文件
│   ├── src/               # 实现
│   └── build.sh           # 独立编译安装脚本
├── CMakeLists.txt          # catkin 包装 CMake
└── package.xml             # catkin 包声明
```

`syslog_ros_wrap` 是一个 **catkin 包**，通过 `add_subdirectory(syslog)` 将 syslog 静态库包装为 catkin 组件。

### 1.2 API 速查

| 宏/函数 | 作用 |
|---------|------|
| `SYS_LOG_INIT_FULL(dir, severity, max_mb, module)` | 初始化日志系统（在 main() 中调用一次） |
| `SYS_LOG(severity, print_to_terminal) << ...` | 向默认模块写日志 |
| `SYS_LOG_TO(module, severity, print_to_terminal) << ...` | 向指定模块写日志 |
| `syslog::LogSeverity::INFO` | INFO 级别（0） |
| `syslog::LogSeverity::WARNING` | WARNING 级别（1） |
| `syslog::LogSeverity::ERROR` | ERROR 级别（2），同时输出到 stderr |
| `syslog::LogSeverity::FATAL` | FATAL 级别（3），记录后 `std::abort()` |

### 1.3 宏参数说明

```
SYS_LOG_TO("module_name", syslog::LogSeverity::INFO, false) << "message";
           ↑                ↑                            ↑
       模块名 → 文件名    日志级别                    是否打印到终端
```

| 参数 | 类型 | 含义 | 示例 |
|------|------|------|------|
| `module` | `string` | 模块名，决定写入 `<module>.log` 文件 | `"loc_v2_main"` |
| `severity` | `LogSeverity` | 日志级别，低于 `MinSeverity` 被过滤 | `INFO` / `WARNING` / `ERROR` |
| `print_to_terminal` | `bool` | `true`=同时输出到终端，`false`=仅写文件 | 告警用 `true`，常规用 `false` |

### 1.4 日志文件格式

```
[2026-09-21 14:30:00.123] [INFO] [TID:13965] [file.cpp:42] [FunctionName] 消息内容
└────── 时间戳 ────────┘ └级别─┘ └── 线程ID ──┘ └ 文件:行号 ┘ └ 函数 ┘ └ 消息 ┘
```

---

## 2. 集成步骤（CMake）

### 2.1 添加 catkin 依赖

`CMakeLists.txt` — 在 `find_package(catkin REQUIRED COMPONENTS ...)` 中加入 `syslog_ros_wrap`：

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  deeplumin_msgs
  deeplumin_health_checker
  syslog_ros_wrap                  # ← 新增
)
```

`catkin_package` 中可选添加:

```cmake
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros pcl_ros pcl_conversions deeplumin_msgs syslog_ros_wrap
)
```

链接到节点（`${catkin_LIBRARIES}` 已自动包含 syslog）：

```cmake
target_link_libraries(localization_v2_node
  localization_v2_core
  localization_v2_ros
  ${catkin_LIBRARIES}    # 已包含 libsyslog.a 和 pthread
  ${PCL_LIBRARIES}
)
```

### 2.2 头文件包含

在每个使用 syslog 的 `.cpp` 文件中添加：

```cpp
#include <syslog/logger.h>
```

---

## 3. 初始化（main 函数）

```cpp
int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_v2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // ===== syslog 初始化（必须在第一次 SYS_LOG 之前） =====
  std::string log_dir;
  pnh.param<std::string>("log_dir", log_dir,
      "/home/hhy/2004ros/DeepLumin_zq/src/localization/loclog");  // 默认路径
  SYS_LOG_INIT_FULL(log_dir,
                    syslog::LogSeverity::INFO,    // MinSeverity：记录 INFO 及以上
                    50,                           // MaxFileSize：单文件 50MB 滚动
                    "loc_v2_main");                // 默认模块名
  // ================================================

  localization_v2::LocalizationV2Node node(nh, pnh);
  ros::spin();
  return 0;
}
```

日志路径可通过 launch 文件配置（`launch` 中使用 `$(find ...)` 语法）：

```xml
<node pkg="localization_v2" type="localization_v2_node" name="localization_v2" output="screen">
  <param name="log_dir" value="$(find localization_v2)/../../loclog" />
  <!-- ... -->
</node>
```

> **重要**：ROS launch 文件使用 `$(find pkg_name)`（**圆括号**）语法，不是 `${find}`（花括号）。

---

## 4. 日志模块划分方案

建议按严重程度分两个模块文件，便于排错：

| 模块名 | 输出文件 | 存储内容 | 日志级别 |
|--------|---------|---------|---------|
| `"loc_v2_main"` | `loc_v2_main.log` | 正常流程日志 | INFO |
| `"loc_v2_alarm"` | `loc_v2_alarm.log` | 关键告警，快速定位问题 | WARNING + ERROR |

生成的文件结构：

```
loclog/
├── loc_v2_main.log       ← 正常信息
└── loc_v2_alarm.log      ← 告警（WARN/ERROR）
```

---

## 5. 日志替换对照表

### 5.1 节点生命周期 — `loc_v2_main`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 |
|------|-------------|------------|---------|
| L190 | `ROS_INFO("started slam=... gm=...")` | `SYS_LOG(loc_v2_main, INFO, true)` | ✅ 一次性 |
| L358 | `ROS_INFO("recovered from localization failure ...")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | ❌ |
| L386 | `ROS_INFO("recovered from tracking lost ...")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | ❌ |
| L407 | `ROS_INFO("initialized from global matcher ...")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | ❌ |
| L454 | `ROS_INFO("updated map->odom from global matcher ...")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | ❌ |
| L491 | `ROS_INFO("reset map->odom from manual initial pose ...")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | ❌ |
| L552 | `ROS_INFO("recovered from tracking lost via relocalization ...")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | ❌ |

### 5.2 重定位 adapter — `loc_v2_main`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 |
|------|-------------|------------|---------|
| L14 | `ROS_INFO("relocalization adapter: result=... command=...")` | `SYS_LOG_TO("loc_v2_main", INFO, true)` | ✅ 一次性 |
| L24 | `ROS_INFO("relocalization start requested")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | ❌ |
| L29 | `ROS_INFO("relocalization stop requested")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | ❌ |

### 5.3 质量告警 — `loc_v2_alarm`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 |
|------|-------------|------------|---------|
| L247 | `ROS_WARN_THROTTLE(1.0, "GM consecutive failures=%d/%d")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L254 | `ROS_WARN("tracking lost, starting relocalization recovery")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L274 | `ROS_WARN_THROTTLE(2.0, "GM bad: inlier_rmse=...")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L282 | `ROS_WARN_THROTTLE(2.0, "GM degraded: inlier_ratio=...")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L288 | `ROS_WARN_THROTTLE(2.0, "GM bad: inlier_ratio below degraded_thresh")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L304 | `ROS_WARN("tracking lost due to prolonged degraded GM quality")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L315 | `ROS_WARN("tracking lost due to poor GM quality")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L343 | `ROS_WARN_THROTTLE(2.0, "skip GM recovery: quality too low")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L444 | `ROS_WARN_THROTTLE(1.0, "jump guard triggered: dx=... dy=...")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L472 | `ROS_WARN_THROTTLE(2.0, "unknown initial pose frame")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L484 | `ROS_WARN("invalid manual initial pose, ignored")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L506 | `ROS_WARN_THROTTLE(2.0, "relocalization failed, consecutive=%d/%d")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L511 | `ROS_ERROR("recovery failed, localization failure")` | `SYS_LOG_TO("loc_v2_alarm", ERROR, true)` | ✅ 自动输出到 stderr |
| L517 | `ROS_WARN_THROTTLE(2.0, "relocalization result not accepted")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L522 | `ROS_WARN_THROTTLE(2.0, "relocalization inlier ratio ... below limit")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |
| L527 | `ROS_WARN_THROTTLE(2.0, "relocalization rmse ... above limit")` | `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` | ✅ |

### 5.4 积累过程日志（低频，避免刷屏）— `loc_v2_main`

| 行号 | 原始 ROS 调用 | syslog 替换 | 说明 |
|------|-------------|------------|------|
| L362 | `ROS_INFO_THROTTLE(1.0, "GM recovery accumulating for localization failure")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | 频率由 throttle 保护，改为文件记录 |
| L390 | `ROS_INFO_THROTTLE(1.0, "GM recovery accumulating for tracking lost")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | 同上 |
| L413 | `ROS_INFO_THROTTLE(1.0, "cached global matcher candidate during initializing")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | 同上 |
| L557 | `ROS_INFO_THROTTLE(1.0, "relocalization recovery accumulating")` | `SYS_LOG_TO("loc_v2_main", INFO, false)` | 同上 |

### 5.5 调试日志（当前已注释）— 可选保留

| 行号 | 原始 ROS 调用 | 建议 |
|------|-------------|------|
| L192 | `// ROS_INFO("[LOC_DEBUG] ... started ...")` | 已注释，已被新的 SYS_LOG 替代，可删除 |
| L238 | `// ROS_INFO("[LOC_DEBUG] GM raw result ...")` | 建议直接删除，保留代码作为历史参考 |
| L331 | `// ROS_INFO("[LOC_DEBUG] gm_candidate_map_odom computed ...")` | 建议直接删除 |
| L426 | `// ROS_INFO("[LOC_DEBUG] jump guard check ...")` | 建议直接删除 |
| L374 | `ROS_DEBUG("GM quality bad during recovery, reset counter")` | 改为 `SYS_LOG_TO("loc_v2_alarm", WARNING, true)` 或直接升级为 WARNING |

---

## 6. 关键注意事项

### 6.1 THROTTLE 的处理

syslog 库没有内建的限频（THROTTLE）机制。对于原有 `ROS_WARN_THROTTLE(1.0, ...)` 的地方：

- **告警日志**（P0）：建议直接升级为每次触发都记录，因为告警是低频事件，不需要限频
- **积累过程日志**（P1）：如确实需要限频，可自行封装一个简单的 throttle helper

大多数告警日志（GM 质量退化、jump guard 触发、relocalization 失败等）本身就是低频事件，直接写入文件不存在刷屏风险。

### 6.2 初始化必须在第一次 SYS_LOG 之前

```cpp
// 错误：先调用 SYS_LOG，再初始化
SYS_LOG(INFO, true) << "message";      // ❌ 不会写文件
SYS_LOG_INIT_FULL(...);                 // 太晚了

// 正确
SYS_LOG_INIT_FULL(...);                 // ✅ 先初始化
SYS_LOG(INFO, true) << "message";      // 正常写入文件
```

未初始化时，日志只输出到终端，不写文件。

### 6.3 error 级别自动输出到 stderr

`ERROR` 级别的 `SYS_LOG_TO(..., ERROR, false)` 即使 `print_to_terminal=false`，也会自动输出到 `stderr`。因此告警日志可以统一用 `print_to_terminal=true`（手动控制终端输出），或者依赖 `ERROR` 级别的自动输出行为。

### 6.4 日志路径配置

- 使用 launch 文件的 `$(find pkg_name)` **圆括号**语法
- 通过 `pnh.param` 设置默认值，launch 文件不传参时使用 C++ 代码中的绝对路径作为 fallback

---

## 7. 从 ROS 日志迁移到 syslog 的通用替换模式

### 7.1 ROS_INFO → SYS_LOG_TO

```cpp
// Before
ROS_INFO("[localization_v2] initialized from global matcher map_id=%s inlier=%.3f",
         msg->map_id.c_str(), msg->inlier_ratio);

// After
SYS_LOG_TO("loc_v2_main", syslog::LogSeverity::INFO, false)
    << "initialized from global matcher "
    << "map_id=" << msg->map_id << " "
    << "inlier=" << msg->inlier_ratio;
```

### 7.2 ROS_WARN → SYS_LOG_TO

```cpp
// Before
ROS_WARN("[localization_v2] tracking lost, starting relocalization recovery");

// After
SYS_LOG_TO("loc_v2_alarm", syslog::LogSeverity::WARNING, true)
    << "tracking lost, starting relocalization recovery";
```

### 7.3 ROS_WARN_THROTTLE → SYS_LOG_TO（直接写入，无限频）

```cpp
// Before
ROS_WARN_THROTTLE(1.0, "[localization_v2] GM consecutive failures=%d/%d",
                  count, threshold);

// After
SYS_LOG_TO("loc_v2_alarm", syslog::LogSeverity::WARNING, true)
    << "GM consecutive failures=" << count << "/" << threshold;
```

### 7.4 ROS_ERROR → SYS_LOG_TO

```cpp
// Before
ROS_ERROR("[localization_v2] recovery failed, localization failure");

// After
SYS_LOG_TO("loc_v2_alarm", syslog::LogSeverity::ERROR, true)
    << "recovery failed, localization failure";
```

### 7.5 ROS_DEBUG → 升级为 WARNING 或删除

```cpp
// Before
ROS_DEBUG("[localization_v2] GM quality bad during recovery attempt, reset counter");

// After（建议直接升级为告警）
SYS_LOG_TO("loc_v2_alarm", syslog::LogSeverity::WARNING, true)
    << "GM quality bad during recovery attempt, reset counter";
```

---

## 8. 替换工作量统计

| 文件 | 总日志数 | 需替换数 | 已注释/可删 | 建议模块 |
|------|---------|---------|-----------|---------|
| `localization_v2_node.cpp` | ~40 处 | ~35 处 | ~4 处（已注释） | `loc_v2_main` + `loc_v2_alarm` |
| `relocalization_ros_adapter.cpp` | 3 处 | 3 处 | 0 | `loc_v2_main` |

推荐分批替换策略：
1. **第一批**：P0 关键告警（`loc_v2_alarm`，~16 行），快速落地告警文件写入
2. **第二批**：P1 启动/恢复/更新日志（`loc_v2_main`，~12 行），补全正常流程
3. **第三批**：清理已注释的 `ROS_INFO` 调试代码（4 行）