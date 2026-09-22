# global_matcher_v2 syslog 日志替换方案

## 文档目的

将 global_matcher_v2 中的 ROS 日志（`ROS_INFO`/`ROS_WARN`/`ROS_ERROR` 等）替换为 syslog 库日志，实现持久化文件存储与集中排错。

---

## 1. 日志模块划分

| 模块名 | 输出文件 | 存储内容 | 日志级别 |
|--------|---------|---------|---------|
| `"gm_v2_main"` | `gm_v2_main.log` | 正常流程日志（启动、配置、匹配结果） | INFO |
| `"gm_v2_alarm"` | `gm_v2_alarm.log` | 关键告警，快速定位问题（失败、异常、限频告警） | WARNING + ERROR |

生成的文件结构：

```
loclog/
├── gm_v2_main.log       ← 正常信息
└── gm_v2_alarm.log      ← 告警（WARN/ERROR）
```

---

## 2. CMake 集成（不需修改，已添加）

`global_matcher_v2` 的 `CMakeLists.txt` 已依赖 `syslog_ros_wrap`。

---

## 3. main 函数初始化

```cpp
int main(int argc, char** argv) {
  ros::init(argc, argv, "global_matcher_v2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // syslog 初始化
  std::string log_dir;
  pnh.param<std::string>("log_dir", log_dir,
      "/home/hhy/2004ros/DeepLumin_zq/src/localization/loclog");
  SYS_LOG_INIT_FULL(log_dir,
                    syslog::LogSeverity::INFO,
                    50,
                    "gm_v2_main");                  // 默认模块

  GlobalMatcherV2Node node(nh, pnh);
  ros::spin();
  return 0;
}
```

launch 配置：

```xml
<node pkg="global_matcher_v2" type="global_matcher_v2_node" name="global_matcher_v2" output="screen">
  <param name="log_dir" value="$(find localization_v2)/../../loclog" />
</node>
```

> 建议所有定位模块共用 `log_dir`，统一保存到 `loclog/` 目录下。

---

## 4. 日志替换对照表

### 4.1 `global_matcher_v2_node.cpp` — 全员

#### 4.1.1 节点生命周期 — 写入 `gm_v2_main`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L243 | `ROS_ERROR("failed to init map manager mode=%s dir=%s pcd=%s")` | `SYS_LOG_TO("gm_v2_alarm", ERROR, true)` | ✅ | 严重：地图加载失败，升级为告警模块 |
| L326 | `ROS_INFO("map_mode=%s matcher=%s robust=%s result_pose_frame=%s")` | `SYS_LOG_TO("gm_v2_main", INFO, true)` | ✅ 一次性 | 启动配置信息 |
| L378 | `ROS_INFO("received bootstrap initial pose from %s")` | `SYS_LOG_TO("gm_v2_main", INFO, false)` | ❌ | 初始定位信号，低频 |
| L392 | `ROS_INFO("manual initial pose z completed to %.2f")` | `SYS_LOG_TO("gm_v2_main", INFO, false)` | ❌ | 人工初始位姿 z 补全 |
| L408 | `ROS_INFO("received manual initial pose from %s")` | `SYS_LOG_TO("gm_v2_main", INFO, false)` | ❌ | 人工初始位姿接收 |
| L421 | `ROS_INFO("re-anchored last_map_source_ from gm anchor (init arbitration)")` | `SYS_LOG_TO("gm_v2_main", INFO, false)` | ❌ | 仲裁重定位锚点更新 |
| L724 | `ROS_INFO("mode=%s success=%d converged=%d source=%zu target=%zu fitness=%.4f inlier=%.3f rmse=%.3f delta=%.3fm yaw=%.2fdeg cost=%.1fms reason=%s")` | `SYS_LOG_TO("gm_v2_main", INFO, false)` | ❌ | **核心匹配结果日志**——每次匹配都记录 |

#### 4.1.2 关键告警 — 写入 `gm_v2_alarm`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L394 | `ROS_WARN_THROTTLE(2.0, "cannot estimate z from map at manual pose ...")` | `SYS_LOG_TO("gm_v2_alarm", WARNING, true)` | ✅ | 人工位姿 z 估计失败 |
| L427 | `ROS_WARN("chunked mode: chunks are auto-managed, ignore load_map_command")` | `SYS_LOG_TO("gm_v2_alarm", WARNING, false)` | ❌ | 非预期命令 |
| L452 | `ROS_WARN("map segments are auto-managed, ignore command '%s'")` | `SYS_LOG_TO("gm_v2_alarm", WARNING, false)` | ❌ | 非预期命令 |
| L665 | `ROS_WARN_THROTTLE(2.0, "matcher converged=false but quality gates passed ...")` | `SYS_LOG_TO("gm_v2_alarm", WARNING, true)` | ✅ | 异常：未收敛但质量通过 |
| L697 | `ROS_WARN("match failed and result deviates from last_map_source_ ... Discarding ...")` | `SYS_LOG_TO("gm_v2_alarm", WARNING, true)` | ✅ | 匹配失败 + 丢弃历史 |
| L800 | `ROS_WARN_THROTTLE(2.0, "source_odom missing, use map_base as map_source")` | `SYS_LOG_TO("gm_v2_alarm", WARNING, true)` | ✅ | 里程计缺失，使用兜底初值 |

#### 4.1.3 差异点云 & 聚类 — 写入 `gm_v2_main`（高频但重要）

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L1098 | `ROS_INFO("Difference: %zu / %zu not in map, ratio=%.3f !!!!!!")` | `SYS_LOG_TO("gm_v2_main", INFO, false)` | ❌ | ❗ 定位失效标志（差异点过多），**建议升级到告警模块** |
| L1104 | `ROS_INFO("Difference: %zu / %zu not in map, ratio=%.3f")` | `SYS_LOG_TO("gm_v2_main", INFO, false)` | ❌ | 定位有效（差异点在合理范围内） |
| L1125 | `ROS_INFO("start cluster !")` | `SYS_LOG_TO("gm_v2_main", INFO, false)` | ❌ | 聚类开始 |
| L1137 | `ROS_INFO("Cluster: %d obstacles, eps=%.3f, clustered_pts=%zu / raw_diff=%zu")` | `SYS_LOG_TO("gm_v2_main", INFO, false)` | ❌ | 聚类统计结果 |

> **特别建议**：L1098（`!!!!!!!`）应使用 `WARNING` 级别写入 `gm_v2_alarm`，因为它是"定位失效"的关键告警信号，不应淹没在正常日志中。

### 4.2 `map_manager.cpp` — 1 处

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L310 | `ROS_WARN_THROTTLE(2.0, "failed to load chunk id=%d path=%s")` | `SYS_LOG_TO("gm_v2_alarm", WARNING, true)` | ✅ | chunk 加载失败，关键告警 |

### 4.3 已注释的调试日志（建议直接删除）

| 行号 | 注释内容 | 建议 |
|------|---------|------|
| L679–683 | `// ROS_INFO("[GM_DEBUG] match result: ...")` | 直接删除，不再保留 |
| L763–766 | `// ROS_INFO("[GM_DEBUG] use fused_odom as initial guess: ...")` | 直接删除，不再保留 |
| L815–817 | `// std::cout << "tracking_map_base: " << ...` | 直接删除，不再保留 |

---

## 5. 替换工作量统计

| 文件 | 需替换总数 | `gm_v2_main` | `gm_v2_alarm` | 已注释（可删） |
|------|-----------|-------------|--------------|--------------|
| `global_matcher_v2_node.cpp` | 18 处 | 9 处 | 6 处 | 3 处 |
| `map_manager.cpp` | 1 处 | 0 处 | 1 处 | 0 处 |
| **合计** | **19 处** | **9 处** | **7 处** | **3 处** |

推荐分批替换策略：

1. **第一批（P0）**：`gm_v2_alarm` 7 处告警（含 `map_manager.cpp` 的 chunk 加载失败和 `ROS_ERROR`）
2. **第二批（P1）**：`gm_v2_main` 核心日志，特别是 L724 的匹配结果日志（核心数据，调试必备）
3. **第三批（P2）**：启动配置、初始位姿、差异点云聚类等低频信息日志
4. **清理**：删除 3 处已注释的 `[GM_DEBUG]` 代码

---

## 6. 替换示例

### 6.1 核心匹配结果（最关键的日志）

```cpp
// Before
ROS_INFO("[global_matcher_v2] mode=%s success=%d converged=%d source=%zu target=%zu "
         "fitness=%.4f inlier=%.3f rmse=%.3f delta=%.3fm yaw=%.2fdeg cost=%.1fms reason=%s",
         current_mode_.c_str(), success, match_result.fine_converged, source->size(),
         target_pcl_ptr->size(), match_result.fine_fitness, metrics.inlier_ratio,
         metrics.inlier_rmse, delta_translation, delta_yaw * 180.0 / kPi, elapsed_ms,
         reject_reason.c_str());

// After
SYS_LOG_TO("gm_v2_main", syslog::LogSeverity::INFO, false)
    << "mode=" << current_mode_ << " "
    << "success=" << success << " "
    << "converged=" << match_result.fine_converged << " "
    << "source=" << source->size() << " "
    << "target=" << target_pcl_ptr->size() << " "
    << "fitness=" << match_result.fine_fitness << " "
    << "inlier=" << metrics.inlier_ratio << " "
    << "rmse=" << metrics.inlier_rmse << " "
    << "delta=" << delta_translation << "m "
    << "yaw=" << (delta_yaw * 180.0 / kPi) << "deg "
    << "cost=" << elapsed_ms << "ms "
    << "reason=" << reject_reason;
```

### 6.2 告警日志

```cpp
// Before
ROS_WARN_THROTTLE(2.0,
                  "[global_matcher_v2] matcher converged=false but quality gates passed "
                  "(fitness=%.4f inlier=%.3f rmse=%.3f)",
                  match_result.fine_fitness, metrics.inlier_ratio, metrics.inlier_rmse);

// After
SYS_LOG_TO("gm_v2_alarm", syslog::LogSeverity::WARNING, true)
    << "matcher converged=false but quality gates passed "
    << "fitness=" << match_result.fine_fitness << " "
    << "inlier=" << metrics.inlier_ratio << " "
    << "rmse=" << metrics.inlier_rmse;
```

### 6.3 `ROS_ERROR`（失败升级）

```cpp
// Before
ROS_ERROR("[global_matcher_v2] failed to init map manager mode=%s dir=%s pcd=%s",
          config_.map_mode.c_str(), config_.map_dir.c_str(),
          config_.map_pcd_path.c_str());

// After
SYS_LOG_TO("gm_v2_alarm", syslog::LogSeverity::ERROR, true)
    << "failed to init map manager "
    << "mode=" << config_.map_mode << " "
    << "dir=" << config_.map_dir << " "
    << "pcd=" << config_.map_pcd_path;
```

### 6.4 `map_manager.cpp` chunk 加载失败

```cpp
// Before
ROS_WARN_THROTTLE(2.0, "[map_manager] failed to load chunk id=%d path=%s", id,
                  info.path.c_str());

// After
SYS_LOG_TO("gm_v2_alarm", syslog::LogSeverity::WARNING, true)
    << "[map_manager] failed to load chunk "
    << "id=" << id << " "
    << "path=" << info.path;
```

---

## 7. 与 localization_v2 共用 `loclog` 目录

| 模块 | 模块名 | 日志文件 | 说明 |
|------|--------|---------|------|
| `localization_v2` | `loc_v2_main` + `loc_v2_alarm` | `loc_v2_main.log` + `loc_v2_alarm.log` | 详见 localization_v2 方案 |
| `global_matcher_v2` | `gm_v2_main` + `gm_v2_alarm` | `gm_v2_main.log` + `gm_v2_alarm.log` | 本文档 |
| `slam_mapping_v2` | 待定 | 待定 | 后续模块 |

所有模块共用一个 `log_dir`，各模块通过 `module` 参数分文件输出，互不干扰：

```
loclog/
├── loc_v2_main.log
├── loc_v2_alarm.log
├── gm_v2_main.log
├── gm_v2_alarm.log
└── slam_v2_main.log      ← 后续
```