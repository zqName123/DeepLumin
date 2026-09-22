# relocalization 模块分析与 syslog 日志替换方案

---

## 1. 节点文件区别

### 1.1 `relocalization_node.cpp`

```cpp
int main(int argc, char** argv) {
  ros::init(argc, argv, "relocalization");
  ROS_ERROR("Use global_relocalization_node or bag_eval_node. "
            "Launch files are provided in the relocalization package.");
  return 1;
}
```

**这是一个占位桩文件**，直接输出错误提醒用户改用正确的可执行文件。编译为 `relocalization_node` 可执行文件，但**不实际使用**。

### 1.2 `global_relocalization_node.cpp`

这是 **实际在线重定位节点**，编译为 `global_relocalization_node` 可执行文件。提供完整功能：

- 订阅 LiDAR 点云
- Scan Context 描述子提取 + 数据库检索
- GICP 精配准
- 发布 `RelocalizationResult` 到 `/localization/relocalization/result`
- 支持 `start`/`stop` 命令控制匹配流程

### 1.3 总结

| 文件 | 编译目标 | 用途 | 是否使用 |
|------|---------|------|---------|
| `relocalization_node.cpp` | `relocalization_node` | 占位桩，提示用错节点 | ❌ |
| `global_relocalization_node.cpp` | `global_relocalization_node` | 在线实时重定位 | ✅ |
| `bag_eval_node.cpp` | `bag_eval_node` | 离线 bag 回放评测 | 仅调试 |

---

## 2. localization_v2 实际使用的是哪个？

`localization_v2` 通过 **话题通信** 与重定位模块交互，不直接调用重定位节点：

```
localization_v2 (RelocalizationRosAdapter)
  │
  ├── 订阅：/localization/relocalization/result  ←── global_relocalization_node 发布
  │     (deeplumin_msgs/RelocalizationResult)
  │
  └── 发布：/localization_v2/relocalization/command  ──→ global_relocalization_node 订阅
        (std_msgs/String "start"/"stop")
```

对应的 launch 配置（`localization_v2.launch:96-97`）：

```xml
<param name="relocalization/result_topic" value="/localization/relocalization/result" />
<param name="relocalization/command_topic" value="/localization_v2/relocalization/command" />
```

同理 `global_relocalization.launch` 配置的 `result_topic` 也指向 `/localization/relocalization/result`，topic 匹配对接。

> **结论**：`localization_v2` 依赖于 `global_relocalization_node` 发布的结果话题，通过 `RelocalizationRosAdapter` 接收。同一时间只应启动一种重定位（GM 或 ArUco），因为它们共用同一个 result topic。

---

## 3. 重定位时实际使用的代码文件

当 localization_v2 启动重定位时，参与运行的代码文件链：

```
global_relocalization_node (进程入口)
  │
  ├── src/ros/global_relocalization_node.cpp       # 主逻辑：订阅/回调/发布
  │
  └── library: relocalization_core (librelocalization_core.a)
        │
        ├── src/core/relocalization_core.cpp        # Core 初始化 + relocalize() 入口
        ├── src/core/keyframe_database.cpp          # 关键帧数据库加载与管理
        ├── src/core/common.cpp                     # 点云预处理、坐标转换
        ├── src/descriptor/scan_context_descriptor.cpp  # Scan Context 描述子
        ├── src/descriptor/csc_descriptor.cpp       # CSC 描述子
        ├── src/matcher/gicp_global_matcher.cpp     # GICP 精配准
        ├── src/matcher/vgicp_global_matcher.cpp    # VGICP 精配准
        ├── src/matcher/matcher_preprocess.cpp      # 匹配预处理
        └── src/ros/param_loader.cpp                # 参数加载
```

CMakeLists.txt 中该可执行文件的构建方式：

```cmake
add_executable(global_relocalization_node src/ros/global_relocalization_node.cpp)
target_link_libraries(global_relocalization_node relocalization_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
```

> **注意**：`localization_v2` 侧的 `relocalization_ros_adapter.cpp` 属于 localization_v2 自身代码，不参与重定位计算，只负责接收结果。

---

## 4. 日志替换对照表

### 4.1 日志模块划分

| 模块名 | 输出文件 | 存储内容 | 日志级别 |
|--------|---------|---------|---------|
| `"reloc_main"` | `reloc_main.log` | 正常流程日志（启动、配置、匹配结果） | INFO |
| `"reloc_alarm"` | `reloc_alarm.log` | 关键告警（初始化失败、命令错误、匹配失败） | WARNING + ERROR |

### 4.2 `global_relocalization_node.cpp` — 核心节点

#### 4.2.1 节点生命周期 — 写入 `reloc_main`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L121 | `ROS_INFO_STREAM("Loaded " << core_.descriptor().size() << " descriptors")` | `SYS_LOG_TO("reloc_main", INFO, true)` | ✅ 一次性 | 描述子库加载完成 |
| L122 | `ROS_INFO_STREAM("result_topic=... frame_id=...")` | `SYS_LOG_TO("reloc_main", INFO, true)` | ✅ 一次性 | 配置参数一览 |
| L129 | `ROS_INFO_STREAM("query_frame.rotate_xy_y_negx=... yaw_deg=...")` | `SYS_LOG_TO("reloc_main", INFO, true)` | ✅ 一次性 | 查询帧旋转配置 |

#### 4.2.2 启停命令 — 写入 `reloc_main`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L139 | `ROS_INFO("[global_relocalization] matching enabled (start)")` | `SYS_LOG_TO("reloc_main", INFO, false)` | ❌ | 命令：启动匹配 |
| L141 | `ROS_INFO("[global_relocalization] matching disabled (stop), skip matching flow")` | `SYS_LOG_TO("reloc_main", INFO, false)` | ❌ | 命令：停止匹配 |
| L144 | `ROS_WARN_THROTTLE(2.0, "unknown command %s (expect start/stop)")` | `SYS_LOG_TO("reloc_alarm", WARNING, true)` | ✅ | 未知命令告警 |

#### 4.2.3 匹配结果（核心数据）— 写入 `reloc_main`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L191 | `ROS_WARN_STREAM(status.data)` | `SYS_LOG_TO("reloc_alarm", WARNING, true)` | ✅ | **匹配失败** — 告警，包含详细指标 |
| L214 | `ROS_INFO_STREAM(status.data)` | `SYS_LOG_TO("reloc_main", INFO, false)` | ❌ | **匹配成功** — 正常日志，含 success/candidate/score/fitness 等 |

匹配结果 `status.data` 的内容格式为：

```
success=1 candidate=42 sc_score=0.85 fitness=0.02 inlier_ratio=0.95 inlier_rmse=0.12 inliers=850/1000 gicp_pts=10000/50000 elapsed_ms=234.5
```

### 4.3 `relocalization_core.cpp` — Core 初始化（库层）

#### 4.3.1 关键告警 — 写入 `reloc_alarm`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L36 | `ROS_ERROR("[relocalization] failed to load keyframe database")` | `SYS_LOG_TO("reloc_alarm", ERROR, true)` | ✅ | 严重：关键帧数据库加载失败 |
| L42 | `ROS_ERROR("[relocalization] unsupported descriptor_type: %s")` | `SYS_LOG_TO("reloc_alarm", ERROR, true)` | ✅ | 严重：不支持的描述子类型 |
| L46 | `ROS_ERROR_STREAM("[relocalization] failed to load descriptor database")` | `SYS_LOG_TO("reloc_alarm", ERROR, true)` | ✅ | 严重：描述子数据库加载失败 |
| L54 | `ROS_ERROR("[relocalization] unsupported matcher_type: %s")` | `SYS_LOG_TO("reloc_alarm", ERROR, true)` | ✅ | 严重：不支持的匹配器类型 |

#### 4.3.2 正常流程 — 写入 `reloc_main`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L58 | `ROS_INFO_STREAM("[relocalization] descriptor: " << descriptor_->summary())` | `SYS_LOG_TO("reloc_main", INFO, true)` | ✅ 一次性 | 描述子概要 |
| L59 | `ROS_INFO_STREAM("[relocalization] matcher: " << matcher_->summary())` | `SYS_LOG_TO("reloc_main", INFO, true)` | ✅ 一次性 | 匹配器概要 |

### 4.4 `keyframe_database.cpp` — 关键帧数据库

#### 4.4.1 正常流程 — 写入 `reloc_main`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L50 | `ROS_INFO_STREAM("Loaded global map: ... points, voxel_leaf=...")` | `SYS_LOG_TO("reloc_main", INFO, true)` | ✅ 一次性 | 全局地图加载统计 |

### 4.5 `common.cpp` — 公共工具

#### 4.5.1 告警 — 写入 `reloc_alarm`

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L297 | `ROS_WARN_THROTTLE(10.0, "[preprocessCloud] VoxelGrid would overflow...")` | `SYS_LOG_TO("reloc_alarm", WARNING, true)` | ✅ | 点云预处理参数异常告警（低频，10s 限频） |

### 4.6 `relocalization_node.cpp` — 占位桩

| 行号 | 原始 ROS 调用 | syslog 替换 | 终端打印 | 说明 |
|------|-------------|------------|---------|------|
| L5 | `ROS_ERROR("Use global_relocalization_node or bag_eval_node...")` | `SYS_LOG_TO("reloc_alarm", ERROR, true)` | ✅ | 用户误用此节点时提示 |

### 4.7 `bag_eval_node.cpp` — 离线评测（仅调试用，可暂不替换）

| 行号 | 原始 ROS 调用 | 建议 | 说明 |
|------|-------------|------|------|
| L53 | `ROS_ERROR("bag_file is empty...")` | 保留 | 命令行参数错误，一次性 |
| L66 | `ROS_ERROR_STREAM("bag_use_truth_pose=true but no poses...")` | 保留 | 配置错误 |
| L76 | `ROS_ERROR_STREAM("Failed to open report: ...")` | 保留 | 文件打开失败 |
| L89 | `ROS_ERROR_STREAM("Failed to open bag: ...")` | 保留 | bag 文件打开失败 |
| L95 | `ROS_ERROR_STREAM("No messages on topic ...")` | 保留 | 话题不存在 |
| L104-107 | 4 行 `ROS_INFO_STREAM` 启动配置信息 | 保留 | 一次性输出 |
| L117 | `ROS_ERROR("Bag timestamp range does NOT overlap pose_file")` | 保留 | 时间戳不匹配 |
| L156 | `ROS_WARN_STREAM("Skip empty cloud...")` | 保留 | 空点云跳过 |
| L165 | `ROS_WARN_STREAM_THROTTLE(2.0, "Skip cloud... no pose within...")` | 保留 | 无位姿匹配 |
| L186 | `ROS_INFO_STREAM(topk_ss.str())` | 保留 | top-k 候选打印 |
| L252 | `ROS_INFO_STREAM("bag_query=...")` | 保留 | 每次查询结果 |
| L275 | `ROS_WARN_STREAM("Skip bag_time=...")` | 保留 | 异常跳过 |
| L282 | `ROS_INFO_STREAM("Bag evaluation finished...")` | 保留 | 最终统计 |
| L286 | `ROS_INFO_STREAM("  top1_rate=... topk_rate=...")` | 保留 | 最终统计 |
| L290 | `ROS_WARN("No valid bag queries processed.")` | 保留 | 无有效查询 |
| L293 | `ROS_INFO("Visualization topics remain latched...")` | 保留 | 可视化提示 |

> `bag_eval_node` 是离线评测工具，不参与在线运行。建议单独处理或暂不替换。

---

## 5. 替换工作量统计（线上模块）

| 文件 | 需替换总数 | `reloc_main` | `reloc_alarm` |
|------|-----------|-------------|--------------|
| `global_relocalization_node.cpp` | 7 处 | 5 处 | 2 处 |
| `relocalization_core.cpp` | 6 处 | 2 处 | 4 处 |
| `keyframe_database.cpp` | 1 处 | 1 处 | — |
| `common.cpp` | 1 处 | — | 1 处 |
| `relocalization_node.cpp` | 1 处 | — | 1 处 |
| **合计** | **16 处** | **8 处** | **8 处** |

> `bag_eval_node.cpp` 另有 ~17 处日志，作为离线工具建议单独处理。

---

## 6. 替换示例

### 6.1 匹配结果（成功/失败分流）

```cpp
// Before — 成功时
ROS_INFO_STREAM(status.data);

// Before — 失败时
ROS_WARN_STREAM(status.data);

// After — 成功写入 main，失败写入 alarm
if (result.success) {
  SYS_LOG_TO("reloc_main", syslog::LogSeverity::INFO, false) << status.data;
} else {
  SYS_LOG_TO("reloc_alarm", syslog::LogSeverity::WARNING, true) << status.data;
}
```

### 6.2 Core 初始化失败

```cpp
// Before
ROS_ERROR("[relocalization] failed to load keyframe database");
return false;

// After
SYS_LOG_TO("reloc_alarm", syslog::LogSeverity::ERROR, true)
    << "[relocalization] failed to load keyframe database";
return false;
```

### 6.3 配置启动日志

```cpp
// Before
ROS_INFO_STREAM("Loaded " << core_.descriptor().size() << " descriptors");

// After
SYS_LOG_TO("reloc_main", syslog::LogSeverity::INFO, true)
    << "Loaded " << core_.descriptor().size() << " descriptors";
```

### 6.4 点云预处理告警

```cpp
// Before
ROS_WARN_THROTTLE(10.0, "[preprocessCloud] VoxelGrid would overflow (leaf=%.3f, ...). "
                  "Returning range-filtered cloud without downsampling. ...",
                  leaf, extent_x, extent_y, extent_z);

// After
SYS_LOG_TO("reloc_alarm", syslog::LogSeverity::WARNING, true)
    << "[preprocessCloud] VoxelGrid would overflow "
    << "(leaf=" << leaf << ", extent=" << extent_x << "x" << extent_y << "x" << extent_z << "). "
    << "Returning range-filtered cloud without downsampling. "
    << "Consider reducing max_range (<=50m) or increasing voxel_leaf (>=0.1m).";
```

---

## 7. 数据流总览

```
                        global_relocalization_node
                        ┌──────────────────────────────────────┐
  /ouster/points ─────→ │ onCloud()                            │
                        │   → 点云预处理 (common.cpp)           │
                        │   → Scan Context 描述子提取           │
                        │   → 数据库检索 (top-K)                │
                        │   → GICP 精配准                       │
                        │   → 质量门控                         │
                        │                                      │
  /localization_v2/     │   onCommand("start"/"stop")           │
  relocalization/       │   → 控制 matching_enabled_           │
  command ←──────────── │                                      │
                        │                                      │
                        ├── 发布：reloc_pose (PoseStamped)      │
                        ├── 发布：reloc_status (String)        │
                        ├── 发布：reloc_aligned_scan (PC2)     │
                        ├── 发布：reloc_target_submap (PC2)    │
                        │                                      │
                        └── 发布：/localization/ ──→ localization_v2
                        │       relocalization/    (RelocalizationRosAdapter
                        │       result              → onRelocalizationResult)
                        └──────────────────────────────────────┘
```