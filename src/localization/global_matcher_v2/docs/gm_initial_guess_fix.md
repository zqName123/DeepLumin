# GM 初值选择逻辑修复

## 日期
2026-09-16

## 问题描述

里程计漂移时，GM 仍使用上一次成功的匹配结果（`last_map_source_`）作为初值，
搜索范围锁定在旧位置，无法检测到漂移后的匹配失败，导致 localization_v2 状态
永远停在 TRACKING。

## 根因

`global_matcher_v2_node.cpp` 的 `selectInitialGuess()` 函数中，TRACKING 模式下
`last_map_source_` 优先级高于 `tracking_map_base`（fused_odom）。

`last_map_source_` 是一个固定的 map→source 变换，不随里程计漂移更新。
当 odom_base 变化时，初值 = last_map_source_ * odom_base 仍在旧位置附近，
GICP 从旧位置出发能找到局部匹配，质量门控全部通过，GM 报告 success。

## 修改内容

### 文件：`global_matcher_v2/src/ros/global_matcher_v2_node.cpp`

**函数**：`selectInitialGuess()`

**修改**：TRACKING 模式下，优先使用 `tracking_map_base`（fused_odom）计算初值，
`last_map_source_` 降级为 fallback（仅在无 tracking 数据时使用）。

**修改前**（原始逻辑）：
```cpp
if (have_last_map_source_) {
    guess.map_source = last_map_source_;   // 优先用上次成功结果
    return guess;
}
map_base = tracking_map_base;              // 只有从未成功过才走到
```

**修改后**（修复逻辑）：
```cpp
if (have_tracking) {
    map_base = tracking_map_base;          // 优先用 fused_odom
    have_map_base = true;
} else if (have_last_map_source_) {
    guess.map_source = last_map_source_;   // fallback：无 tracking 时用上次结果
    return guess;
}
```

### 效果

| 场景 | 修改前 | 修改后 |
|------|--------|--------|
| 里程计漂移 | 初值不变，GM 在旧位置匹配 → 虚假 success | 初值跟着漂移，GM 在当前位置匹配 → 正确 success/failure |
| 正常跟踪 | 初值 = 上次结果，匹配正常 | 初值 = fused_odom，匹配正常 |
| GM 从未成功 | 用 fused_odom（无变化） | 用 fused_odom（无变化） |
| 无 tracking 数据 | 用 last_map_source_（无变化） | 用 last_map_source_（无变化） |

## 回退方法

如果此修改引入问题，将 `selectInitialGuess()` 中的 if-else 优先级恢复即可：

```cpp
// 恢复为 last_map_source_ > tracking_map_base
if (have_last_map_source_) {
    guess.reason = "tracking:last_result";
    guess.map_source = last_map_source_;
    ...
    return guess;
}
guess.reason = "tracking:fused_odom";
map_base = tracking_map_base;
```

即把 `if (have_tracking)` 和 `else if (have_last_map_source_)` 的顺序对调。

## 相关配置

- `runtime.use_tracking_initial_pose: true` — 启用跟踪初值（不影响此修改）
- `runtime.source_cloud_is_odom_frame: true` — 源点云在 odom 系（影响 map_source 计算）

## 验证方法

1. 在有里程计漂移的场景下运行，观察 GM 日志中 `mode=` 是否变为 `tracking:fused_odom`
2. 当里程计漂移时，观察 GM 是否能正确报告 `success=false`
3. 观察 localization_v2 状态是否从 TRACKING 正常降级为 TRACKING_LOST
