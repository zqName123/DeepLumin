# LiDAR 退化方向 H 列置零 + 顺序观测更新方案（三期）

- 日期：2026-09-11
- 状态：方案已确认可行，待实现
- 前置文档：`dcreg_huber_design.md`（二期 P 膨胀，已否决）、`lm_slam_observation_refactor_plan.md`
- 目标不变：隧道等长直走廊场景下，LiDAR 旋转（pitch）/z 平移退化时，用轮速 + GNSS 压制 z 轴漂移，且不破坏 LiDAR 匹配

## 1. 背景与已否决方案

### 1.1 已否决：持久 P 膨胀（`kf_.change_P`）

- 现象：进退化区之前 LiDAR 匹配就已错位。
- 根因：膨胀后的 P 持久化 → 下一帧 LiDAR IEKF 以膨胀 P 为先验 → 退化方向增益过大 → 状态被拽偏 → 修正后的状态经 `MapIncremental` 写入 iVox 地图 → 地图被掰斜 → 最近邻关联退化。正反馈闭环。

### 1.2 已否决：瞬态 P 膨胀（P_gain 只参与增益计算）

- 现象：全局 P 未被污染（日志证实 P_rot≈1e-8 与膨胀 P 分离），但进退化区前仍出现轻微错位；z 从向上漂变为向下漂（过矫正）。
- 根因：LiDAR 更新把 P_pitch 压到 1e-7（滤波器"自认为" pitch 已知到 ~0.018°），瞬态 P_gain=0.0087 却让 GNSS 以 k≈0.9 每帧拉 pitch 0.2~0.3°。修正量与滤波器真实信息量矛盾 → 状态-地图不一致，经同样的"状态→地图"路径污染，只是更慢。
- 附带教训：诊断日志曾读全局 P 打印 `gain~1e-5`，而实际 K 用的是 P_gain（真实 k≈0.9），严重误导调试。**诊断必须读取实际参与 K 计算的 P。**

### 1.3 问题的本质

faster-lio 的 LiDAR 更新是信息形式：

```
P⁺ = (P⁻¹ + HᵀR⁻¹H)⁻¹
```

弱约束方向（cond 13~25，即 λ_min/λ_max≈1/25）的 H 列虽小但非零，更新"全信"它 → P 被压到 1e-8 量级的假自信 → 后续轮速/GNSS 增益 k=p/(p+r)≈1e-5，外部观测彻底失效。

**任何"事后改 P"都无法恢复信息簿记的一致性。** 必须在信息进入滤波器之前拦截：不让 LiDAR 在退化方向产生假自信。这正是本方案的核心。

## 2. 方案（三阶段顺序更新，全部为标准 EKF 形式）

```
IMU 预测 → ① LiDAR 更新（退化方向 H 列置零）→ ② 轮速更新 → ③ GNSS 姿态更新
```

### 2.1 阶段①：LiDAR 更新（改动点）

在 `h_share_model → ObsModelWithDCReg → ApplyDCRegAndWeights` 内，DCReg 检测与分数平滑（已有）之后：

- **退化方向 H 列置零**：scores 顺序 `[roll, pitch, yaw, x, y, z]` 对应 h_x 列 `[3, 4, 5, 0, 1, 2]`（h_x 前 6 列为 [pos_x,pos_y,pos_z, rot_x,rot_y,rot_z]，已由现有重排代码确认）。
- 判据：`smooth_degeneracy_scores_[d] > 0`（即 cond > 阈值 25）→ 该列置零。平滑分数已提供时间连续性，避免阈值附近抖动。
- **"增大退化方向 R"的等价实现**：信息形式中 HᵀR⁻¹H，把方向 d 的 R→∞ ⟺ 该列置零。faster-lio 使用单一标量 R（LASER_POINT_COV），无法按方向调 R，**列置零即其等价形式**，无需单独 R 旋钮。
- 保留 Huber 行权重；**关闭行级 DegeneracyWeight**（与列置零构成双重抑制，冗余）。
- 每次 IEKF 迭代重算（h_share_model 每迭代调用一次，DCReg 已如此），随迭代线性化点自适应。

**效果**：P⁺ 在退化方向 = 先验值（不再被 LiDAR 压缩），跨帧按过程噪声自然增长。轮速/GNSS 增益按真实 p 自发上升——这是"诚实的不确定性"，滤波一致性保持。

### 2.2 阶段②：轮速更新（已有实现，不动）

- 先验 = LiDAR 后验 (x, P)；3D 世界系速度伪观测，H@[cols 12-14] = -I，R = 0.1089（σ=0.33 m/s）。
- `update_single_observation` 标准 KF 更新。
- 退化 z 方向 P_vel 自然增长 → 轮速话语权自发出现，无需任何人为授权。

### 2.3 阶段③：GNSS 姿态更新（已有实现，不动）

- 3D 相对姿态观测（首帧基准 + 全维含 yaw），H@[cols 3-5] = I，R = 0.03（对齐 lm_slam `ObsModelGPS` 内部实际生效值，非调用处传入的 1e-3）。
- P_pitch 自然增长后 k_pitch 自发 0 → 0.2~0.4，渐进拉回。

### 2.4 代码清理（瞬态膨胀机制全部移除）

| 位置 | 内容 |
|---|---|
| [laser_odometry.h L327-328](file:///home/hhy/2004ros/DeepLumin/src/localization/slam_odometry_v2/include/faster_lio/laser_odometry.h) | 删 `transient_inflated_P_`、`has_transient_inflation_` |
| [laser_odometry.h L191-205](file:///home/hhy/2004ros/DeepLumin/src/localization/slam_odometry_v2/include/faster_lio/laser_odometry.h) | 删死代码声明 `DetectDegeneracy()`、`InflateDegenerateP()`、`InflateDegeneratePFromDCReg()` |
| [laser_odometry.cc L269/L330/L488](file:///home/hhy/2004ros/DeepLumin/src/localization/slam_odometry_v2/src/faster_lio/laser_odometry.cc) | 删对应三个函数实现（均无调用点，已 grep 确认） |
| [laser_odometry.cc L622/L756](file:///home/hhy/2004ros/DeepLumin/src/localization/slam_odometry_v2/src/faster_lio/laser_odometry.cc) | 两处 `update_single_observation` 调用去掉 P_gain 参数 |
| [laser_odometry.cc L1019-1025](file:///home/hhy/2004ros/DeepLumin/src/localization/slam_odometry_v2/src/faster_lio/laser_odometry.cc) | Run() 中瞬态膨胀调用块删除 |
| [esekfom.hpp L1848+](file:///home/hhy/2004ros/DeepLumin/src/localization/slam_odometry_v2/include/faster_lio/IKFoM_toolkit/esekfom/esekfom.hpp) | `update_single_observation` 还原原始签名（去掉 P_gain） |

### 2.5 配置（slam_odometry_v2.yaml）

```yaml
use_vel: true                    # 恢复轮速（三阶段完整链路）
use_degeneracy_weight: false     # 行级降权关闭，由列置零接管
dcreg_condition_threshold: 25.0  # 保持：平直段 cond 13~19 完全不干预
gnss_attitude_noise: 0.03        # 保持（lm_slam 实际生效值）
wheel_velocity_noise: 0.1089     # 保持（σ=0.33 m/s）
```

## 3. 预期行为与数值

- **平直段**（cond 13~19 < 25）：无任何干预，行为 == 全关基线（该基线已验证匹配正确）。
- **退化段**：pitch 列置零后 P_pitch 每帧增长 ΔP ≈ gyr_cov·dt² ≈ 0.01 rad² → 2~3 帧内 k≈0.25~0.4；稳态 k_eq ≈ √(Q·R)/(√(Q·R)+R) 量级 ≈ 0.3~0.4。GNSS 平滑 pitch（50 帧窗）作为参考真值。
- **全退化**（DCReg 分解失败）：6 列全零 → LiDAR 观测零贡献 → 纯 IMU 预测 + 轮速 + GNSS 推进（正确的兜底行为）。
- z 轴机制：GNSS 姿态约束 pitch → 切断"pitch 误差 × 前向速度 → z 漂移"的积分路径；轮速约束前向速度兜底。

## 4. 验证清单

1. 平直段：无列置零日志，k_rot≈1e-5，匹配与全关基线一致。
2. 退化段：k_rot(pitch) 从 0 自然爬升到 0.2~0.4；无任何 "P inflated" 日志。
3. 进退化区之前无轻微错位（对比 1.2 节现象是否消失）。
4. z 漂移被压制且无反向过矫正（对比 1.2 节的下漂）。
5. 轨迹与 lm_slam 对比。

## 5. 备用调参（按序启用，先验证再调）

1. 接管太快/太慢 → 调 `dcreg_condition_threshold`（25 → 15 或 40）。
2. 硬置零过激 → 改连续缩放 `col_scale = 1 − smooth_score_d`（保留部分弱信息）。
3. GNSS pitch 噪声主导 → 平滑窗 50 → 15，并加日志对比 LIO pitch / GNSS 原始 pitch / 平滑后 pitch。
4. P 增长过快 → 下调 `mapping.gyr_cov` / `b_gyr_cov`。
