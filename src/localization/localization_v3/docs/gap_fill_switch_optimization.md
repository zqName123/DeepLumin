# 统一源 gap 填充与切换连续性优化（问题定位 + 修改方案 + 验收）

> **【已被取代 2026-10-12】** 本文的机制部分（gap 填充 / 三阶段 / 双 glue）已在
> `local_odom_gateway_design.md` 中推翻重构：DR 增量收窄到"接管后才应用"，
> 检测窗推进与观测链分离，源抽象表驱动。**保留本文的"问题定位"与"实测证据"部分**（§1、附录），
> 机制以新文档为准。

> 日期：2026-10-12
> 关联：`dr_odometry_fallback_design.md`（DR 接入总体设计）、`多源里程计接入改造方案.md`（源注册表后续规划）
> 范围：`localization_v3`（core/local_odom_source、ros/source_bridge_node、config/source_bridge.yaml）+ `dr_odometry_v2`（观测门控 3 处小改）
> 状态：代码已改、单测全绿；**bag/实车回归待跑**（见 §5）

---

## 0. 现象与目标

实跑（`use_dr_source:=true`）暴露两个问题：

| # | 现象 | 关闭 DR 后 |
|---|---|---|
| 1 | 纯 SLAM 期轨迹**间隔性毛刺**（偶尔跳变） | 消失 |
| 2 | 切到 DR 瞬间就跳动，且 DR 段效果比未接桥时差 | — |

目标语义（本次以之为准绳）：

1. **纯 SLAM 期 DR 输出不参与**（定位结果只作 DR 观测，DR 输出不进运动链）；
2. **切到 DR 无停顿/滞后**（断流期输出不停冻结，锚定 + DR 增量续推）；
3. **切回 SLAM 无缝**（切换瞬间输出恒等）。

> 三条存在张力：若断流期完全不用 DR，输出必冻结（违反 2）；若正常节拍也让 DR 参与，就会出现毛刺（违反 1）。
> 解法是把 DR 的参与边界**钉在"样本超龄"这一事件上**：正常节拍内纯 SLAM，样本一旦超龄立即用 DR 增量续推，
> 且从"超龄续推"到"DR 接管"是**同一个公式**——切换只是簿记层面的事。

---

## 1. 问题定位

### 1.0 前提：当时源码处于改到一半的不一致状态

`local_odom_source.hpp / .cpp / source_bridge_node.cpp / test_local_odom_source.cpp` 四者 API 互不一致，
`catkin_make --pkg localization_v3` 直接编译失败（`Pose3d` 无 `anchor`/`motion`、`Config` 无 `gap_fill_delay_sec`、
`have_dr_at_slam_` 未声明等）。devel 里的可执行文件是更早一次成功构建的产物。
**因此实跑行为 = 旧二进制行为，当前源码 ≠ 实跑源码**；下文把"实跑证据"与"源码意图"分开陈述。

### 1.1 纯 SLAM 期毛刺（问题 1）——三个成因

**A. gap 阈值卡在 SLAM 周期里（主因）**

- LiDAR 10Hz → SLAM 里程计周期 ≈ 0.1s（`slam_odometry_v2.yaml` `sensors: lidar: 10`）；
- gap 填充设计阈值 **0.12s**（`dr_odometry_v2.yaml` 注释"与桥 gap_fill_delay 对齐"，但该参数在
  `source_bridge.yaml` / `Config` 里**根本不存在**——幽灵参数）；
- 实测 `loclog/source_bridge.log` 5s 心跳里的 `slam_age` 采样：**0.002 ~ 0.124s** 抖动。

⇒ 只要一帧晚 20ms，gap 填充就介入，输出在"纯 SLAM"与"SLAM 锚 + DR 增量"之间反复横跳 → 间隔性台阶/尖刺。
DR 关闭时 `dr_fresh=false`，gap 填充永不启用 → 毛刺消失，与现象完全吻合。

**B. 锚与运动没分开发 → DR 观测自回灌**

设计红线（本仓库文档早已写明"实测该环会造成大幅抖动"）：`source_anchor` 必须是**纯源锚**。
实际 `source_bridge_node.cpp` 把**同一个 pose** 同时发给 `/localization/odom_source` 与
`/localization/source_anchor`。gap 期该 pose 含 DR 自身增量 → DR 观测到自己的输出 → 正反馈。

**C. 配对时间基错位**

`last_slam_` 的时间戳是扫描时刻（`laser_odometry.cc` 用 `measures_.lidar_bag_time_`），
而 DR 配对若取 `onSlam` **到达时刻**的 `last_dr_`，两者差一个 LIO 处理延迟内的运动量。
gap 填充每次启停都按该错位量跳一次。

### 1.2 切 DR 瞬间跳动 + DR 段变差（问题 2）

**D. 接管锚定基准错**

```cpp
glue_dr_ = compose(compose(glue_slam_, last_slam_), inverse(last_dr_));  // 锚在"纯 SLAM 位姿"
```

切换前输出（gap 已推进）= `A ⊕ (D(t_s)⁻¹ ⊕ D(t))`，切换后 = `A`。
**瞬间回跳 = 检测窗内已走的 DR 运动量**（0.25s × 车速，2m/s 时约 0.5m）——"开始切换就产生跳动"。

**E. DR 被观测污染 → 接管后质量下降**

| 项 | 证据 | 影响 |
|---|---|---|
| 零速度观测 | `converters.cpp` `has_velocity = finiteVec(twist)`，而锚/SLAM 里程计**都不填 twist**（全零被当成"有效零速度"） | 以 10Hz 把速度态拽向 0，零偏连带被污染 |
| 姿态增益偏大 | 观测更新 `K_θ≈0.57`（P=1e-4、R=(0.5°)²） | DR 姿态被锚贴着走，观测一停即自由漂 |
| 首帧/确认跳变硬对齐 | `applyPoseStep` 覆盖位姿 + **速度清零** + 协方差重写 | 启动期状态反复重置 |
| 门控恢复慢 | `slam_stable_sec: 1.0` | SLAM 恢复后 1s 内不观测，误差先积累 |

**F.（次要）fusion 对局部里程计跳变零防护**：`fusedPose = map_odom ⊕ odom_base`，`setOdomBase` 无条件覆盖，
jump_guard 只挡 map_odom 修正。**桥输出的任何毛刺/跳变原样进轨迹/TF/loc_point** → 连续性只能靠桥自己保证。

---

## 2. 设计不变式（本次的骨架）

### 2.1 输出双视角

| 视角 | 定义 | 去向 |
|---|---|---|
| `Sample::anchor` | 纯源位姿 `glue ⊕ T_source`，**永不含 DR 增量** | 只喂 DR 标定观测 |
| `Sample::motion` | 运动输出，见下 | fusion `odom_base` / GM `source_odom` / 云注册 |

### 2.2 三阶段语义（DR 是否进输出链的唯一分界）

```
① 纯 SLAM   age ≤ gap_fill_delay_sec      motion = anchor = glue_slam ⊕ T_slam      ← DR 不进输出链
② 检测窗     gap_fill_delay < age ≤ slam_timeout
                                       motion = anchor ⊕ (D(t_s)⁻¹ ⊕ D(t))         ← 只用 DR 增量补 gap
③ DR 接管   age > slam_timeout_sec        ② 的公式继续，锚冻结在最后 SLAM 样本
```

②→③ **无换算、无台阶**：所谓"切换"只发生在簿记层面（源点云路径 / 状态上报）。

### 2.3 切换连续性不变式（接管/回切通用）

```
glue_new = unified_motion(t_sw⁻) ⊕ T_new(t_sw)⁻¹
⇒ 切换瞬间 unified_motion 严格相等，检测窗内已走的运动量不丢
```

接管特例（gap 填充开着时）化简为 `glue_dr = T_slam_last ⊕ D(t_s)⁻¹`，即
**"锚定位姿 + 当前 DR 增量"**——与用户描述的切换语义一致。

### 2.4 DR 配对

每个 SLAM 样本到达时按【样本时间戳】在 DR 历史**插值**取 `D(t_s)`（不是到达时刻的 DR 状态）。
另设 `gap_pairing_valid_`：配对必须挂在当前 SLAM 锚链上，DR 期间到达的样本不作数
（否则回切瞬间 gap 公式与接管公式基准不一致，仍会出台阶）。

---

### 2.5 虚拟系确立与 glue 赋值（2026-10-12 补充）

glue = 「源自己的里程计系 → 虚拟统一系」的**固定刚体变换**（`unified = glue ⊕ T_source`）。
它不是标定、不是 map 修正，只在换源瞬间算一次后保持常数。用途三处：
输出位姿、SLAM 注册云变换（桥契约：云与里程计同系）、纯源锚。

**赋值规则：**

| 场景 | 赋值 | 理由 |
|---|---|---|
| 首源 = SLAM | `glue_slam = I` | 虚拟系定为 LIO 系 |
| **冷启动 DR 先起 → LIO 就绪** | **`glue_slam = I`（直接赋值，不换算、不等滞回）** | 虚拟系以 LIO 系为准；此时 map_odom 未建立、无连续性可保，连续性换算反而会把启动期 DR 的误差（roll/pitch 未重力对齐）永久烘进虚拟系 |
| 运行期接管 SLAM→DR | `glue_dr = unified(t_sw⁻) ⊕ T_dr(t_sw)⁻¹` | 连续性不变式（§2.3） |
| 运行期回切 DR→SLAM | `glue_slam = unified(t_sw⁻) ⊕ T_slam(t_sw)⁻¹` | 同上 |

判据是 `slam_frame_established_`（虚拟系是否已由 LIO 帧确立 = 是否出现过 SLAM 驱动输出）：
未确立 → 冷启动路径；已确立 → 连续性不变式。

> 收益：正常时序下 `glue_slam` 恒为 I ⇒ SLAM 注册云**零成本原样透传**（不经 PCL 往返），
> "经桥"与"纯 SLAM"喂给 GM 的点云逐字节一致。
> 代价：冷启动那一次换源输出会跳（DR 系 → LIO 系）。此时 fusion 的 map_odom 尚未建立，
> 跳的是一个未初始化的量；**若 GM 已在 DR 临时系里匹配成功并建立了 map_odom，该跳变会带到融合输出**
> ——启动窗口通常 1~2s 内 LIO 就绪，GM 一般来不及匹配，暂按可接受处理。



## 3. 修改清单

### 3.1 `localization_v3/core/local_odom_source`（核心，纯 C++ 零 ROS）

| 改动 | 说明 |
|---|---|
| API 统一为 `Sample{motion, anchor}` + `sample()` | 与测试契约一致；`purePose()` 为无仲裁副作用取锚接口 |
| 接管/回切改用 §2.3 通用不变式 | 消灭"切换瞬间回跳"（问题 2-D） |
| 新增 `Config::gap_fill_delay_sec`（默认 0.15） | 消灭幽灵参数；**红线注释**：必须 > SLAM 里程计周期并留余量 |
| gap 填充条件收紧 | `kSlam && 配对有效 && 有 DR 配对 && DR 存活 && age > gap_fill_delay` |
| 新增 DR 历史 + 按样本戳插值配对 | 消灭时间基错位（问题 1-C） |
| `gap_pairing_valid_` 锚链校验 | 见 §2.4 |
| `slam_frame_established_` 冷启动直接定虚拟系为 LIO 系 | 见 §2.5：DR 先起时 LIO 到达即 `glue_slam = I`，不换算、不等滞回 |
| 删除 `flap_warned_`（core 无日志依赖） | 改由 `Snapshot::flap_guard` 交桥层告警 |

### 3.2 `localization_v3/ros/source_bridge_node`（薄适配层）

| 改动 | 说明 |
|---|---|
| **锚 / 运动分开发** | `source_anchor`←`anchor`（事件式，每个 SLAM 样本一次）、`odom_source`←`motion`（100Hz）→ 断自回灌（问题 1-B） |
| 锚只在 SLAM 驱动输出时发 | DR 驱动期间不发锚（第二重防回灌） |
| 锚在 `onSlam` 回调内立即发布 | 去掉"等下一个 100Hz tick"的额外延迟 |
| twist 透传 | 沿用源消息的 body 系速度（SLAM 里程计本身不填 twist，行为与接入前一致） |
| 云注册历史容差 0.5s → 0.2s，历史 2.5s | DR 模式注册云对错帧的风险收窄 |
| gap 填充状态进 5s 心跳日志、翻转保护一次性告警 | 可观测性 |

### 3.3 `config/source_bridge.yaml`

新增 `odometry/gap_fill_delay_sec: 0.15`，注释写明取值红线与 `dr_odometry_v2` 门控的对齐关系。

### 3.4 `dr_odometry_v2`（观测质量，3 处小改）

| 改动 | 说明 |
|---|---|
| 新增 `localization/use_velocity`（默认 **false**） | 定位锚是位姿锚、不带 twist；全零 twist 不得当"停车"观测（问题 2-E 首条） |
| `pose_noise_yaw_deg` 接线到 `R(5,5)` | 原为死参数（加载后从未使用） |
| `slam_age_limit` 0.12 → **0.15** | 与 `gap_fill_delay_sec` 对齐：观测期 = 纯 SLAM 期 |

> 未改：`applyPoseStep` 硬对齐、`slam_stable_sec=1.0` 恢复窗、事件队列迟到丢弃策略。
> 这些是 DR 侧既有设计，本次不动（见 §5 遗留）。

---

## 4. 验收

### 4.1 单元测试（已跑，全绿）

`test_v3_local_odom_source` **16 项**（原 12 项契约用例原样保留 + 新增 4 项）：

| 用例 | 锁定的行为 |
|---|---|
| `SlamFirstAdoptsSlamFrame` / `SlamGlueIdentityWhenSlamOnly` | 首源虚拟系 = SLAM 系 |
| `TakeoverIsContinuous` / `DrMotionPropagatesAfterTakeover` | 接管恒等 + DR 运动续推 |
| `HandbackIsContinuousWithHysteresis` | 回切滞回 + 恒等 |
| `SlamGlueNonIdentityAfterDrExcursion` | 回切后云必须 ⊕ glue_slam（桥契约） |
| `SlamCloudFreshnessRequired` / `CloudNotRequiredWhenDisabled` | 部分故障防护 |
| `FreshSlamOutputExcludesDrIncrement` | 新鲜期 DR 不进输出链 |
| `GapFillAndSeamlessTakeover` | 检测窗补 gap + 接管不丢运动量 |
| `BothLostGivesNoOutput` / `FlapGuardDoublesConfirmFrames` | 双失无输出 / 翻转保护 |
| **`NoGapFillWithinNormalSlamCadence`**（新） | **问题 1 回归**：0.10~0.14s 抖动节拍下 gap 填充零触发、motion≡anchor |
| **`GapFillEngagesOnlyWhenOverdue`**（新） | 超龄才启用；启用时 anchor 仍纯 |
| **`GapFillPairedAtSampleStampNotArrival`**（新） | **问题 1-C 回归**：按样本戳插值配对 |
| **`HandbackIsContinuousWhileDrHasMoved``**（新） | **问题 2 回归**：DR 已前进 10m 再回切仍恒等 |
| **`ColdStartDrThenSlamAnchorsLioFrame`**（新） | 冷启动 DR 先起 → LIO 到达即 `glue_slam = I`、无滞回（§2.5） |

其余套件无回归：

```
test_v3_fusion_core        14 PASSED     test_v3_jump_guard       9 PASSED
test_v3_state_machine      12 PASSED     test_v3_failure_detector 9 PASSED
dr_odometry_v2: test_v2_eskf 11 PASSED   test_v2_event_queue 4    test_v2_pose_gate 4
```

> 构建提示：gtest 目标不在 `catkin_make` 默认目标内，需
> `cd build/localization/<pkg> && make test_v3_*`（或 `catkin_make tests`）才会重编；
> 否则跑的是旧二进制，且与新 core 共享库 ABI 不匹配会误报。

### 4.2 三条目标的验收判据

| 目标 | 判据 | 锁定方式 |
|---|---|---|
| 纯 SLAM 期 DR 不参与 | 正常节拍下 `gap_fill` 零触发，`motion ≡ anchor ≡ 纯 SLAM` | `NoGapFillWithinNormalSlamCadence` |
| 切 DR 无停顿/滞后 | `‖motion(t_sw⁺) − motion(t_sw⁻)‖ < 1e-6` 且检测窗运动量保留在轨迹里 | `GapFillAndSeamlessTakeover` |
| 切回 SLAM 无缝 | 同上恒等 | `HandbackIsContinuousWithHysteresis` / `HandbackIsContinuousWhileDrHasMoved` |

### 4.3 尚未做（需要 bag）

- [ ] 纯 SLAM 一段：`traj.txt` / RViz path 无间隔性毛刺（对比 `use_dr_source:=false`）；
- [ ] kill slam 节点 / 掐断 odom：检测窗内输出不冻结，切 DR 瞬间无台阶；
- [ ] SLAM 恢复：回切无台阶，`curr_scan_in_map` 与 path/TF 重合（桥契约）；
- [ ] DR 段漂移率对比（本次未动 `applyPoseStep`，预期改善来自"去掉零速度观测"）。

---

## 5. 遗留与后续

| # | 项 | 说明 |
|---|---|---|
| 1 | 锚的时间戳 | 现用发布时刻（DR 事件队列按 stamp 排序、丢迟到事件，扫描戳会被丢）。代价：观测带一个 LIO 处理延迟错位（≈ v·latency），落在跳变门限小跳档内由 ESKF 吃掉。若 bag 实测可见航向锯齿，再做一阶外推补偿 |
| 2 | `applyPoseStep` 硬对齐 | 首帧/确认跳变直接覆盖状态并清零速度，是 DR 段质量的剩余风险点 |
| 3 | `slam_stable_sec=1.0` | SLAM 恢复后 1s 内不观测；回切后 DR 会自由漂 1s，若需要可缩短 |
| 4 | fusion 侧零防护 | 局部里程计跳变原样进输出（设计使然）；连续性目前完全由桥保证 |
| 5 | `base_yaw_offset=-π/2` | 只作用于 TF `map->base`，与 topic 位姿差 90°，排查"跳变"时别误判 |
| 6 | 源注册表改造 | 见 `多源里程计接入改造方案.md`（二元硬编码 → 表驱动），本次未做 |

---

## 附：时间基与实测证据

`loclog/source_bridge.log`（2026-10-12 10:29~10:32 实跑，旧二进制）：

```
source=SLAM slam_age=0.092  ...     ← 5s 心跳随机采样
source=SLAM slam_age=0.1239 ...     ← 已越过 0.12 阈值 → gap 填充误触发
source=SLAM slam_age=0.1077 ...
odometry source switch -> DR (switch#2)   slam_age=0.446   ← 断流接管
```

`slam_age` 分布 0.002~0.124s 与"阈值 0.12s 卡在 0.1s 周期 + 抖动里"一致，
是问题 1 的直接证据。
