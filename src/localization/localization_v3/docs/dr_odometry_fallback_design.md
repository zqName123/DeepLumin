# DR 航位推算接管设计文档（SLAM 断流容错）

> 版本：v1.0（设计评审稿，不含代码改动）
> 日期：2026-10-10
> 关联：`docs/localization_v3_optimization_plan.md`（v3 主优化方案）
> 范围：`localization_v3` 增加局部里程计源管理（SLAM 主 / DR 备）；`dr_odometry` 仅做只读审计 + 可选改进建议，主方案不改它。

---

## 0. 背景与目标

当前 `localization_v3` 的局部里程计**只有一个来源**：`slam_odometry_v2`（`/localization/slam_odom`）。该链路断流后：

- `fusion_core` 的 `odom_base_` 冻结 → 融合位姿不再推进；
- 失效判定 `slam_timeout_sec=0.5s` 触发 → **直接 LOCALIZATION_FAILURE**，系统完全失效。

目标：引入 `dr_odometry`（IMU+轮速 ESKF）作为**短时备用里程计**——SLAM 断流时从当前位置继续提供 odom 位姿，**匹配判断机制与状态机转换保持完全一致**；SLAM 恢复后无痕接管回来。

设计诉求（对应提问 1~4）：

| # | 问题 | 本文回答 |
|---|---|---|
| Q1 | DR 与 SLAM 坐标系不一致，如何从当前位置续推？常驻运行是否冗余？ | §3.2：热备 + 换源时刻一次对齐（数学见 §4.1）；常驻成本论证见 §3.2-C |
| Q2 | DR 接管后失效怎么判？点云怎么办？ | §4.2：原始点云按统一 odom 位姿注册 → GM 匹配/质量分级/失效计数**机制不变** |
| Q3 | SLAM 恢复如何回切？如何丝滑？ | §4.3：回切重锚（保 map_base 连续）+ 滞回防抖，数学见 §4.3 |
| Q4 | DR 无 GPS 时是否只靠姿态+轮速？IMU+轮速融合对不对？开关怎么留？ | §2.2 审计结论 + §4.6 开关矩阵与短时推理约定 |

---

## 1. 现状分析

### 1.1 localization_v3 的单源依赖（断流即失效链路）

```
slam_odometry_v2 ──/localization/slam_odom───────────► fusion_.setOdomBase()   （唯一 odom 输入）
                 ──/localization/cloud_registered_effect─► GM 源点云            （唯一匹配点云来源）
                                                          │
slam 断流 ──► odom_base 冻结 / GM 无源点云 ──► failure_detector.slamTimeout(0.5s)
                                              └──► forceLocalizationFailure（直接失效）
```

受影响的三件事：**位姿推进**（odom_base）、**匹配判据**（GM 源点云）、**失效判定**（slam_timeout 直接判死）。接管方案必须同时兜住这三件事。

### 1.2 dr_odometry 现状（只读梳理）

| 项 | 现状 | 位置 |
|---|---|---|
| 架构 | 15 维误差状态卡尔曼（ESKF）：p/v/θ/bg/ba | `dr_odometry/core/dr_eskf.*` |
| 预测 | IMU 中值积分（陀螺推姿态、加计推速度，`max_dt=0.05` 截断） | `dr_eskf.cpp:82-129,163-200` |
| 观测 | 轮速（世界系纵向速度 + 非完整约束）、GNSS（heading/velocity/position 三个独立开关） | `dr_eskf.cpp:213-299` |
| 输出 | `/localization/dr_odom`（nav_msgs/Odometry，frames `odom->base_link`）、status、path | `dr_odometry_node.cpp:62-64` |
| 坐标系 | 自己的局部系：原点=启动时刻（或首个有效 GNSS 位置） | `dr_odometry.yaml` 头注释 |
| 运行方式 | **常驻**：spin 循环 100Hz 发布，IMU 回调驱动滤波 | `dr_odometry_node.cpp:79-86` |
| 状态注入 | `DrEskf::reset(DrState)` 核心已有（"外部强制重置"），**节点未暴露接口** | `dr_eskf.hpp:43,69-73` |
| 资源 | 纯滤波（无点云），每 IMU 帧一次 15×15 传播 + 1~4 维更新 | — |

### 1.3 DR ESKF 融合正确性审计（Q4）

| # | 审查项 | 结论 | 依据 |
|---|---|---|---|
| 1 | 预测模型：`p+=v·dt+½a·dt², v+=a·dt, q⊗=δq(ω,dt)` | ✅ 正确 | `dr_eskf.cpp:175-179` |
| 2 | 重力符号（z-up，静止时 R·accel+g=0） | ✅ 正确 | `dr_eskf.cpp:165-173`，`g=(0,0,-9.80665)` |
| 3 | 误差状态 F：`δp'=δv；δv'=-R[a]×δθ-R·δba；δθ'=-δbg` | ✅ 正确 | `dr_eskf.cpp:185-191` |
| 4 | 轮速更新：世界系速度观测 + 非完整约束（侧向/垂向≈0），噪声随 yaw 旋转对齐 | ✅ 正确 | `dr_eskf.cpp:213-258` |
| 5 | 轮速**只修速度、不污染姿态**（H 无姿态项，避免轮速残差反向注入 yaw） | ✅ 设计正确 | `dr_eskf.cpp:240-242` 注释 |
| 6 | GNSS 三开关独立（heading/velocity/position），heading 有车速门限 | ✅ 正确 | `dr_eskf.cpp:260-299`，`min_wheel_speed_for_heading=0.2` |
| 7 | `use_gnss_position` **默认 false**（不绑绝对位置/原点漂移） | ✅ 已符合"默认无 GPS"诉求 | `dr_odometry.yaml:85` |
| 8 | 轮速可用性开关：`can_use_valid_flag=false` 时速度有限即参与 | ✅ 开关保留 | `ros_adapter.cpp:139-149` |
| 9 | 卡尔曼更新：Joseph 形式 + 对称化 + 有限性检查 + LDLT | ✅ 健壮 | `dr_eskf.cpp:301-340` |
| 10 | **初始化未做重力对齐**：roll/pitch 从 0 起步，靠滤波相关性慢慢修正 | ⚠️ 短板 | `initializeFromImu` 只记时间戳（`dr_eskf.cpp:75-80`） |
| 11 | **无 GNSS 航向时 yaw 纯陀螺漂移**，轮速无法修 yaw（H 无姿态项） | ⚠️ 固有 | 由 #5 的正确设计决定 |
| 12 | `max_dt=0.05` 截断：IMU 断流恢复会丢积分时间 | ⚠️ 边界 | `dr_eskf.cpp:107` |
| 13 | frames 配置 `odom/base_link` 与 SLAM 同名（TF 混淆隐患） | ⚠️ 命名 | `dr_odometry.yaml` frames 段；`publish_tf=false` 暂无冲突 |

**审计总结**：IMU+轮速融合推理**正确可用**，符合短时航位推算定位；两处固有限制（#10 姿态初始化、#11 yaw 漂移）分别用"接管时姿态对齐"（§4.1，主方案解决）与"限制 DR 模式时长 + 在线漂移监视"（§4.6）覆盖。开关机制（use_wheel / use_gnss* / can_use_valid_flag）**保持不动**。

---

## 2. 设计原则

1. **一个主语义**：统一"局部里程计"（unified local odom）。SLAM/DR 只是它的两个实现，切换发生在源管理层，**下游（GM/失效判定/状态机）无感知**——这是"判断和状态转换保持一致"的根本保证。
2. **切换保连续**：任何换源时刻 map_base 连续（§4.1/§4.3 的锚定数学），不引入输出跳变。
3. **判断机制不变**：DR 模式下 GM 继续匹配、质量三级、jump guard、失效计数全部照旧；只换源数据。
4. **失效语义不变**：五态状态机不动；仅把"slam_timeout=直接失效"改为"源切换事件"，全源丢失才失效。
5. **DR 只做短时**：DR 是桥接不是替代；带最长时长与漂移监视（§4.6）。
6. **不破坏现有定位效果**：SLAM 正常时所有路径与现状逐字节一致（透传）。

---

## 3. 总体方案

### 3.1 核心结构：LocalOdomSource（局部里程计源管理）

```
                ┌────────────────────────────────────────────┐
slam_odom ─────►│                                            │
                │   LocalOdomSource（新模块，纯逻辑可测）      │──► 统一 odom->base
dr_odom ───────►│   · 源状态机：kSlam ⇄ kDr ⇄ kNone          │    （锚定换源，连续）
                │   · 换源锚定 / 回切重锚 / 滞回防抖           │──► 统一源点云（fallback 注册）
                └────────────────────────────────────────────┘
                                    │
                     fusion_core（不变）· GM（配置不变）· 状态机（不变）
```

### 3.2 坐标系与续推策略（Q1）——三方案对比

**方案 A（推荐）：DR 热备 + 换源时刻一次对齐**

DR 常驻运行在自己的 `dr_odom` 系下推算；SLAM 断流时刻 t0 由 `LocalOdomSource` 计算一次对齐：

```
A = T_odom_base(t0) ⊕ (T_dr_base(t0))⁻¹        （dr 系 → SLAM odom 系，接管期间常量）
T_odom_base(t) = A ⊕ T_dr_base(t)               （统一里程计继续在 SLAM odom 系里推进）
```

- **从当前位置续推**：t0 时刻 `A ⊕ T_dr_base(t0) = T_odom_base(t0)`，严格无缝；
- **下游零感知**：GM 的 `source_odom`、云注册、TF 全部继续用 SLAM 的 odom 语义；
- **map->odom 不动** → map_base 连续；
- 无需修改 `dr_odometry`。

**方案 B（备选）：按需启动 + 位姿注入**

SLAM 断流才拉起 dr_odometry，并把 SLAM 最后位姿注入（`DrEskf::reset` 已有核心能力，节点需补一个注入接口）。优缺点：

- ✅ 不跑 DR 不占资源；注入后输出天然在 SLAM odom 系（无需 A）；
- ❌ 冷启动零偏未收敛（gyro bias 靠相关性慢慢修）、首帧姿态无重力对齐（审计 #10）、拉起延迟（ROS 节点启动 + 首帧 IMU 对齐）——**接管瞬间到前几秒精度最差，恰是最需要它的时刻**；
- ❌ 需要改 dr_odometry（暴露注入 service/topic）。

**方案 C（否决）：双里程计紧耦合/实时对齐**——复杂度高、收益低。

**热备成本论证（回应"占资源/冗余"顾虑）**：

| 项 | dr_odometry（热备） | slam_odometry_v2（主） |
|---|---|---|
| 每周期计算 | 15×15 矩阵传播 + 1~4 维更新（**微秒级**） | 每帧点云 KD-tree/体素配准（**几十 ms**） |
| 数据量 | IMU 200Hz + CAN 50Hz（纯标量/向量） | 每帧数万点 |
| 结论 | 相对 LIO **可忽略**（<1% 量级） | — |

"冗余"是链路冗余而非逻辑冗余：两条链路被 `LocalOdomSource` 收编为一个抽象下的两个源，状态机/判断层仍然单线。**推荐方案 A**；方案 B 作为配置项 `dr_mode: hot | on_demand` 保留演进空间（本期不做）。

### 3.3 数据流（两种模式）

```
【SLAM 模式（常态）】
slam_odom ──────────────透传──► /localization/odom_source ──► GM source_odom
cloud_registered_effect ─透传──► /localization/source_cloud ─► GM source_cloud
fusion_.setOdomBase( SLAM pose )      （与现状逐字节一致）

【DR 模式（SLAM 断流）】
dr_odom ──A⊕T_dr_base──► /localization/odom_source ──► GM source_odom（配置不变）
/ouster/points ─⊕T_odom_base⊕T_base_lidar──► /localization/source_cloud ─► GM source_cloud
fusion_.setOdomBase( A⊕T_dr_base )    （GM 匹配/质量/失效判断机制全部不变）
```

---

## 4. 关键机制设计

### 4.1 SLAM→DR 接管（含数学）

**触发**：`slam_odom` 新鲜度超 `slam_timeout_sec(0.5s)` 且 DR 源健康（最近 `dr_timeout_sec` 内有数据）。

**锚定**（t0 为接管时刻，取最后一帧 SLAM 位姿与最近一帧 DR 位姿）：

```
A            = T_odom_base_slam(t0) ⊕ (T_dr_base(t0))⁻¹
T_odom_base  = A ⊕ T_dr_base(t)          （接管期间 A 常量）
map_odom 不变 ⇒ map_base(t0⁺) = map_base(t0⁻)   （输出连续）
```

**姿态一致性**：SLAM odom 与 DR 世界系均为 z-up（LIO 重力对齐 / DR 重力项 `(0,0,-g)`），A 的旋转差主要是 yaw + 小 roll/pitch，数学上无系统性错切。

**接管后立即做两件事**：

1. `fusion_core` 的 `setOdomBase` 输入切到统一源（同一 API，不改 fusion_core）；
2. 校验 `A ⊕ T_dr_base(t0)` 与 `T_odom_base_slam(t0)` 的差为 0（构造保证），并记录接管日志（接管时 DR 速度/零偏健康摘要）。

**若 DR 不可用**（无 IMU/无数据）：维持现状——`slam_timeout` 直接 LOCALIZATION_FAILURE（reason="all odometry sources lost"）。

### 4.2 DR 模式下的匹配与失效判定（Q2）

**结论：把原始点云用统一 odom 位姿注册到 odom 系，GM 机制原样继续。** 不是"转到 map 系再判"——GM 的接口就是"源点云（源系）+ 源里程计 + map 初值"，map 变换在 GM 内部由 `map_source` 猜测完成，**判据（inlier/rmse 三级 + jump guard + 失效计数）一行不改**。

```
fallback 云注册（SLAM 云断流时启用）：
  T_odom_lidar = T_odom_base(t_cloud) ⊕ T_base_lidar      （T_base_lidar 外参，复用 GM 的 base_to_lidar）
  cloud_in_odom = raw_cloud(/ouster/points) ⊕ T_odom_lidar
  → 发布 /localization/source_cloud（header.frame_id = odom）
```

| 判定机制 | DR 模式下 | 说明 |
|---|---|---|
| GM 质量三级（good/degraded/bad） | **不变** | 同一套阈值 |
| jump guard（防几何相似跳变） | **不变** | DR 漂移产生的中等修正走 L2 三帧确认，仍生效 |
| GM 失败/坏质快计数 → TRACKING_LOST | **不变** | 判据一致 |
| 出图快路径 / 无修正看门狗 | **不变** | — |
| 恢复（reloc，用原始点云） | **不变** | reloc 本来就用 `/ouster/points`，SLAM 断流不影响 |
| 人工位姿 + 验证 | **不变** | 验证匹配用 fallback 云，流程不变 |
| 源超时 | **扩展** | `slam_timeout` 不再直接失效→切源；新增 `dr_timeout`（DR 模式下 DR 断流）→ 失效 |

**fallback 云的实现注意**：用扫描时刻（cloud stamp）的统一 odom 位姿注册（两帧间线性插值位置/球面插值姿态），保持与 SLAM 注册云同语义（GM 不感知差异）；原始点云也断流时 GM 侧转 TRANSIENT（轻计数）+ 看门狗兜底。

### 4.3 DR→SLAM 回切（Q3）：丝滑的数学与机制

**触发（滞回防抖）**：SLAM 连续 `switch_back_confirm_frames(5)` 帧新鲜（≈0.5s）才回切；避免断续流导致源来回翻转。短时间多次翻转（如 60s 内 >3 次）→ 保持 DR 并告警。

**回切重锚（保证 map_base 连续）**：SLAM 断流期间车辆在动，SLAM 恢复后其 odom->base 必然与 DR 续推有缺口（数据缺口或节点重启归零）。切换瞬间把缺口**全部吸收进 map->odom**（这正是修正帧的职责）：

```
T_map_base(t1⁻) = map_odom_old ⊕ T_odom_base_dr(t1)      （DR 当前位姿，切换前一瞬间）
map_odom_new    = T_map_base(t1⁻) ⊕ (T_odom_slam_base(t1))⁻¹
⇒ T_map_base(t1⁺) = map_odom_new ⊕ T_odom_slam_base(t1) = T_map_base(t1⁻)   （严格连续）
```

- SLAM **数据缺口型恢复**（节点存活、数据晚到）：缺口位移被 map->odom 吸收，随后 GM 逐帧修回；
- SLAM **节点重启型恢复**（odom 归零）：公式同样成立（`T_odom_slam_base` 小、map_odom 补偿）；
- 回切后若 `|T_odom_base_dr ⊖ T_odom_slam_base|` 过大（>3m/15°）记 WARNING（"slam gap absorbed"），供事后分析；
- 速度/twist 源同步切回 SLAM；TF、话题均无跳变。

**丝滑性验收指标**：map_base 位置/航向时序无阶跃（<1cm/0.1°，构造保证）；速度输出无尖峰；下游停车/规划无抖动。

### 4.4 状态机与状态语义（保持一致）

五态状态机**完全不动**。源切换只存在于 `LocalOdomSource` 层：

| 状态 | DR 模式下的表现 | 与现状差异 |
|---|---|---|
| TRACKING | 正常跟踪（GM 继续修正）；level=WARNING，reason="slam odometry lost, running on DR" | 仅状态文案/告警 |
| TRACKING_LOST | 恢复流程（reloc）正常 | 无 |
| LOCALIZATION_FAILURE | 全源丢失 / 恢复预算耗尽 | failure_reason 增加源信息 |

FusionStatus 扩展（消息已有字段，无需改 msg）：`used_sources` 追加 `"dr_odometry"`；`last_dr_age` 真实化；`is_dr_available` 由 DR 数据年龄导出。

### 4.5 失效判定扩展（failure_detector）

| 信号 | 现状 | 新行为 |
|---|---|---|
| `slam_timeout_sec(0.5s)` | → 立即 LOCALIZATION_FAILURE | → **切源事件**（DR 可用则切 DR）；DR 也不可用 → LOCALIZATION_FAILURE（"all odometry sources lost"） |
| `dr_timeout_sec(0.5s)`（新） | — | DR 模式下 DR 断流 → LOCALIZATION_FAILURE |
| `max_dr_duration_sec(300s)`（新） | — | DR 模式超时仍无 SLAM → 升级告警 / FAILURE（可配，§4.6） |
| 无修正看门狗 / GM 计数 / 出图 / lost 预算 | 不变 | 不变 |

### 4.6 DR 使用约定（Q4）

**开关矩阵（机制保留，只定推荐默认）**：

| 场景 | use_imu_accel | use_wheel | use_gnss | use_gnss_heading | use_gnss_velocity | use_gnss_position |
|---|---|---|---|---|---|---|
| **无 GPS 矿井（默认）** | true | true | **false** | —（订阅都不订） | — | — |
| 有 GPS（可选） | true | true | true | true（车速≥0.2m/s） | false | **false**（保持） |

- 无 GPS 时 DR = 陀螺姿态积分 + 轮速纵向速度 + 非完整约束；**位置是速度积分的结果，姿态（yaw/pitch/roll）来自陀螺+滤波**——即"只信姿态与短时推进、不信绝对位置"，与 `use_gnss_position=false` 的现状一致；
- 轮速/GNSS 开关、`can_use_valid_flag` 语义**原样保留**，不改 dr_odometry。

**短时推理正确性保障（在主方案内闭环）**：

1. **姿态起点正确**：接管对齐（§4.1）直接继承 SLAM 姿态，绕开 DR 冷启动 roll/pitch 未对齐问题（审计 #10）；
2. **在线漂移监视**：GM 每次 good 修正给出 map_base_GM，与 DR 续推 map_base 比较得漂移量 `drift = |map_base_GM ⊖ map_base_DR|`，滑窗统计——超 `dr_drift_warn(2m/5°)` 告警、超 `dr_drift_fail(5m/15°)` 判降级/失效（可配）；
3. **最长时长**：`max_dr_duration_sec`（默认 300s）——超过仍未回 SLAM，升级 LOCALIZATION_FAILURE（reason="DR fallback exceeded max duration"）；
4. 适用窗口结论：**分钟级短时可用**（轮速标度/打滑 + yaw 陀螺漂移决定），超过窗口必须靠 GM 修正或回 SLAM。

**dr_odometry 侧改进建议（可选，不阻塞主方案）**：

| # | 建议 | 收益 |
|---|---|---|
| D-a | `initializeFromImu` 做重力对齐（用首帧/前 N 帧加计定 roll/pitch） | 冷启动/无外部对齐场景的姿态精度 |
| D-b | 暴露位姿重锚接口（只改 p/v/θ 保零偏与协方差的 `reanchor(pose,vel)`，区别于全量 `reset`） | 支持方案 B / 运行中重锚 |
| D-c | frames 改名 `dr_odom/base_link`（或保持 `publish_tf=false`） | 避免与 SLAM 的 odom 同名混淆 |
| D-c | 输出 DR 协方差/漂移率到 status | 供上层融合加权 |

---

## 5. 代码优化方案（结构）

### 5.1 模块划分（localization_v3 内，风格与 v3 一致：core 纯逻辑可测 / ros 薄适配）

```
localization_v3/
├── core/
│   ├── local_odom_source.hpp/.cpp        [新] 局部里程计源管理（核心）
│   │     struct OdomSample { Pose3d pose; Vec3d velocity; double stamp; }
│   │     enum class OdomSourceState { kSlam, kDr, kNone };
│   │     onSlam(sample) / onDr(sample) / tick(now)
│   │     unifiedPose(now) → OdomSample    （含 A 对齐、回切重锚、滞回）
│   │     源切换事件回调（供 failure_detector / 日志）
│   ├── failure_detector.hpp/.cpp         [扩展] 源超时语义（slam→切源，dr→失效，全丢→失效）
│   ├── fusion_core / jump_guard / quality_gate / state_machine / init_arbiter   [不变]
├── ros/
│   ├── fallback_cloud_registrar.hpp/.cpp [新] 原始点云 → odom 系注册（SLAM 云断流时）
│   │     输入 raw_cloud + 统一 odom 位姿 + T_base_lidar；输出 /localization/source_cloud
│   ├── source_bridge.hpp/.cpp            [新] 统一源话题发布（odom_source / source_cloud 透传或注册）
│   ├── status_publisher.hpp/.cpp         [扩展] used_sources += dr_odometry；last_dr_age 真实化
│   └── localization_v3_node.cpp          [编排扩展] 订阅 DR/原始点云；驱动 LocalOdomSource
└── config/localization_v3.yaml           [扩展] odometry_source / fallback_cloud 两段
```

**职责边界**：

| 模块 | 职责 | 不做 |
|---|---|---|
| `LocalOdomSource` | 源选择、换源锚定（A）、回切重锚、滞回、源健康 | 不碰 map_odom、不做质量判定 |
| `fallback_cloud_registrar` | 点云坐标变换注册 | 不判质量 |
| `failure_detector` | 源超时/漂移监视信号 | 不直接切源（切源是 LocalOdomSource 决策） |
| `fusion_core` | **不变**：只消费统一 `odom_base` | — |
| `state_machine` | **不变** | — |

### 5.2 接口变化汇总

| 位置 | 变化 |
|---|---|
| `fusion_core` | **无**（`setOdomBase` 语义不变，输入改喂统一源） |
| `state_machine` | **无** |
| `jump_guard`/`quality_gate`/`init_arbiter` | **无** |
| `failure_detector` | `slamTimeout` 语义拆分；新增 `drTimeout`、`drDrift` 信号、`max_dr_duration` |
| `status_publisher` | DR 源信息（消息字段已有） |
| `node` | 新增 DR 订阅、raw cloud 订阅、统一源话题发布 |
| launch | GM 的 `topics/source_odom` → `/localization/odom_source`；`topics/source_cloud` → `/localization/source_cloud`；新增 `dr_odometry` 节点启动 |
| `dr_odometry` | **主方案零改动**（改进建议见 §4.6-D） |

### 5.3 配置（新增两段）

```yaml
# -------------------- 局部里程计源（SLAM 主 / DR 备）--------------------
odometry_source:
  slam_topic: /localization/slam_odom        # 主源
  dr_topic: /localization/dr_odom            # 备源（dr_odometry 输出）
  dr_mode: hot                               # hot(热备,推荐) | off | on_demand(预留)
  slam_timeout_sec: 0.5                      # SLAM 断流 → 切 DR
  dr_timeout_sec: 0.5                        # DR 模式下 DR 断流 → 失效
  switch_back_confirm_frames: 5              # 回切滞回：连续 N 帧 SLAM 新鲜才回切
  flap_guard_count: 3                        # 60s 内切换超 N 次 → 保持 DR 并告警
  max_dr_duration_sec: 300                   # DR 模式最长时长，超时升级失效
  dr_drift_warn_trans: 2.0                   # 在线漂移监视告警门限（m / deg）
  dr_drift_warn_yaw_deg: 5.0
  dr_drift_fail_trans: 5.0
  dr_drift_fail_yaw_deg: 15.0

# -------------------- 源点云（SLAM 云断流时的 fallback 注册）--------------------
fallback_cloud:
  enabled: true
  raw_cloud_topic: /ouster/points            # 原始点云
  source_cloud_topic: /localization/source_cloud   # 统一源点云输出（GM source_cloud 指向这里）
  odom_source_topic: /localization/odom_source     # 统一源里程计输出（GM source_odom 指向这里）
  base_to_lidar:                             # T_base_lidar（与 GM 外参一致）
    translation: [0.0, 0.0, 0.0]
    rotation: [1,0,0, 0,1,0, 0,0,1]
```

### 5.4 时序（正常→断流→恢复）

```
t        SLAM 正常（透传，一切同现状）
t0       SLAM 断流（0.5s 无数据）
t0+0.5s  LocalOdomSource：A = T_odom_base ⊕ T_dr_base⁻¹，切 DR
         · fusion 输入 = A⊕T_dr_base（map_base 连续）
         · 源点云切 fallback 注册（GM 继续匹配，判据不变）
         · FusionStatus: used_sources += dr_odometry，level=WARNING
t0+...   DR 短时续推；GM 正常修正；失效判定同常态
         · 若 GM 漂移监视超门限 / DR 断流 / 超 max_dr_duration → 失效
t1       SLAM 恢复（连续 5 帧新鲜）
t1+ε     回切重锚：map_odom_new = map_base(t1⁻) ⊕ T_odom_slam_base(t1)⁻¹
         · map_base 严格连续；SLAM 缺口吸收进 map->odom，GM 随后修回
         · 源点云切回 SLAM 注册云；used_sources 去掉 dr_odometry
```

---

## 6. 测试与验收

| 用例 | 通过标准 |
|---|---|
| 单测：`LocalOdomSource` 换源锚定 | A⊕T_dr(t0)=T_odom_base(t0)（构造连续）；切源前后 map_base 差 <1e-6 |
| 单测：回切重锚 | map_base(t1⁻)=map_base(t1⁺)；SLAM 缺口/归零两种情形 |
| 单测：滞回/防抖 | 0.5s 内抖动流不来回切换；flap_guard 生效 |
| 单测：failure_detector 源信号 | slam 断→切源不判死；DR 断→失效；双源断→失效（"all odometry sources lost"） |
| bag：kill slam 节点 | 0.5s 内切 DR；map_base 无跳变；GM 匹配持续、状态时间线同常态 |
| bag：SLAM 断 30s 再恢复 | 回切连续；吸收 gap 后 GM 逐步修回；无 OUTPUT 尖峰 |
| bag：无 GPS 全程 | DR 仅姿态+轮速；漂移监视数值合理（与 GM 修正差） |
| bag：DR 模式内 GM 出图 | 出图快路径/失效语义与 SLAM 模式一致 |

---

## 7. 风险与决策点

| # | 决策点 | 推荐 | 备选 |
|---|---|---|---|
| D1 | DR 热备 vs 按需启动 | **热备**（成本可忽略，接管零预热） | 按需（省资源，代价：冷启动零偏/姿态、启动延迟，需改 dr_odometry） |
| D2 | DR 模式下 GM 是否继续匹配 | **继续**（fallback 云注册，判据一致） | 仅看门狗+reloc（更简单但"判断机制"中断） |
| D3 | DR 模式状态语义 | 状态不变（TRACKING）+ level=WARNING + used_sources 标注 | 新增状态（打破"状态转换一致"） |
| D4 | DR 模式最长时长 | 300s（超时升级失效） | 600s / 仅告警不失效 |
| D5 | 漂移监视超限的处置 | 告警(2m/5°) + 降级(5m/15°) | 仅告警 |
| D6 | dr_odometry 改进（重力对齐/重锚 API/改名） | 列入后续，不阻塞 | 本期一并做 |

风险：

- **R1 fallback 注册云质量低于 SLAM 注册云**（无去畸变/运动补偿）→ GM inlier 可能下降；缓解：注册用扫描时刻位姿插值 + 保留 deskew 由 GM 内部处理；验收 bag 对比 inlier 分布。
- **R2 DR 漂移导致 GM 大修正被 jump guard 拦** → L2 三帧确认放行真实漂移修正；漂移监视兜底告警。
- **R3 双源抖动（SLAM 断续流）** → 回切滞回 + flap_guard。
- **R4 A 对齐受外参/时间戳误差影响** → 接管时刻做一致性检查（构造差为 0）；DR/SLAM 时间戳统一用各自 header stamp。

---

## 附录 A：改动清单（供实施拆分）

| 阶段 | 内容 | 涉及 |
|---|---|---|
| P1 | `core/local_odom_source` + 单测（锚定/回切/滞回） | localization_v3 |
| P2 | `failure_detector` 源信号扩展 + 单测 | localization_v3 |
| P3 | `ros/fallback_cloud_registrar` + `source_bridge` + node/launch/config | localization_v3 |
| P4 | status_publisher 扩展 + 文档 | localization_v3 |
| P5 | bag 回归（断流/恢复/无 GPS） | 测试 |
| P6（可选） | dr_odometry 改进项 D-a~D-c | dr_odometry |
