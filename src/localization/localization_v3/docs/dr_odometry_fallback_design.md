# DR 航位推算接管设计文档（SLAM 断流容错）— 定稿

> 版本：v2.0（完整方案，替代 v1.x 迭代稿）
> 日期：2026-10-10
> 关联：`localization_v3_optimization_plan.md`（v3 主优化方案，本文是其扩展）
> 改动范围：`dr_odometry`（观测接口，P0）+ `localization_v3`（源管理/fallback 云/失效扩展）。**本文档为设计方案，未含代码改动。**

---

## 0. 问题、前提与问答速览

**问题**：`localization_v3` 的局部里程计只有 SLAM 一个来源，断流后融合位姿冻结、`slam_timeout` 直接判 LOCALIZATION_FAILURE——完全失效。引入 `dr_odometry` 做短时备用。

**前提（贯穿全文）**：dr_odometry 绝对精度差（无 GNSS 时 yaw 陀螺漂移 + 轮速积分误差），只适合"输出保活的短时桥接"。本方案的治本手段是**用定位结果辅助标定 DR**（§4.1），把它的短时精度提到可用水平。

| # | 问题 | 答案（详见） |
|---|---|---|
| Q1 | DR 订阅定位的 status + odom，status 正常时把 odom 作观测更新 DR 状态，并做跳变判断防伪跳变污染，可行吗？ | **可行，本方案的核心机制 §4.1**。门控= status TRACKING + SLAM 新鲜；跳变判断=创新分级（小跳直收/大跳多帧一致/超限拒） |
| Q2 | 文档说对齐到 SLAM odom 系，但定位发布的是 map 系，该用哪个？ | **DR 世界系 = map 系**（§3.2）。观测=定位 map->base 零转换；接管对齐量退化为现成的 `map_odom(t0)⁻¹`。"对齐 odom 系"版本已废弃（转换是恒等式，见 §3.2 对比表） |
| Q3 | DR 源替换时直接替换即可？判断校验/状态切换要改吗？ | **位姿层不能直接换**（需锚定投影 + 源点云 fallback + 失效源信号扩展）；**判断/校验/状态机完全不改**（§3.1、§5.3） |
| Q4 | 无 GPS 时 DR 靠什么？IMU+轮速融合对不对？开关怎么留？ | 审计 §1.3 全部通过；开关矩阵 §4.6 保持不变 |
| Q5 | DR 效果差、热备是否有问题？ | 裸热备有问题；本方案 = **辅助标定热备 + 接管准入 + 保活桥接定位**（§3.4/§4.2） |
| Q6 | SLAM 正常时用定位结果约束 DR 零偏，断流时 DR 自带好估计？ | **成立，治本**（§4.1）：外部姿态观测使 gyro bias 可观，yaw 漂移率降一个量级 |

---

## 1. 现状与审计

### 1.1 localization_v3 的单源依赖（断流即失效）

```
slam_odometry_v2 ──/localization/slam_odom────────────► fusion_.setOdomBase()   （唯一 odom 输入）
                 ──/localization/cloud_registered_effect─► GM 源点云             （唯一匹配点云来源）
slam 断流 ──► odom_base 冻结 / GM 无源点云 ──► slam_timeout(0.5s) ──► 直接 LOCALIZATION_FAILURE
```

受影响三件事：**位姿推进**、**匹配点云**、**失效判定**。接管方案必须同时兜住。

### 1.2 dr_odometry 现状

| 项 | 现状 | 位置 |
|---|---|---|
| 架构 | 15 维 ESKF（p/v/θ/bg/ba），IMU 预测 + 轮速/GNSS 观测 | `dr_odometry/core/dr_eskf.*` |
| 输出 | `/localization/dr_odom`（Odometry）、status、path | `dr_odometry_node.cpp` |
| 坐标系 | 自有局部系（原点=启动/首帧 GNSS），frames 名与 SLAM 同名 `odom` | `dr_odometry.yaml` |
| 运行 | 常驻 spin（100Hz 发布，IMU 驱动滤波） | `dr_odometry_node.cpp:79-86` |
| 状态接口 | `reset(DrState)` 核心有、节点未暴露；**无观测注入接口（P0 要加）** | `dr_eskf.hpp:43` |
| 资源 | 纯滤波，微秒级/帧（相对 LIO 可忽略） | — |

### 1.3 ESKF 融合正确性审计（Q4）

| # | 审查项 | 结论 |
|---|---|---|
| 1 | 预测：中值 IMU 积分、dt 截断、`δp'=δv; δv'=-R[a]×δθ-R·δba; δθ'=-δbg` | ✅ 正确 |
| 2 | 重力符号（z-up）；Joseph 协方差更新；有限性检查 | ✅ 正确 |
| 3 | 轮速：世界系速度观测 + 非完整约束；**H 无姿态项不污染 yaw** | ✅ 设计正确 |
| 4 | GNSS 三开关独立；`use_gnss_position` 默认 false（不绑 GPS 绝对位置） | ✅ 符合"默认无 GPS" |
| 5 | `use_wheel` / `use_gnss*` / `can_use_valid_flag` 开关完备 | ✅ 保持不动 |
| 6 | 初始化未做重力对齐（roll/pitch 从 0 起步） | ⚠️ 被辅助标定覆盖（观测直接给姿态） |
| 7 | 无 GNSS 航向时 **yaw 零偏基本不可观**（轮速修不了 yaw）→ 漂移 | ⚠️ 被 §4.1 辅助标定治本 |
| 8 | frames 与 SLAM 同名；`publish_tf=false` 暂无冲突 | ⚠️ 建议改名 `dr_odom`（不阻塞） |

**总结**：IMU+轮速融合推理正确；短板 #6/#7 由辅助标定（§4.1）解决，不是靠调参。

---

## 2. 设计原则

1. **一个主语义**：统一"局部里程计"。SLAM/DR 是两个源，切换在源管理层完成，**下游判断层无感知**——这是 Q3"判断和状态切换不改"的根本保证。
2. **切换保连续**：任何换源时刻 map_base 连续，不引入输出跳变。
3. **用定位结果标定 DR**：定位正常时的 map->base 是全局校正后的正确位姿，作 DR 观测（含跳变防污染）。
4. **判断机制不变**：GM 质量三级、jump guard、失效计数、状态机转移、恢复预算全部沿用。
5. **DR 只做短时保活**：带接管准入、漂移监视、时长上限；预期"快速进 TRACKING_LOST→reloc"是正常路径。
6. **不破坏现有定位效果**：SLAM 正常时的输出路径与现状逐字节一致。

---

## 3. 总体方案

### 3.1 核心结构：统一局部里程计源（LocalOdomSource）

```
                    ┌──────────────────────────────────────────────┐
slam_odom ─────────►│ LocalOdomSource                              │──► 统一 odom->base
                    │  源状态：kSlam ⇄ kDr ⇄ kNone                 │    （换源锚定，map_base 连续）
map_base_dr ───────►│  · SLAM 在：透传 SLAM                        │──► 统一源点云（fallback 注册）
                    │  · SLAM 断：投影锚定后用 DR                   │
                    └──────────────────────────────────────────────┘
                    fusion_core / jump_guard / quality_gate / 状态机   【全部不变】
```

统一源输出喂给：`fusion_.setOdomBase()`（融合）、GM `source_odom`、fallback 云注册、TF。

### 3.2 坐标系方案（Q2 核心）：DR 世界系 = map 系

**观测源是定位发布的 map->base（全局正确位姿），DR 世界系直接对齐 map 系**：

| 方案 | 观测 | DR 世界系 | 评价 |
|---|---|---|---|
| a. 观测 SLAM odom | `/localization/slam_odom` | SLAM odom 系 | 丢掉 GM 全局修正，观测混着 LIO 漂移 |
| b. 观测定位结果，转换到 odom 系 | `map_odom⁻¹ ⊕ map_base` | odom 系 | **恒等式**：化简后=SLAM odom，转换白做 |
| **c. 观测定位结果，世界系=map（采用）** | `map->base` **零转换** | **map 系** | 观测质量最高；接管对齐量=现成的 `map_odom(t0)⁻¹` |

**方案 c 的完整数学**：

```
【热备期（SLAM 活、status TRACKING）】
  DR 状态活在 map 系；观测 z = T_map_base（/localization_v2/odom，经 §4.1 门控）
  统一源 = SLAM odom->base（透传，与现状逐字节一致）

【接管 t0（SLAM 断流）】
  T_odom_base(t) := map_odom(t0)⁻¹ ⊕ T_map_base_dr(t)      （map_odom(t0) 冻结为常量 A⁻¹）
  ⇒ map_base(t0⁺) = map_odom(t0) ⊕ T_odom_base(t0) = T_map_base_dr(t0)   严格连续

【DR 模式中 GM 修正 map_odom → map_odom(t1)】
  map_base = map_odom(t1) ⊕ T_odom_base = map_odom(t1) ⊕ map_odom(t0)⁻¹ ⊕ T_map_base_dr
  ——GM 修正正常生效（因定义里用的是冻结的 map_odom(t0)，不自洽抵消）

【回切 t1（SLAM 恢复）】
  map_odom_new = map_base(t1⁻) ⊕ (odom_base_slam(t1))⁻¹    （缺口吸收进 map->odom，见 §4.4）
```

**要点**：

- map 系与 DR 重力模型一致（LIO z-up / DR g=(0,0,-g)），无系统性错切；
- 辅助模式下"接管对齐"退化为一个现成常量 `map_odom(t0)⁻¹`，`LocalOdomSource` 无需额外标定；
- **非辅助模式**（DR 未被观测标定，如冷启动）仍需经典对齐 `A = T_odom_base_slam(t0) ⊕ T_dr_base(t0)⁻¹`——保留该分支以兼容 `dr_mode: hot(辅助) | standalone`。

### 3.3 数据流（三阶段）

```
【热备标定】定位 status+odom ──► DR 观测门控(§4.1) ──► DrEskf.update（零偏/速度/姿态收敛）
            SLAM odom ──透传──► 统一源 ──► fusion/GM/TF            （同现状）
【DR 接管】 dr map 位姿 ──⊕map_odom(t0)⁻¹──► 统一源 ──► fusion/GM/TF
            /ouster/points ──⊕T_odom_base⊕T_base_lidar──► 统一源点云 ──► GM source_cloud
            （GM 匹配/质量三级/jump guard/失效计数/状态机：机制全部不变）
【回切】    SLAM odom 恢复 → map_odom 重锚 → 统一源切回透传（§4.4）
```

### 3.4 DR 角色定位（预期管理，DR 质量差前提）

| 项 | 定位 |
|---|---|
| 核心价值 | reloc 恢复完成前**输出保活**：map_base/TF 连续 → 下游受控停车 |
| 加分项 | 辅助标定后 GM 能在漂移尚小时继续匹配修正（匹配不上不强求） |
| 明确不做 | 不承诺 DR 模式定位精度；不做长时间航位定位 |
| 预期路径 | SLAM 断 → DR 保活 →（GM 修不动）TRACKING_LOST → reloc 全局恢复 → 回切。**快速进 TRACKING_LOST→reloc 是正常路径** |
| 时长预期 | 实际使用十几秒~几十秒；`max_dr_duration_sec=120` 仅上限保护 |

---

## 4. 关键机制

### 4.1 辅助标定：定位结果观测 DR（Q1/Q6 核心）

**目的**：把 gyro bias / accel bias / 速度 / 姿态在热备期标定好，接管时 DR 自带好估计。

**为什么能行**：DR 误差状态已有 `δθ' = -δbg` 耦合（`dr_eskf.cpp:191`），缺外部姿态观测闭环——引入 map->base 观测后 yaw 误差被持续修正，**gyro bias 变得可观**；位置/速度观测辨识 accel bias 与轮速标度。车辆运动提供激励。

**预期改善（量级，需 bag 验证）**：

| 指标 | 无辅助 | 辅助标定后 |
|---|---|---|
| yaw 漂移率 | 0.6~3 °/min | ~0.05~0.2 °/min |
| 60s 后 yaw 误差 | 1-3°（GM 匹配域边缘） | 0.1~0.2° |
| 有效桥接窗 | 几十秒 | 1~3 分钟量级 |

#### 4.1.1 观测门控（订阅 status + odom）

DR 订阅 `/localization_v2/status`（FusionStatus）与 `/localization_v2/odom`（map->base）。观测生效条件（与配置项对应）：

```
state == "TRACKING"  &&  output_valid == true  &&  last_slam_age < obs_slam_age_limit(0.2s)
（可选更严：level == LEVEL_NORMAL，配置 obs_require_level_normal）
```

- `last_slam_age` 用于排除**闭环回喂**：DR 模式下定位输出本身由 DR 驱动，必须关观测（否则 DR 更新 DR 自己）；
- status 非 TRACKING（LOST/FAILURE/初始化）不观测——劣质结果不进滤波；
- 观测内容：`map->base` 位姿（p/θ）+ twist 速度（body 系，免坐标转换）。

#### 4.1.2 跳变判断（防伪跳变污染）

定位端 map_base 的跳变有两种：**真实修正**（GM 校正漂移，后续观测稳定在新水平）与**伪跳变**（几何相似误匹配，一次性/来回跳）。按创新量 `Δ = |观测 ⊖ DR 预测|` 分级：

| 档 | 条件（默认） | 处理 |
|---|---|---|
| 小跳 | Δ ≤ 0.5m / 2° | **直接接收**（正常跟踪） |
| 大跳 | 0.5~5m / 2~15° | **多帧一致确认**：缓存候选，连续 K=3 帧互相一致（≤0.5m/2°）才作为**台阶修正**吸收（只修 p/θ，**不清零偏、不清协方差**）；不一致→清缓存拒收 |
| 超限 | Δ > 5m / 15° | **拒收**（伪跳变特征），计数并拉大后续观测 R |

> 与 TRACKING 的 jump guard **同构不同标的**：jump guard 防错误修正写进 map_odom（§见主优化方案）；这里是 DR 观测门控，防错误位姿污染滤波器状态。二者共享"几何相似误匹配"这一防御对象，但互不复用代码（作用域不同）。

#### 4.1.3 ESKF 观测接口（dr_odometry 侧，P0）

`reset()` 只写名义状态、无协方差更新——零偏学不到，**不可用**。需新增正规观测更新：

```cpp
// DrEskf（与 use_gnss_* 同风格开关）
bool use_external_pose = true;              // 外部位姿观测总开关
double external_pose_noise_trans = 0.05;    // 观测噪声（m）
double external_pose_noise_yaw_deg = 0.5;   // 观测噪声（deg）
double external_vel_noise = 0.2;            // 速度观测噪声（m/s）

void feedExternalPose(const Pose3d& p_map_base, const Vec3d& v_body, double stamp);
// 内部：创新门控(§4.1.2) → H 作用于 [p, v, θ] → update() → injectError()
// 首帧观测：reanchor（定 map 系基准，保零偏与协方差结构）
```

配置开关保持现有风格（`use_wheel` / `use_gnss*` / `can_use_valid_flag` 不动，新增 `use_external_pose`）。

### 4.2 SLAM→DR 接管

**触发**：`slam_timeout_sec(0.5s)` 无 SLAM 且 DR 源新鲜 + 通过接管准入。

**锚定**（方案 c）：统一源 `T_odom_base(t) = map_odom(t0)⁻¹ ⊕ T_map_base_dr(t)`，`map_odom(t0)` 冻结——map_base 构造连续（§3.2）。

**接管准入分级**（辅助模式的可信度指标）：

| 指标 | 说明 |
|---|---|
| bias 收敛度 | `√P_bg` 对角低于门限 + 辅助观测有效时长 ≥ `min_aid_duration_sec(30s)` |
| 观测创新统计 | 近 10s 创新 RMS 在门限内 |
| 拒收率 | 伪跳变拒收占比 < 30% |

| 分级 | 策略 |
|---|---|
| good | 正常接管，走完整恢复预算 |
| weak | 接管但直接 DEGRADED 输出，恢复预算减半 |
| bad | **跳过 DR 保活**（`dr_bad_admission: skip`），直接 TRACKING_LOST→reloc；或 `keep_alive` 仅保活+DEGRADED |

> 注：辅助模式下无法用"A(t) 漂移率"评估（观测使 DR 贴着定位走，读数恒小）——准入指标如上，取代 v1 稿的 A(t) 方案。

**若 DR 不可用/准入 bad 且 skip**：`slam_timeout` 语义退回现状——LOCALIZATION_FAILURE（reason="all odometry sources lost"）。

### 4.3 DR 模式：匹配与失效判定（Q2/Q3）

**机制全部不变**，只换源数据：

| 机制 | DR 模式 | 说明 |
|---|---|---|
| GM 质量三级 / jump guard / 失败快计数 / 出图快路径 / 无修正看门狗 | **不变** | 同一套阈值与逻辑 |
| 恢复（reloc） | **不变** | reloc 用原始 `/ouster/points`，不依赖 SLAM/DR |
| 人工位姿 + 验证 | **不变** | 验证匹配用 fallback 云 |
| 状态机五态转移 | **不变** | 切换只在源管理层 |
| 源点云 | **换 fallback** | SLAM 注册云死掉 → 原始点云按统一 odom 位姿注册（下） |
| 源超时语义 | **扩展** | §4.5 |

**fallback 云注册**（保留 GM 配置不动——GM 的接口就是"源点云(源系)+源里程计"）：

```
T_odom_lidar = T_odom_base(t_cloud) ⊕ T_base_lidar        （T_base_lidar 复用 GM 外参）
cloud_in_odom = raw_cloud(/ouster/points) ⊕ T_odom_lidar   （扫描时刻位姿，帧间插值）
→ /localization/source_cloud（frame_id=odom）──► GM source_cloud
```

**预期管理**：DR 漂出 GM 匹配收敛域后匹配失败是预期内——失败计数→TRACKING_LOST→reloc 全局恢复。GM 能匹配的时间窗就是有效桥接窗。

### 4.4 DR→SLAM 回切（Q3 丝滑性）

**触发（滞回）**：SLAM 连续 `switch_back_confirm_frames(5)` 帧新鲜；60s 内切换超 `flap_guard_count(3)` 次 → 保持 DR 并告警。

**回切重锚**（缺口/归零误差全部吸收进 map->odom）：

```
map_odom_new = map_base(t1⁻) ⊕ (odom_base_slam(t1))⁻¹
⇒ map_base(t1⁺) = map_base(t1⁻)   严格连续
```

- SLAM 数据缺口型 / 节点重启归零型恢复，公式均成立；
- 回切后 `|T_odom_base_dr ⊖ odom_base_slam|` > 3m/15° 记 WARNING（"slam gap absorbed"）；
- twist 源同步切回；TF/话题无跳变。

### 4.5 失效判定扩展与状态语义

**状态机五态与转移完全不变**。仅 `failure_detector` 源信号语义扩展：

| 信号 | 现状 | 新行为 |
|---|---|---|
| `slam_timeout_sec(0.5s)` | → 立即 LOCALIZATION_FAILURE | → **切源事件**（DR 可用则切 DR）；双源皆失 → FAILURE（"all odometry sources lost"） |
| `dr_timeout_sec(0.5s)`（新） | — | DR 模式下 DR 断流 → LOCALIZATION_FAILURE |
| `max_dr_duration_sec(120s)`（新） | — | DR 超时仍无 SLAM → FAILURE（"DR fallback exceeded max duration"） |
| DR 漂移监视（新） | — | GM good 修正 vs DR 续推：>2m/5° 告警；>3m/10° 直接 TRACKING_LOST 交恢复 |
| GM 计数 / 出图 / 看门狗 / lost 预算 | 不变 | 不变 |

**状态语义（下游契约不变）**：

| 状态 | 语义 | 下游 |
|---|---|---|
| TRACKING_LOST | 定位不可信、正在恢复 | 停车（降级位姿仍输出） |
| LOCALIZATION_FAILURE | 恢复失败/无法恢复 | 停车 + 标记不可用 |

DR 模式下状态可保持 TRACKING，`level=WARNING`（reason="slam odometry lost, running on DR"），`used_sources` 追加 `dr_odometry`，`last_dr_age` 真实化（消息字段已有，无需改 msg）。

### 4.6 DR 使用约定（Q4）

**开关矩阵（机制保留，只定推荐默认）**：

| 场景 | use_imu_accel | use_wheel | use_gnss | use_gnss_heading | use_gnss_velocity | use_gnss_position | use_external_pose |
|---|---|---|---|---|---|---|---|
| **无 GPS 矿井（默认）** | true | true | **false** | — | — | — | **true** |
| 有 GPS | true | true | true | true（车速≥0.2） | false | false | true |

- 无 GPS 时 DR = 陀螺姿态 + 轮速纵向速度 + 非完整约束；位置是速度积分——**只信姿态与短时推进，不信绝对位置**；`use_gnss_position=false` 保持；
- 轮速/GNSS 开关、`can_use_valid_flag` 语义原样保留。

**短时推理正确性保障**：

1. 观测标定（§4.1）→ bias 收敛、姿态/速度对齐，接管即好估计；
2. 接管准入分级（§4.2）→ 坏 DR 不无声接管；
3. 接管后漂移监视（§4.5）→ 超限直接交恢复；
4. `max_dr_duration_sec=120` 上限保护。适用窗口：辅助后 1~3 分钟量级，仍受轮速标度/打滑限制。

**dr_odometry 其他改进（后续，不阻塞）**：冷启动重力对齐；frames 改名 `dr_odom`；输出协方差/漂移率到 status。

**进阶（P6，可选）**：DR 模式下 GM 间歇匹配成功时，把 GM 位姿低频回喂 DR（同一观测接口、R 拉大）——桥接期持续校准，桥接窗再延长。

---

## 5. 代码方案

### 5.1 模块划分

```
dr_odometry/                                  【P0】
├── core/dr_eskf.hpp/.cpp          [扩展] feedExternalPose 观测更新（H 作用 p/v/θ，创新门控）
├── ros/ros_adapter.hpp/.cpp       [扩展] 订阅 FusionStatus + map->base odom；门控 + 跳变判断
└── config/dr_odometry.yaml        [扩展] use_external_pose / 噪声 / 门控参数

localization_v3/
├── core/
│   ├── local_odom_source.hpp/.cpp        [新] 统一源管理：透传/投影锚定(map_odom(t0)⁻¹)/回切重锚/滞回/准入
│   ├── failure_detector.hpp/.cpp         [扩展] 源超时语义（slam→切源，dr→失效，双失→失效）+ 漂移监视
│   └── fusion_core / jump_guard / quality_gate / state_machine / init_arbiter  【不变】
├── ros/
│   ├── fallback_cloud_registrar.hpp/.cpp [新] 原始点云→odom 系注册
│   ├── source_bridge.hpp/.cpp            [新] 统一源话题发布（odom_source / source_cloud）
│   ├── status_publisher.hpp/.cpp         [扩展] used_sources/last_dr_age（消息字段已有）
│   └── localization_v3_node.cpp          [扩展] 订阅 DR + raw cloud；驱动 LocalOdomSource
└── config/localization_v3.yaml           [扩展] odometry_source / fallback_cloud 两段
```

### 5.2 dr_odometry 侧接口（P0 详情）

| 变更 | 内容 |
|---|---|
| `DrEskf::feedExternalPose(p_map_base, v_body, stamp)` | 正规观测更新（Joseph）；首帧 reanchor（定 map 基准，保零偏）；创新门控内嵌（小跳直收/大跳 K 帧一致/超限拒） |
| ros_adapter 订阅 | `/localization_v2/status`（FusionStatus）+ `/localization_v2/odom`（map->base） |
| 门控逻辑 | §4.1.1 条件 + §4.1.2 跳变判断（放 ros 层，core 只做更新） |
| 配置 | `use_external_pose / external_pose_noise_* / external_vel_noise / obs_slam_age_limit / obs_require_level_normal / jump_confirm_frames(3) / jump_soft/hard 阈值` |
| 开关模式 | 与 `use_gnss_*` 并列，可独立开关；`use_external_pose=false` 时行为=现状 |

### 5.3 接口变化汇总（不变项是重点）

| 位置 | 变化 |
|---|---|
| `fusion_core` / `jump_guard` / `quality_gate` / `state_machine` / `init_arbiter` | **无** |
| GM（global_matcher_v2）配置 | **无**（仅 launch 把 source_odom/source_cloud 指到统一源话题） |
| relocalization | **无** |
| `failure_detector` | 源超时语义拆分 + 漂移监视 |
| `status_publisher` | DR 源信息（msg 字段已有） |
| launch | 启动 `dr_odometry`；GM 两话题改指 `/localization/odom_source`、`/localization/source_cloud` |

### 5.4 配置（localization_v3 新增）

```yaml
odometry_source:
  slam_topic: /localization/slam_odom
  dr_topic: /localization/dr_odom
  dr_mode: hot                      # hot(辅助热备,推荐) | off
  slam_timeout_sec: 0.5             # SLAM 断流 → 切 DR
  dr_timeout_sec: 0.5               # DR 模式下 DR 断流 → 失效
  switch_back_confirm_frames: 5     # 回切滞回
  flap_guard_count: 3
  max_dr_duration_sec: 120
  min_aid_duration_sec: 30          # 接管准入：辅助观测有效时长下限
  admission_max_innovation_rms: 0.5 # 接管准入：创新 RMS 上限（m）
  admission_max_reject_ratio: 0.3   # 接管准入：伪跳变拒收占比上限
  dr_drift_warn_trans: 2.0          # 接管后漂移监视（m/deg）
  dr_drift_warn_yaw_deg: 5.0
  dr_drift_fail_trans: 3.0
  dr_drift_fail_yaw_deg: 10.0
  dr_bad_admission: skip            # skip | keep_alive

fallback_cloud:
  enabled: true
  raw_cloud_topic: /ouster/points
  source_cloud_topic: /localization/source_cloud
  odom_source_topic: /localization/odom_source
  base_to_lidar: {translation: [0,0,0], rotation: [1,0,0, 0,1,0, 0,0,1]}
```

dr_odometry 新增（见 §5.2）：`dr/use_external_pose / external_pose_noise_trans / external_pose_noise_yaw_deg / external_vel_noise / obs_slam_age_limit / obs_require_level_normal / jump_soft_trans / jump_soft_yaw_deg / jump_hard_trans / jump_hard_yaw_deg / jump_confirm_frames`。

### 5.5 时序

```
热备   SLAM 正常：定位 status+odom 观测 DR（§4.1 门控+跳变判断），bias/速度/姿态收敛
       统一源=SLAM 透传，一切同现状
t0     SLAM 断流 0.5s：接管准入评估（bias 收敛度/创新 RMS/拒收率）
       good/weak → 统一源 = map_odom(t0)⁻¹ ⊕ T_map_base_dr（map_base 严格连续）
       bad+skip → 不接管，直接 TRACKING_LOST→reloc
t0+    DR 保活；GM 用 fallback 云继续匹配（机制不变）；漂移监视在跑
       （GM 修不动 → TRACKING_LOST → reloc 全局恢复 = 正常路径）
t1     SLAM 恢复（连续 5 帧新鲜）→ 回切重锚 map_odom_new = map_base(t1⁻) ⊕ odom_base_slam(t1)⁻¹
       统一源切回 SLAM 透传；used_sources 去掉 dr_odometry
```

---

## 6. 测试与验收

| 用例 | 通过标准 |
|---|---|
| 单测：`feedExternalPose`（P0） | 仿真零偏下辅助 N 秒后 bias 收敛（误差<50%）；跳变判断三档正确（小跳直收/大跳 K 帧确认/超限拒）；拒收伪跳变不污染 bias |
| 单测：门控 | status 非 TRACKING / SLAM 不新鲜（防闭环）/ level 门 → 不观测 |
| 单测：`LocalOdomSource` | 投影锚定后 map_base(t0⁺)=map_base(t0⁻)（<1e-6）；GM 修正 map_odom 后 map_base 正常响应；回切重锚连续 |
| 单测：滞回/准入 | 抖动流不翻转；bad→skip 不接管；weak→预算减半 |
| 单测：failure_detector 源信号 | slam 断→切源不判死；DR 断→失效；双失→"all odometry sources lost" |
| bag：辅助标定有效性 | 同路线两遍（开/关 use_external_pose）对比接管后漂移率，预期降一个量级 |
| bag：kill slam 节点 | 0.5s 切 DR，map_base 无跳变；GM 继续匹配或按预期进 LOST→reloc |
| bag：伪跳变注入 | 人为制造 GM 误修正，DR 观测拒收、bias 不被污染 |
| bag：SLAM 断 30s 再恢复 | 回切连续、gap 吸收、无输出尖峰 |

---

## 7. 决策点与风险

| # | 决策 | 结论 |
|---|---|---|
| D1 | 热备 vs 按需 | **辅助标定热备**（成本可忽略、接管零预热、bias 已收敛）；按需冷启动否决 |
| D2 | 坐标系 | **DR 世界系 = map 系**，观测零转换（§3.2 方案 c） |
| D3 | 观测源 | 定位 map->base（status 门控），twist 速度并观测 |
| D4 | 跳变判断 | 创新三档 + 大跳 K=3 帧一致确认；超限拒收 |
| D5 | DR 模式 GM 匹配 | 继续（fallback 云），尽力而为；机制不变 |
| D6 | DR 模式状态语义 | 状态不变 + level=WARNING + used_sources 标注 |
| D7 | 接管准入 bad 处置 | **skip**（直接走恢复）；keep_alive 可配 |
| D8 | DR 时长上限 | 120s（预期实际远短于此） |

风险与缓解：

| 风险 | 缓解 |
|---|---|
| fallback 云无去畸变，GM inlier 偏低 | 扫描时刻位姿插值注册；bag 对比 inlier 分布 |
| 观测被定位伪跳变污染 | §4.1.2 跳变判断（本方案核心防线）+ 拒收率进接管准入 |
| DR 与 SLAM 误差相关（非独立备份） | 目标是连续性桥接非独立冗余；漂移监视兜底 |
| DR 漂移出 GM 匹配域 | 预期内；TRACKING_LOST→reloc 是正常路径 |
| 双源抖动 | 回切滞回 + flap_guard |

---

## 附录 A：实施拆分

| 阶段 | 内容 | 涉及 |
|---|---|---|
| P0 | `feedExternalPose` + 观测门控（订阅 status/odom、跳变判断）+ 单测 | dr_odometry |
| P1 | `core/local_odom_source`（投影锚定/回切重锚/滞回/准入）+ 单测 | localization_v3 |
| P2 | `failure_detector` 源信号 + 漂移监视 + 单测 | localization_v3 |
| P3 | `fallback_cloud_registrar` + `source_bridge` + node/launch/config | localization_v3 |
| P4 | status_publisher 扩展 + 文档收尾 | localization_v3 |
| P5 | bag 回归（辅助有效性/断流/伪跳变/回切） | 测试 |
| P6（可选） | GM 位姿低频回喂 DR；frames 改名；重力对齐 | 两者 |
