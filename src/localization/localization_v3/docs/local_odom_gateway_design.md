# local_odom_gateway：多源局部里程计统一接入与无缝切换 — 设计文档

> 版本：v1.0（设计稿，**未含代码**）
> 日期：2026-10-12
> 关联：`dr_odometry_fallback_design.md`（DR 接入原方案）、`多源里程计接入改造方案.md`（源注册表思想，本文吸收）、`gap_fill_switch_optimization.md`（上一轮修补记录，本文取代其机制部分）
> 范围：**替代** 当前 `source_bridge_node` + `core/local_odom_source` 的实现；`dr_odometry_v2` 观测接口微调；fusion/GM/状态机**零改动**
> 状态：设计定稿待评审。**推翻现有实现**，保留的代码在 §10 列出。

---

## 0. 要求与验收标准

| # | 要求 | 验收判据 |
|---|---|---|
| **R1** | 全局定位结果作 DR 观测约束（绝对观测） | 观测值 = `map_base`（GM 校正后的全局位姿），不是 LIO 里程计 |
| **R2** | SLAM↔DR 切换**无缝、实时、无卡顿跳变** | 切换瞬间 `‖unified⁺ − unified⁻‖ < 1e-6`；断流期输出不冻结；无一次性台阶 |
| **R3** | 下游不用感知 | fusion / GM / 状态机 / 控制**零改动**，只换话题输入 |
| **R4** | SLAM 运行时 DR 不参与 | SLAM 样本正常节拍期间，运动链的表达式里**没有 DR**（结构性，不是阈值性） |
| **R5** | 两条链路区分清楚 | 观测链（标定）与运动链（输出）**相位互斥**（§4.5 不变式 I1/I2），正常运行时互不干扰 |
| **R6** | 统一入口转发为标准话题 | **仅前端里程计**（SLAM/DR/后续其它里程计）统一管理；**传感器（IMU/CAN 等）由各自节点直订**，不经本模块 |
| **R7** | 模块化接口化，易接其他里程计 | 新增源 = 配置 1 条 + 订阅 3 行，核心零改动 |
| **R8** | 去除无用/错误代码 | §10 逐条列出删除项 |
| **R9** | 代码不冗余、模块/逻辑清晰 | core 纯 C++ 库（可单测）+ 薄 ROS 层 + 双前端；单文件单职责 |

---

## 1. 为什么推翻当前实现

### 1.1 根因：一条链路承担了两个职责

当前 `LocalOdomSource` 里，**DR 的位姿差分被写进运动链**：

```cpp
// local_odom_source.cpp:246-248（现状）
if (gapFillActive(...)) {
  out->motion = compose(out->anchor, compose(inverse(dr_at_slam_), last_dr_));  // ★ DR 进运动链
}
```

而 `gapFillActive` 的触发是**时间阈值**（`age > 0.15s`），早于换源阈值（`0.25s`）。后果：

1. **SLAM 还是主源时 DR 就进了输出**——只要周期抖动跨过阈值就启停一次（实测 `slam_age` 抖到 0.124s，阈值 0.15s 只差 26ms）→ 间隔性毛刺；
2. DR 增量的一切误差（观测台阶、配对错位、零速度污染）**直接变成 SLAM 轨迹上的跳变**；
3. 三处阈值（`gap_fill_delay` / `slam_timeout` / `slam_age_limit`）互相耦合，调一个动两个。

### 1.2 其它结构性缺陷

| # | 缺陷 | 位置 |
|---|---|---|
| a | **透传被重采样**：100Hz ZOH + `now()` 戳，丢掉扫描戳与原频率 → 与纯 SLAM 喂给 fusion/GM 的流不一致 | `source_bridge_node.cpp:257-258` |
| b | **断流定义被污染**：`slam_require_cloud=true` 把"注册云断"当"里程计断流"→ 里程计活着也换源 → GM 输入突变 | `local_odom_source.cpp:45-57` |
| c | **观测值不是绝对位姿**：喂的是纯 LIO 位姿（带漂移），全局校正的绝对信息被丢弃 | `dr_odometry_v2.yaml: topics/localization_pose` |
| d | **门控有洞**：`slam_require_cloud` 触发换源时 SLAM 探针仍新鲜 → 观测继续 → 自回灌 | `dr_odometry_v2_node.cpp:153-161` |
| e | **硬对齐清零速度**：`applyPoseStep` 覆盖位姿 + 速度清零 + 协方差重写 | `eskf.cpp:460-484` |
| f | 双 glue + 换源换算把 DR 误差烘进长期坐标系；冷启动 glue≠I | 已部分修补（冷启动），结构仍复杂 |
| g | 源抽象二元硬编码（`kSlam/kDr`），接第三个源要改 4 处 | `local_odom_source.hpp` |

### 1.3 设计原则（从教训提炼）

> **P1. 一条链路一个职责。** 运动链只管"位姿怎么推进"，观测链只管"怎么标定 DR"，两链不得回流。
> **P2. 绝对位姿与运动增量分离。** 绝对位姿来自定位链（odom 系），运动增量来自 DR（与 DR 世界系无关）。
> **P3. 应用时机分层。** DR 增量只在"DR 成为运动源"后应用；SLAM 期只算不应用（热备）。
> **P4. 切换连续性由构造保证，不靠调参。** `glue_new = unified(t_sw⁻) ⊕ T_new(t_sw)⁻¹`。
> **P5. 契约显式。** 话题、坐标系、时间戳、单位全在接口层定义，核心零 ROS。

---

## 2. 总体架构：两条链路

```
【观测链 · 标定】（仅 kRunning 相位通）
   fusion 的 map_odom (/localization/map_to_odom)
        │  ⊕ anchor（纯源位姿，扫描戳）
        ▼  (门控：相位互斥 —— 结构性)
   /localization/odometry_observation ──► dr_odometry_v2（ESKF 观测约束）
                                              │ 标定 bias / 速度 / 姿态
                                              ▼
【运动链 · 输出】                                          DR 状态收敛
   slam_odom ──►┐
   dr_odom  ──►│  SourceRegistry ─► Arbiter ─► MotionProvider ─► odom_source / source_cloud
   (vio...)──►┘        │                              │              source_status
        │              │                              └── 锚 + 增量
        │              └─ 云统一（registered 透传 / raw_register 注册）
        └─ 原始点云 /ouster/points ──► CloudUnifier（raw_register 用）
                                       ↑ 仅点云（原始云是注册输入，不是传感器总线）

【传感器 · 各自节点直订，不经本模块】
   IMU ──► dr_odometry_v2（预测）     CAN ──► dr_odometry_v2（轮速）
```

> **范围界定**：本模块只统一管理**前端里程计**（SLAM / DR / 后续其它里程计）及其**点云**。
> IMU、CAN 等传感器由各自节点直接订阅——高频流多一跳只增延迟，且它们不是"里程计统一管理"的职责。

**两链不回流的保证**：

| 方向 | 为什么不回流 |
|---|---|
| 观测链 ↛ 运动链 | 观测只进 DR 的 ESKF；运动链用的 `Δ = D(t_a)⁻¹⊕D(t)` 是**差分**，绝对观测的台阶只在观测期施加，而观测期 ⊆ 运动链纯 SLAM 期（DR 增量未应用） |
| 运动链 ↛ 观测链 | 观测值 = `map_odom ⊕ anchor`，anchor 是**纯源位姿**，永不含 DR 增量 |

---

## 3. 坐标系与时间契约

### 3.1 坐标系

| 系 | 定义 | 所有者 |
|---|---|---|
| `map` | 全局地图系 | 地图 / GM |
| `odom` | **主源（LIO）的局部里程计系**，会话期内定义一次 | 主源 |
| `map_odom` | map→odom 修正 | fusion（GM 写入） |
| 源自有系 | 各源自己的里程计系 | 各源 |
| **虚拟系** | gateway 输出所处系 = `odom`（冷启动定为主源系） | gateway |

**契约（硬性）**：gateway 输出的**里程计与源点云必须同处虚拟系**。凡与 `map_odom` 组合的量（fusion 可视化点云、TF、path）必须同系。

### 3.2 时间

| 量 | 时间戳 | 说明 |
|---|---|---|
| 透传的 odom / cloud | **源原始戳** | 不重打、不重采样（治 §1.2a） |
| 锚 `(A, t_a)` | `t_a` = 源样本的**扫描戳** | 配对与外推基准 |
| 增量基准 `D(t_a)` | 按 `t_a` 在 DR 轨迹**插值** | 不是取到达时刻的 DR 状态 |
| 观测 | **扫描戳 `t_a`** | DR 侧按观测时刻传播后再更新 |
| 合成输出（DR 期） | `now()` | 此时无源戳可用；下游 fusion 本就按到达时刻用 |

**时间基归一**：`stampOrNow()` 保留（bag 回放时消息戳与 ROS 时钟不同源的兜底）。

---

## 4. 运动链核心：锚 + 增量

### 4.1 公式

```
T_odom_base(t) = A ⊕ Δ(t_a → t)

A       : 锚 = 最后一个有效样本的位姿（odom 系），带时间戳 t_a
Δ(t_a→t): 运动增量 = D(t_a)⁻¹ ⊕ D(t)      （D = DR 在其自有系的位姿）
```

**关键性质**：`Δ` 是两个位姿的差分，**与 DR 的世界系无关**（相似变换下不变）。所以
> **不需要把 DR 的坐标系对齐到 odom 系，只需要把时间对齐**（按 `t_a` 插值出 `D(t_a)`）。

这消解了"DR 用全局观测（≈map 系）不能直接给下游"的矛盾——我们从不直接使用 DR 的绝对位姿。

### 4.2 应用时机（R4 的落点）

| 阶段 | 判据 | 运动链输出 | DR 增量 |
|---|---|---|---|
| **① SLAM 运行** | 样本新鲜（`age ≤ T_overdue`） | `= A`（纯源） | **不应用**（只算） |
| **② 断流检测窗** | 样本超龄（`T_overdue < age ≤ T_dead`） | `= A ⊕ Δ(t_a→t)` | **应用** |
| **③ DR 接管** | 样本判死（`age > T_dead`） | `= A ⊕ Δ(t_a→t)`（锚冻结） | 应用（同 ②，公式不变） |

> **"SLAM 运行" = 样本在正常节拍到达。** 检测窗内 SLAM 已不"运行"（该来没来），此时用 DR 推进是断流容错，不属于 R4 禁止的范围。
>
> **为什么检测窗要用 DR 而不是 SLAM 自外推**（误差对比，τ = 窗长）：
>
> | 推进源 | 误差来源 | τ=0.15s | τ=0.25s |
> |---|---|---|---|
> | SLAM 一阶外推 | ½aτ² + 位姿噪声→速度噪声·τ | ~2–3 cm | ~5–6 cm |
> | **DR 增量** | 速度误差·τ + 姿态误差·v·τ（IMU+轮速，观测期已标定） | **~0.5 cm** | **~1 cm** |
>
> DR 更准的前提是：**观测链在检测窗内已关闭**（§2），所以 `Δ` 是纯 IMU+轮速传播，不含观测台阶。这就是两条链路分离的直接收益。
>
> 注：若 DR 未标定好（冷启动/刚恢复），可退化为"检测窗冻结 + 短窗"（配置开关 `fallback_to_hold`）。

### 4.3 连续性不变式（R2 的落点）

```
glue_new = unified_motion(t_sw⁻) ⊕ T_new(t_sw)⁻¹
```

- **接管**：`glue_dr = unified(t_sw⁻) ⊕ D(t_sw)⁻¹` ⇒ 切换瞬间输出严格相等，检测窗内的运动量不丢
- **回切**：`glue_slam = unified(t_1⁻) ⊕ T_slam(t_1)⁻¹` ⇒ 同上；吸收的累积误差进入虚拟系（见 §4.4）
- **冷启动**：虚拟系直接定为主源系（`glue_slam = I`），不走换算——此时 `map_odom` 未建立，无连续性可保

### 4.4 虚拟系与 glue 的唯一用途

glue **只在回切时非 I**，用途是吸收 DR 期间的累积误差，保证 `map_base` 不跳。副作用：SLAM 云需 ⊕ `glue_slam` 进虚拟系（PCL 变换）。

> 备选：误差改由 fusion 调 `map_odom_new = map_base(t₁⁻) ⊕ T_slam(t₁)⁻¹` 吸收（设计文档 §4.4 原方案）。**不推荐**——GM 下一次匹配会把 `map_odom` 拉回真实值，误差重新暴露成一次修正跳变。走 glue 则 GM 结果自洽。

### 4.5 两链隔离不变式（R4/R5 的形式化）

| 相位 | 运动链 `T` | 观测链 | DR 增量 `Δ` |
|---|---|---|---|
| **kRunning**（样本正常节拍） | `= A`（纯源） | **开** | **不应用** |
| **kGap**（超龄） | `= A ⊕ Δ` | **关** | 应用（纯传播） |
| **kOnBackup**（判死） | `= A ⊕ Δ`（锚冻结） | **关** | 应用（纯传播） |
| 回切瞬间 | glue 对齐 | 待 stability 重开 | — |

**不变式：**

```
I1  phase == kRunning  ⇒  T 不含 Δ                （DR 不进运动链）
I2  phase != kRunning  ⇒  观测链关闭              （DR 不被观测）
I3  切换瞬间 T(t_sw⁺) == T(t_sw⁻)                 （连续性不变式 §4.3）
I4  anchor 永不含 Δ                              （观测链不回流）
```

**推论（两链互不干扰）**：
- 由 **I1**：kRunning 期运动链表达式里没有 DR ⇒ DR 状态无论怎么被观测修正，**都碰不到输出**；
- 由 **I2**：Δ 被应用的时段（kGap/kOnBackup）观测链是关的 ⇒ Δ 是纯 IMU+轮速传播，**不含观测台阶**；
- 由 **I4**：观测值 `map_odom ⊕ anchor` 里的 anchor 是纯源位姿 ⇒ **运动链不回流进观测链**。

⇒ **正常运行时两链在时间上严格互斥**，唯一的"混合"只发生在切换簿记那一瞬（且位姿公式不变，见 I3）。

**残余风险（需在实现中守住）**：

| # | 风险 | 守法 |
|---|---|---|
| 1 | 配对 `D(t_a)` 若取在观测施加**之前**，观测台阶会漏进 Δ | 配对取自 DR **处理后**的轨迹（按 `t_a` 插值），或退化为取 `t_ov` 时刻 |
| 2 | DR 未标定好时 Δ 误差大 | 接管准入分级（§7.3），bad 时 `skip` 走 reloc |
| 3 | 回切 glue 吸收误差后 SLAM 云要变换一次 | 正确性不受影响，仅一次 PCL 变换开销 |

---

## 5. 模块设计

### 5.1 包结构（core 库 + 双前端）

```
src/localization/local_odom_gateway/
├── include/local_odom_gateway/
│   ├── core/                                 # ★ 纯 C++ 库，零 ROS、零 pub/sub，可单测
│   │   ├── types.hpp                         # 位姿/SE(3)/时间工具
│   │   ├── source_spec.hpp                   # OdomSourceSpec / CloudMode（表驱动）
│   │   ├── source_registry.hpp               # 源注册表 + 运行态 Slot
│   │   ├── arbiter.hpp                       # 仲裁 + 切换连续性 + 滞回/翻转保护
│   │   ├── motion_provider.hpp               # ★ 锚 + 增量 + 相位（运动链核心）
│   │   ├── cloud_unifier.hpp                 # 云统一（registered / raw_register）
│   │   └── observation_gate.hpp              # 观测门控（相位互斥）
│   └── ros/
│       ├── adapters.hpp                      # 各源 ROS 适配（配置驱动订阅，新增源加 3 行）
│       ├── embedded_gateway.hpp              # 前端 A：内嵌 localization_v3_node（默认）
│       └── standalone_gateway.hpp            # 前端 B：独立节点（调试/隔离/复用）
├── src/core/*.cpp
├── src/ros/*.cpp
├── src/standalone_node.cpp                   # 前端 B 的 main
├── config/local_odom_gateway.yaml
├── launch/local_odom_gateway.launch
└── test/  test_motion_provider / test_arbiter / test_observation_gate
```

**分层职责（不冗余的边界）**：

| 层 | 职责 | 依赖 |
|---|---|---|
| `core/` | 仲裁、连续性、锚+增量、云统一、观测门控 | 仅 Eigen（+ STL） |
| `ros/adapters` | 话题 → core 样本；core 结果 → 话题 | core + roscpp |
| `ros/embedded` | 给 `localization_v3_node` 当成员用，进程内 API 直取 | adapters |
| `ros/standalone` | 独立节点，同一套 adapters | adapters |

**内嵌 vs 独立（默认内嵌）**：

| | 内嵌（默认） | 独立节点 |
|---|---|---|
| 点云 hop | **1 跳**（gateway 与 fusion 同进程） | 2 跳 |
| fusion 取运动 | 进程内 `gateway.sample()`，无序列化 | 走话题 |
| 对 GM / DR | 仍走话题（它们是别的进程） | 同 |
| 崩溃连坐 | 与定位同进程 | 隔离 |
| 用途 | 上车默认 | 调试 / 复用 / 降级 |

> 模块化由「core 是纯 C++ 库 + 不变式单测」保证，**与跑在哪个前端无关**；新增里程计源只改 `adapters` 的订阅与 YAML。

### 5.2 核心接口

```cpp
// ── 源规格（配置驱动，新增源只加一条） ──────────────────────────
enum class CloudMode { kNone, kRegistered, kRawRegister };

struct OdomSourceSpec {
  std::string name;                 // "slam" / "dr" / "vio" ...
  int  priority        = 0;         // 高者优先（slam=10, dr=5, vio=7）
  double timeout_sec   = 0.5;       // 样本判死阈值
  double overdue_sec   = 0.0;       // 样本超龄阈值（0 = 用 priority 主源的周期自适应）
  int  switch_back_confirm = 5;     // 降级源回切滞回帧数
  CloudMode cloud_mode = CloudMode::kNone;
  std::string odom_topic, cloud_topic;
};

// ── 源运行态 ────────────────────────────────────────────────
struct SourceSlot {
  OdomSourceSpec spec;
  Pose3d last_pose;                 // 最近样本（源系）
  double  last_stamp = -1.0;
  Twist   last_twist;
  Pose3d  glue;                     // 源系→虚拟系（切换时计算）
  bool    established = false;      // 虚拟系是否已由该源确立
  int     confirm_count = 0;
  // 周期自适应：实测样本周期 EMA，用于 overdue/dead 阈值
  double  period_ema = -1.0;
};

// ── 运动链核心 ──────────────────────────────────────────────
struct Anchor {
  Pose3d pose;        // A（虚拟系）
  double stamp = -1;  // t_a（扫描戳）
  Pose3d dr_at;       // D(t_a)（DR 自有系，按戳插值）
  bool   valid = false;
};

class MotionProvider {
 public:
  // 样本到达：落锚 + 配对 D(t_a) + 更新周期 EMA
  void onSample(const Pose3d& pose, double stamp, int source_id);
  // DR 样本：维护轨迹历史（供插值）
  void onDr(const Pose3d& pose, double stamp);
  // 取运动输出（内部决定是否用增量）
  bool motion(TPose* out, double now) const;
  // 取纯源锚（观测链用；无副作用）
  bool anchor(TPose* out, double stamp) const;
  Phase phase() const;   // kRunning / kGap / kOnBackup
};
```

### 5.3 Arbiter（仲裁 + 切换）

```
候选 = 所有 fresh 的源
目标 = 候选中 priority 最高者
无候选 → 无输出（下游看门狗判失效）

目标 ≠ active:
  目标.priority > active.priority  → 立即切（主源恢复/更好源出现）
  否则（降级）                    → 需 switch_back_confirm 帧滞回
  切换时：glue_目标 = unified(t_sw⁻) ⊕ T_目标(t_sw)⁻¹     ← §4.3 不变式
  翻转保护：flap_window 内切换 ≥ flap_guard → confirm 帧加倍
```

**与源数量无关**（R7）：连续性公式对任意源对成立。

### 5.4 CloudUnifier

| CloudMode | 处理 |
|---|---|
| `kRegistered` | 源自带注册云 → ⊕ `glue` 进虚拟系发布（glue=I 时**零成本原样透传**） |
| `kRawRegister` | 原始点云 ⊕ `(unified_pose ⊕ T_base_lidar)` 注册发布 |
| `kNone` | 无云（GM 无法配，状态机自行降级） |

**外参红线**：`T_base_lidar` 必须与该传感器真实安装一致，且与 LIO 内部注册外参（`extrinsic_R`）同源。启动时打印自检日志。

---

## 6. 观测链设计（R1 / R5）

### 6.1 观测值

```
z = map_odom ⊕ anchor          ≡  map_base   （GM 校正后的全局位姿）
```

- 由 **gateway 发布**（它有 anchor，订 fusion 的 `/localization/map_to_odom`）
- 等价性：`map_odom ⊕ odom_base = map_base`，用合成式的好处是**能挂在结构性门控上**（见下）

> **为什么不直接订 `/localization/fused_odom`**：那是 200Hz 连续流，无法表达"只在纯 SLAM 瞬间观测"；且在检测窗内它已含 DR 增量 → 自回灌。

### 6.2 门控（结构性，非阈值性）

| 门 | 条件 | 防什么 |
|---|---|---|
| **G1 结构门** | **锚事件驱动**：只在新样本到达、且运动链处于 `kRunning` 时发观测 | 自回灌（DR 增量期/DR 期自动断） |
| G2 状态门 | fusion `status == TRACKING && output_valid` | 劣质结果不进滤波 |
| G3 质量门 | 跳变三档（小跳直收 / 大跳 N 帧确认 / 超限拒） | GM 伪跳变污染 bias |

**G1 的关键**：观测由**锚消息**驱动（收到锚才观测一次），不是"时间窗 + 200Hz 流"。锚只在 `kRunning` 发布 ⇒ 观测窗口 ⊆ 运动链纯 SLAM 窗 ⇒ **结构上不可能自回灌**，不再依赖 `slam_probe` 之类的活性近似（治 §1.2d）。

### 6.3 时间戳

观测带**扫描戳 `t_a`**。DR 侧需支持"迟到观测按其时刻处理"：
- 首选：事件队列对 `kPose` 事件放宽迟到容忍（`max_latency`），先 `propagateTo(t_a)` 再更新
- 退化：按当前状态更新，接受创新量含 `v·latency`（配 `max_latency` 上限）

### 6.4 观测对 DR 的作用范围

| 项 | 是否更新 | 理由 |
|---|---|---|
| p / θ | ✓ | 标定姿态与位置 |
| bias（gyro/accel） | ✓ | **绝对观测的核心收益**：gyro bias 可观，yaw 漂移率降一个量级 |
| v | **默认不观测** | 消息 twist 无有效值时不得当"零速度"；配置 `use_velocity` |
| 协方差 | ✓ Joseph 形式 | — |
| **硬对齐**（大跳确认） | **只改 p/θ，不清 v / 不重置协方差** | 治 §1.2e |

---

## 7. 切换时序

### 7.1 SLAM → DR（断流接管）

```
t_a          锚落档 (A, t_a)，配对 D(t_a)，周期 EMA 更新
             phase = kRunning    运动链 = A        观测链 = 开
t_ov         超龄（age > 1.3×周期EMA）
             phase = kGap        运动链 = A ⊕ Δ    观测链 = 关 ★
t_sw         判死（age > 2×周期EMA 或 timeout_sec）
             phase = kOnBackup   运动链 = A ⊕ Δ（锚冻结，公式不变）
             簿记：云切 raw_register、status 标注 used_sources
```

**切换瞬间位姿公式一个字不改** ⇒ 无缝（R2）。实时性 = 判死延迟 ≈ 1 个周期 + 抖动余量（~0.1s），优于现状 0.15s。

### 7.2 DR → SLAM（回切）

```
t_b          SLAM 样本恢复 → 滞回确认 N 帧（翻转保护可能加倍）
t_1          换源：glue_slam = unified(t_1⁻) ⊕ T_slam(t_1)⁻¹   ← 误差吸收进虚拟系
             云路径 ⊕ glue_slam 进虚拟系
             phase = kRunning，锚换新，观测链待 stability 后重开
```

### 7.3 边界情况

| 情况 | 处理 |
|---|---|
| 双源皆失 | 无输出 → fusion 看门狗判 LOCALIZATION_FAILURE |
| DR 未标定好（冷启动） | 接管准入分级：good/weak/bad；bad 可 `skip` 直接走 reloc |
| 主源抖动（周期不稳） | 周期 EMA 自适应 + 回切滞回 + 翻转保护 |
| 云断但里程计活 | **不断流、不换源**（治 §1.2b）；只置 GM 降级/告警标志 |

---

## 8. 话题契约（R3 / R6）

### 8.1 输入（配置驱动；**只收前端里程计 + 点云**）

| 用途 | 默认话题 | 说明 |
|---|---|---|
| 主源 odom | `/slam_odometry/odom` | 源系位姿 + 原始戳 |
| 主源注册云 | `/slam_odometry/cloud_effect` | `cloud_mode=registered` |
| 备源 odom | `/dr_odometry/odom` | — |
| 原始点云 | `/ouster/points` | 仅 `raw_register` 模式的注册输入 |
| map_odom | `/localization/map_to_odom` | 观测链合成用 |

> **不收传感器**：IMU / CAN 等由 `dr_odometry_v2` 等节点自己订阅（R6）。

### 8.2 输出

| 话题 | 内容 | 消费者 |
|---|---|---|
| `/localization/odom_source` | `T_odom_base`（虚拟系）+ twist | GM（`source_odom`）；**fusion 内嵌时进程内直取，不走此话题** |
| `/localization/source_cloud` | 源点云（虚拟系） | GM（`source_cloud`）、fusion 可视化 |
| `/localization/odometry_observation` | `map_odom ⊕ anchor`（扫描戳） | **dr_odometry_v2**（标定） |
| `/localization/source_status` | active 源、phase、健康、周期 EMA | 监控 / 日志 / 调试毛刺 |

> **下游零改动**：GM 只需 launch 里把 `topics/source_odom` / `topics/source_cloud` 指到上述话题（与现状一致）；
> fusion 在内嵌模式下由 API 直取（launch 里 `topics/slam_odom` 那条 remap 可删）。

---

## 9. 配置（`config/local_odom_gateway.yaml`）

```yaml
frames:
  odom: "odom"            # 虚拟系 = 主源系
  base_link: "base_link"

sources:
  slam:
    priority: 10
    odom_topic: /slam_odometry/odom
    cloud_topic: /slam_odometry/cloud_effect
    cloud_mode: registered
    timeout_sec: 0.25        # 判死（也可0 = 用 2×周期EMA）
    overdue_sec: 0.15        # 超龄（0 = 用 1.3×周期EMA）
    switch_back_confirm: 5
  dr:
    priority: 5
    odom_topic: /dr_odometry/odom
    cloud_mode: raw_register
    timeout_sec: 0.5
    switch_back_confirm: 5

cloud:
  raw_cloud_topic: /ouster/points     # 仅 raw_register 注册输入（不是传感器总线）
  base_to_lidar:
    translation: [0, 0, 0]
    rotation: [0,1,0, -1,0,0, 0,0,1]   # 必须与 LIO extrinsic_R 一致

observation:
  enabled: true
  map_odom_topic: /localization/map_to_odom
  status_topic: /localization/status
  require_tracking: true
  use_velocity: false          # 消息 twist 无有效值时不得当"零速度"观测
  max_latency: 0.5

switching:
  flap_guard_count: 3
  flap_window_sec: 60.0
  admission_min_aid_sec: 10.0  # DR 接管准入：标定时长下限
  fallback_to_hold: false      # DR 未标定时退化为"检测窗冻结"

output:
  rate: 100.0                  # 仅 DR 期合成输出用；SLAM 期原样转发不走此定时器
```

---

## 10. 与现有代码的映射（R8）

### 保留（搬进新包）

| 现有 | 去向 | 说明 |
|---|---|---|
| `core/types.hpp` | `local_odom_gateway/core/types.hpp` | SE(3) 工具，原样 |
| `LocalOdomSource` 的连续性换算、滞回、翻转保护 | `Arbiter` | 语义保留，改成表驱动 |
| `drPoseAt()` 插值 | `MotionProvider` | 时间对齐核心，保留 |
| `onSlamCloud` 的 glue 云变换 | `CloudUnifier` | 桥契约，保留 |
| `stampOrNow()` | `ros/adapters.hpp` | bag 回放兜底，保留 |
| `test_local_odom_source.cpp` 的 12 个不变式用例 | `test_motion_provider.cpp` | 语义迁移 |

### 重写

| 现有 | 问题 | 新做法 |
|---|---|---|
| `sample()` 的 gap 公式 | 阈值触发 + DR 进运动链 | `MotionProvider` 三相位（kRunning/kGap/kOnBackup），相位由周期 EMA 驱动 |
| `source_bridge_node` 的 100Hz ZOH 重发 | 丢戳/重采样 | SLAM 期**原样转发**；仅 DR 期走合成定时器 |
| 断流判定 | 云条件污染 + 固定阈值 | 只看 odom；阈值随周期 EMA 自适应 |
| 观测源 | 纯 LIO 位姿 | `map_odom ⊕ anchor`，锚事件驱动 |

### 删除

| 项 | 理由 |
|---|---|
| `gap_fill_delay_sec` / `gapPairing` 相关命名 | 语义误导（"gap 填充"实为"检测窗推进"），改叫 `overdue_sec` / phase |
| `slam_require_cloud` 参与断流判定 | 云断 ≠ 里程计断 |
| `glue_dr` 的换算烘入 gap 增量 | 用 §4.3 通用不变式 |
| `flap_warned_`（core 内） | core 无日志依赖，改由 Snapshot 抛出 |
| `pose_noise_yaw_deg` 死参数 | 已接线（保留） |
| 双源枚举 `kSlam/kDr` | 改 `SourceId` 表驱动 |

---

## 11. 测试与验收

### 单元测试（core 零 ROS）

| 用例 | 锁定 |
|---|---|
| `RunningPhaseExcludesDrIncrement` | **R4**：相位 kRunning 时 `motion == anchor`，DR 移动不进链 |
| `GapPhaseKeepsMotion` | 检测窗输出随 DR 推进，不冻结（R2） |
| `TakeoverIsContinuous` / `HandbackIsContinuous` | **R2**：切换瞬间恒等 <1e-6 |
| `GapMotionPreservedAtTakeover` | 检测窗运动量不丢 |
| `PairingUsesSampleStamp` | 时间对齐：`D(t_a)` 按扫描戳插值 |
| `ObservationOnlyWhenRunning` | **R5/R1**：观测只在 kRunning 发 |
| `CloudOdomSameFrame` | 桥契约 |
| `ArbiterPriorityAndHysteresis` | **R7**：多源仲裁、滞回、翻转保护 |
| `ColdStartAnchorsPrimaryFrame` | 冷启动虚拟系=主源系 |

### bag 验收

| 用例 | 通过标准 |
|---|---|
| 纯 SLAM 一段 | 轨迹无间隔性毛刺；与 `use_dr_source:=false` 的输出逐点一致（透传契约） |
| 掐断 SLAM | 检测窗输出不冻结；切 DR 无台阶；`used_sources` 标注 |
| SLAM 恢复 | 回切无台阶；`curr_scan_in_map` 与 path/TF 重合 |
| 对比开/关观测 | 同路线两遍，接管后漂移率下降（验 R1 收益） |
| DR 段漂移 | 与"未接桥的 DR"对比不劣化（验 §1.2c/e 修复） |

---

## 12. 实施阶段

| 阶段 | 内容 | 规模 |
|---|---|---|
| **S1** | `core/`：types + source_spec + source_registry + arbiter（含连续性/滞回/翻转）+ 单测 | ~400 行 |
| **S2** | `motion_provider`（锚+增量、三相位、周期自适应、DR 插值）+ 单测 | ~250 行 |
| **S3** | `cloud_unifier` + `observation_gate` + 单测 | ~250 行 |
| **S4** | `ros/adapters` + 双前端（embedded / standalone）+ config + launch（含透传态原样转发） | ~350 行 |
| **S5** | `dr_odometry_v2` 观测接口：值改 `map_odom⊕anchor`、硬对齐不清 v、迟到观测容忍 | ~80 行 |
| **S6** | 删旧包桥、bag 回归、文档收尾 | — |

---

## 13. 开放问题（需拍板）

| # | 问题 | 建议 |
|---|---|---|
| Q1 | 检测窗推进源：DR 增量（准，需观测链关闭）vs SLAM 外推（自足，误差大） | **DR 增量**（§4.2 分析）；`fallback_to_hold` 留作退化 |
| Q2 | 超龄/判死阈值：固定配置 vs 周期 EMA 自适应 | **自适应**（1.3T / 2T），配置留上限保护 |
| Q3 | 回切误差走 glue vs 走 map_odom | **glue**（GM 自洽，§4.4） |
| Q4 | 观测迟到容忍：DR 队列放宽 vs 接受 `v·latency` | 先做后者（改动小），bag 验证后再决定前者 |
| Q5 | 是否需要 `source_status` 话题 | 建议要（可观测性，调试毛刺必备） |
| Q6 | 新包 vs 留在 `localization_v3` 改造 | **独立新包 `local_odom_gateway`**（core 库 + 内嵌前端默认 + 独立前端可选）：点云少一跳、模块化由 core 库保证、不影响 v3 现有测试 |
| Q7 | 传感器是否经本模块转发 | **不收**（R6）：IMU/CAN 由各自节点直订，本模块只管前端里程计与点云 |
