# localization_v3 定位融合优化方案

> 版本：v2.0（设计评审稿，按评审意见重写）
> 日期：2026-10-10
> 基线：`localization_v2`（commit 4881ace，`src/localization/localization_v2`）
> 范围：定位融合节点（`localization_v3` 包）；`global_matcher_v2` / `relocalization` 不改动，仅经既有话题协作。

---

## 0. 目的与范围

修复 v2 分析出的缺陷，优化状态流转、人工位姿优先级、jump guard 用法与失效判定速度，并保持代码结构清晰。**先评审本方案，通过后动代码。**

优化目标：

1. 修复 v2 问题清单（§2，P1–P11）；
2. 状态流转完整；人工位姿优先级最高；给人工位姿后先匹配判断是否可行，可行则从匹配到的好位姿继续 GM；
3. jump guard 专用于**防几何相似导致的不合理跳变**，按阶段合理使用；
4. 定位失效尽快判定（替代"5s 静默 + 30s 重定位"）；
5. 逻辑与代码结构清晰，不复杂冗余化；
6. 不破坏原有定位效果（融合数学、质量门限语义、对外接口不变）。

不在范围：SLAM/GM/重定位算法内部；多地图；DR/GNSS 融合。

---

## 1. 设计原则（四条，贯穿全文）

**原则 1｜一个阶段一个机制，不叠床架屋**

| 阶段 | 定位现状 | 防护机制 | 明确不用的 |
|---|---|---|---|
| 跟踪（TRACKING） | 可靠、以当前位姿为基准 | **jump guard**（防几何相似跳变）+ 质量分级 | — |
| 人工（人工位姿后验证） | 人工位姿不精确、待核实 | **匹配质量判断**（结果本身是否可行） | 不与人工位姿比距离；验证结果不受 jump guard |
| 恢复（TRACKING_LOST / FAILURE） | 位姿基本失效，恢复**本来就是大跳变** | **质量门限 + 恢复确认窗口** | **不用 jump guard**（会挡恢复） |
| 失效判定 | — | 计数 + 看门狗 + 出图快路径 | — |

**原则 2｜人工优先级最高 = 未经验证的自动结果不得覆盖人工位姿**
人工位姿写入无条件生效（**永不经 jump guard**——人工位姿与上一帧差距大是常态）；其后的自动结果必须先过各自阶段的验证/门限才可写入。

**原则 3｜快速失效、诚实报告**
失效判定按帧计数与超时快速触发；`failure_reason` 记真实原因；数据新鲜度用真实时间戳。

**原则 4｜简单清晰**
状态不新增；模块按职责划分但数量克制；每个判定只在一个模块里做一次；配置项能省则省。

---

## 2. v2 问题清单（修复输入）

行号基于 `localization_v2` 当前源码。

| 编号 | 问题 | 位置 | 后果 |
|---|---|---|---|
| **P1** | 人工位姿后 `requestJumpGuardBypass()`，首条 GM 结果**无条件生效** | `localization_v2_node.cpp:571`、`fusion_core.cpp:52-60` | 人工位姿 ≤1 个 GM 周期（≤100ms）内被过期/错误结果覆盖 |
| **P2** | `/initialpose` 与 GM 结果的回调时序竞态：基于旧锚点算出的结果在人工位姿之后被处理 | `onInitialPose`/`onGlobalMatch` 无时效校验 | 过期结果写回旧错误位姿 |
| **P3** | 人工位姿不向 GM 发锚点（`publishGmAnchor` 仅初始化仲裁时调用） | `:743` vs `:538-579` | GM 初值与人工位姿脱节，匹配从旧位置搜起 |
| **P4** | TRACKING_LOST/FAILURE 恢复结果走 `setMapOdom` 且 v2 设想过加 jump guard | `:396-467, 641` | （方向纠偏：恢复期本就不该用 jump guard，见 §5.3） |
| **P5** | 重定位恢复把 **map->base 当 map->odom 写**（`setMapOdom(reloc_result_map_base_)`） | `:641`（正确写法 `:735`） | 恢复后位姿偏一个 `odom_base` |
| **P6** | 人工位姿无验证环节：写入后直接与 GM 结果同通道竞争 | `onInitialPose` 整体 | 与"人工优先、给值后应匹配验证"的预期不符 |
| **P7** | 失效太慢：GM 失败 50 帧（≈5s）→ LOST；再 30s/失败 50 次 → FAILURE | `localization_v2.yaml:91,95,96` | 出图约 35s 才报失效 |
| **P8** | jump guard 拒绝的结果不计数，状态机误以为正常 | `:512-529` | 持续被拒时失效永不报 |
| **P9** | `publishStatus` 新鲜度是假的（`gm_age = have_gm_candidate_ ? 0 : 1e9`） | `:982-987, 1121` | 下游感知不到断流 |
| **P10** | jump guard 是单次上限（10m/帧 @10Hz≈100m/s），大跳一帧即生效 | `localization_v2.launch:100-101` | 对几何相似误匹配只有一道粗门 |
| **P11** | GM 误匹配成功则永不报失效；FAILURE 的 `failure_reason` 恒为 "recovery timeout" | `onGlobalMatch`、`:1071` | 失效漏报 / 原因失真 |

结构性问题：`localization_v2_node.cpp` 1262 行，仲裁/分级/恢复逻辑内联回调，难测难改。

---

## 3. 总体设计

### 3.1 位姿融合不变

`map_odom = map_base ⊕ odom_base⁻¹`，`map_base = map_odom ⊕ odom_base`。`map_odom` 的写入来源与规则如下（唯一写入口 `FusionCore`）：

| 优先级 | 来源 | 生效阶段 | jump guard | 验证方式 |
|---|---|---|---|---|
| **1** | 人工位姿 `/initialpose` | 任意状态，立即生效 | **永不适用** | 无（直接写入） |
| 2 | 人工位姿后的首个合格 GM 结果（验证结果） | 人工验证期 | **不适用**（确立新基准） | 匹配质量 good |
| 3 | GM 结果（常规跟踪） | TRACKING | **适用** | 质量分级 |
| 4 | 重定位 / GM 恢复结果 | TRACKING_LOST / FAILURE | **不适用**（恢复即大跳变） | 质量门限 + 恢复确认窗口 |
| — | SLAM odom | 全程 | — | 只推进 `odom_base`，不写 `map_odom` |

### 3.2 状态机（不新增状态，保持 v2 五态）

```
UNINITIALIZED ──(slam 到达 & auto_start)──▶ RELOCALIZING ──(初始化仲裁成功 / GM 单源成功)──▶ TRACKING
      │                                        │                                            ▲   │
      └────────────── 人工位姿 ────────────────┼───────────── 人工位姿 ─────────────────────┤   │
                                              │                                            │   │ GM 失败/坏质快计数
                                       (超时保持重定位)                                    │   │ 出图快路径
                                                                     恢复确认窗口满足 ──────┘   ▼
                                                                     (reloc/GM 恢复)      TRACKING_LOST
                                                                                              │
                                                                                        lost_timeout(3s)
                                                                                              ▼
                                                                                     LOCALIZATION_FAILURE
                                                                                        （人工位姿离开）
```

状态语义与 v2 一致：`UNINITIALIZED / RELOCALIZING / TRACKING / TRACKING_LOST / LOCALIZATION_FAILURE`。**人工位姿在任何状态下给出 → 位姿立即生效、状态强制回 TRACKING（绕过驻留）**，与 v2 相同；区别只在其后 GM 结果的写入规则（§3.3）。

转移表（变化点加粗）：

| From | 事件 | To | 动作 |
|---|---|---|---|
| UNINITIALIZED | SLAM 到达 & auto_start | RELOCALIZING | reloc start |
| RELOCALIZING | 初始化仲裁成功（reloc vs GM 评分择优，保留 v2） | TRACKING | 写 map_odom、发 gm_anchor、reloc stop |
| **任意** | **`/initialpose`** | TRACKING | **写 map_odom（无门限）；发 gm_anchor；reloc stop；进入人工验证期（§3.3）** |
| TRACKING | GM 连续失败/坏质 ≥ 快阈值、出图快路径、看门狗超时 | TRACKING_LOST | reloc start |
| TRACKING_LOST | reloc/GM 连续成功 ≥ 确认窗口 | TRACKING | 采纳恢复位姿、reloc stop |
| TRACKING_LOST | **`lost_timeout_sec`（3s）** 或 reloc 连续失败 ≥ 阈值 | LOCALIZATION_FAILURE | reloc stop、报真实原因 |
| LOCALIZATION_FAILURE | `/initialpose` | TRACKING | 同人工流程 |
| LOCALIZATION_FAILURE | GM 连续高质量 ≥ 确认窗口（保留 v2 自恢复） | TRACKING | 采纳 |

驻留防抖：`TRACKING→TRACKING_LOST` 驻留由 v2 的 2.0s **降为 0.2s**（原值拖慢失效发现）；`TRACKING_LOST→TRACKING` 保持 1.0s + 确认窗口；人工触发的转移一律绕过驻留。

### 3.3 人工位姿流程（核心，无新状态、不比距离）

**结论（对应评审意见 2/3）：不新增 MANUAL_CONFIRM 状态；不做"GM 结果 vs 人工位姿"的距离比较。** 人工位姿本身不精确，匹配结果与它有差距是正常的——判断只看**匹配结果本身是否可行**（质量门限）；距离合理性由 GM 自身的 `max_delta` 门限（20m/20°，相对其匹配先验）天然限定，融合层不重复比较。

```
/initialpose 任意状态到达：
  ① FusionCore 写 map_odom（无 jump guard）→ 输出立即切到人工位姿   ← 人工优先级最高
  ② 状态 → TRACKING（绕过驻留），reloc stop
  ③ 发布 gm_anchor(人工位姿) → GM 以人工位姿为先验重新匹配          ← 修 P3
  ④ 记 manual_settle_until = now + manual_settle_sec（默认 0.3s）
       期间丢弃 GM 结果：等 GM 用新锚点算出的新鲜结果，堵回调竞态     ← 修 P2
  ⑤ settle 后首个质量 good 的 GM 结果 = 验证结果：
       → 直接采纳为当前位姿（不与人工位姿比距离、不受 jump guard）
       → 发布 gm_anchor(验证后位姿) → GM 锚定到好位姿
       → 转入常规 TRACKING（此后 jump guard 生效）                  ← 需求 2 "从当前好的位置继续 GM"
  ⑥ 未验证之前：GM 失败/坏质照常快速计数（§3.5）；
       连续失败达到快阈值或验证超时（manual_verify_timeout_sec，默认 5s）
       仍未验证 → 验证失败 → 走 TRACKING_LOST，进入恢复流程
```

**人工优先级的准确含义**（原则 2）：未经验证的自动结果不得覆盖人工位姿；人工验证期内只有"质量 good 的匹配结果"可以更新位姿（它就是验证本身）。验证失败说明人工位姿不可行（点偏/出图），此后交给恢复流程——恢复结果是全局搜索的更优估计，可以更新位姿（§3.4）。

**几何相似风险说明**：验证结果采纳前已过质量门限（inlier≥0.69 & rmse≤0.4）且为 GM 以人工位姿为先验的新鲜匹配结果；若仍误匹配，后续跟踪期由 jump guard 拦异常跳变（§3.4），不会再次无门限生效。

### 3.4 Jump guard（仅 TRACKING，防几何相似跳变）

**作用域（对应评审意见 1/4）**：

| 场景 | jump guard | 理由 |
|---|---|---|
| 人工位姿写入 | **不用** | 人工位姿与上帧差距大是常态 |
| 人工验证结果（首个 good） | **不用** | 人工位姿不精确，验证结果确立新基准 |
| TRACKING 常规 GM 修正 | **用** | 防几何相似误匹配导致的跳变（本职） |
| TRACKING_LOST / FAILURE 恢复（reloc/GM） | **不用** | 恢复本来就是大跳变，限幅会挡恢复；防误匹配交给恢复确认窗口 + 质量门限 |

**两层判定**（比较量沿用 v2 已修正的 fused_pose 变化量，P10）：

| 层 | 条件 | 判定 |
|---|---|---|
| 小跳 | `Δt ≤ soft_max_translation(1.5m)` 且 `Δyaw ≤ soft_max_yaw_deg(5°)` | 直接接受（正常修正零延迟） |
| 大跳 | 超小跳、未超 `hard_max_*`（10m/20°） | **需连续 `confirm_frames(3)` 帧候选互相一致**（帧间差 ≤0.5m/2°）才接受，否则丢弃并计数——单帧几何相似误匹配被拦下，真实持续修正 3 帧（≈0.3s）后生效 |
| 超限 | 超 hard 阈值 | 拒绝（异常跳变；真实需要更大修正时说明已失效，走恢复流程） |

被拒绝/待确认超时的结果**计入失效判定**（修 P8）。大跳确认缓冲带时效（`confirm_window_sec=1.0s`），过期清空。

### 3.5 快速失效判定（替代 5s + 30s）

| 判定 | 条件 | 时延 | 结果 |
|---|---|---|---|
| 跟踪丢失（快） | GM 连续失败/坏质 ≥ `gm_fail_fast_count(3)`（≈0.3s）或出图快路径 | **≤0.5s** | → TRACKING_LOST，reloc start，level=DEGRADED |
| 出图快路径 | GM 拒因 ∈ {cropped target empty, map chunks not ready, target map not ready} 连续 ≥ `map_miss_count(3)` | **≤0.5s** | `failure_reason="out of map"` |
| 定位失效（快） | TRACKING_LOST 内 `lost_timeout_sec(3s)` 未确认恢复，或 reloc 连续失败 ≥ `reloc_failure_threshold(10)` | **≤3.5s** | → LOCALIZATION_FAILURE，level=FAILURE，output_valid=false |
| 无修正看门狗 | 距最后被采纳的修正 > `no_correction_timeout_sec(3s)`（被 jump guard 连续拒绝 / GM 静默） | ≤0.5s 内进 LOST，再 3s 判失效 | → TRACKING_LOST（交恢复流程），未恢复则 LOCALIZATION_FAILURE（修 P8/P11） |
| 人工验证超时 | 人工位姿后 `verify_timeout_sec(5s)` 仍未通过匹配验证 | 到点即转 | → TRACKING_LOST（人工位姿不可信，交恢复） |
| SLAM 断流 | `slam_odom` 年龄 > `slam_timeout_sec(0.5s)` | 立即 | → LOCALIZATION_FAILURE |

GM 拒因分类（`FailureDetector` 内建三类）：

- `MAP_MISS`（出图/地图不可用）→ 快路径；
- `QUALITY`（inlier/rmse/fitness 不达标、jump guard 拒绝）→ 快计数；
- `TRANSIENT`（点云迟到/点数不足/等待数据）→ 轻计数，不累计长失败（防抖，保定位效果）。

恢复确认窗口保留 v2：GM 连续 3 次 / reloc 连续 2 次成功才回到 TRACKING；TRACKING_LOST 期间保持降级输出（`allow_degraded_output=true`）。

### 3.6 数据新鲜度（修 P9/P11）

`status_publisher` 按真实时间戳计算 `last_slam_age / last_global_matcher_age / last_relocalization_age / last_manual_pose_age`；`failure_reason` 记录真实原因（出图 / 断流 / 质量 / 超时）。

---

## 4. 问题 → 对策追踪表

| 问题 | 对策 | 落点 |
|---|---|---|
| P1 bypass 无条件覆盖 | 删除 `requestJumpGuardBypass`；人工后走"settle + 验证结果"流程，验证结果才可写 | §3.3 |
| P2 时序竞态 | 人工后 `manual_settle_sec` 内丢弃 GM 结果（等新锚点的新鲜结果） | §3.3 |
| P3 不发 gm_anchor | 人工位姿与验证结果都发 gm_anchor | §3.3 |
| P4 恢复期 jump guard 方向错误 | **恢复期不用 jump guard**（恢复即大跳变）；防误匹配用质量门限 + 恢复确认窗口 | §3.4 |
| P5 map_base/map_odom 混用 | 重定位恢复统一 `updateMapOdomFromMapBase`；FusionCore 四个写入口语义化 + 单测回归 | §5 |
| P6 人工无验证 | 人工验证期（settle + 首个 good 结果验证 + 验证失败转恢复） | §3.3 |
| P7 失效太慢 | 快计数 3 帧 / lost_timeout 3s / 出图快路径 / 看门狗 | §3.5 |
| P8 被拒不计数 | 拒绝与待确认计入失效判定 + 无修正看门狗 | §3.4 §3.5 |
| P9 假新鲜度 | 真实时间戳 | §3.6 |
| P10 单次上限过粗 | 小跳直过 + 大跳 3 帧一致确认 + 超限拒绝 | §3.4 |
| P11 漏报/原因失真 | 出图快路径 + 看门狗 + 真实 failure_reason | §3.5 §3.6 |

---

## 5. 代码结构

### 5.1 模块划分（core 纯 C++ 可单测，ros 薄适配，节点只编排）

```
localization_v3/
├── include/localization_v3/
│   ├── core/
│   │   ├── types.hpp                 位姿数学（语义不变）
│   │   ├── fusion_core.hpp           位姿状态 + 写入策略（唯一写入口）
│   │   ├── jump_guard.hpp            跳变防护（仅 TRACKING，两层判定）
│   │   ├── quality_gate.hpp          质量三级分级（header-only，多处共用）
│   │   ├── failure_detector.hpp      快速失效（拒因分类/看门狗/超时）
│   │   ├── state_machine.hpp         五态状态机 + 恢复确认 + reloc 命令
│   │   └── init_arbiter.hpp          初始化来源择优（header-only）
│   ├── interface/relocalization_interface.hpp
│   └── ros/
│       ├── localization_v3_config.hpp
│       ├── status_publisher.hpp      FusionStatus 组装（真实新鲜度）
│       └── relocalization_ros_adapter.hpp
├── src/{core,ros}/*.cpp
├── config/localization_v3.yaml       # 参数含义见文件内注释
├── launch/localization_v3.launch
└── test/   test_fusion_core · test_jump_guard · test_state_machine · test_failure_detector
            （CMake 目标名 test_v3_*，避免与 localization_v2 的同名测试目标冲突）
```

**职责边界**（每个判定只在一个模块做一次）：

| 模块 | 职责 | 不做 |
|---|---|---|
| `FusionCore` | `map_odom` 唯一写入口：`applyManualPose`（无门限）/ `applyVerifiedMatch`（人工验证结果，无门限）/ `applyGlobalMatch`（走 jump guard）/ `applyRelocalization`（无门限，修 P5 用 map_base 反推）；人工 settle/验证状态 | 质量评估、状态转移 |
| `JumpGuard` | TRACKING 修正的三层判定 + 大跳确认缓冲 + 拒绝计数 | 写位姿、恢复期判定 |
| `QualityGate` | inlier_ratio/inlier_rmse → good/degraded/bad | 状态转移 |
| `FailureDetector` | 失败/退化计数、无修正看门狗、拒因分类、出图快路径 | 写位姿 |
| `LocalizationStateMachine` | 五态转移、恢复确认窗口、驻留、reloc start/stop 命令 | 数值计算 |
| ROS 层 | 消息↔纯结构体、参数、发布、gm_anchor 下发 | 业务分支 |

节点只保留：回调转换 → 调 core → 发布；定时器驱动输出/状态/失效超时检查。目标 ≤400 行。

### 5.2 命名迁移

| 项 | v2 → v3 |
|---|---|
| 包名 / project / 可执行文件 | `localization_v2` → `localization_v3`（节点 `localization_v3_node`） |
| 命名空间 / 头文件目录 | `localization_v2::` / `include/localization_v2/` → `localization_v3::` / `include/localization_v3/` |
| 配置 / launch / rviz | `localization_v2.*` → `localization_v3.*` |
| 库目标 | `localization_v2_core/_ros` → `localization_v3_core/_ros` |

### 5.3 单元测试

| 测试 | 覆盖 |
|---|---|
| `test_fusion_core` | compose/inverse 数学；四写入口语义；**P5 回归**（map_base↔map_odom）；人工 settle 期间丢弃、验证结果无门限、验证后常规门限 |
| `test_jump_guard` | 小跳直过 / 大跳 3 帧确认 / 超限拒绝；缓冲不一致或超时清空；拒绝计数 |
| `test_state_machine` | 全转移表、快失效、恢复确认、驻留、人工绕过驻留 |
| `test_failure_detector` | 快计数、出图分类、瞬态不累计、看门狗 |

---

## 6. 配置参数（默认值）

> 参数含义与作用以 `config/localization_v3.yaml` 内注释为准（与实现同步维护）；下表为设计默认值摘要。

```yaml
frames: {map: map, odom: odom, base_link: base_link}

topics:                      # 与 v2 完全一致（下游零改动）
  slam_odom: /localization/slam_odom
  global_matcher_result: /localization/global_matcher/result
  initial_pose: /initialpose
  fused_odom: /localization_v2/odom
  map_to_odom: /localization_v2/map_to_odom
  status: /localization_v2/status
  loc_point: /localization_v2/loc_point
  gm_anchor: /localization_v2/gm_anchor

runtime:
  output_rate_hz: 100.0
  max_path_size: 5000
  publish_tf: true
  publish_map_to_odom_tf: true
  publish_odom_to_base_tf: false
  base_yaw_offset: -1.5707963

global_matcher:              # 质量门限语义与 v2 一致（保定位效果）
  result_pose_frame: odom
  good_min_inlier_ratio: 0.69
  degraded_min_inlier_ratio: 0.4
  max_inlier_rmse: 0.4

jump_guard:                  # 仅 TRACKING 生效
  enabled: true
  soft_max_translation: 1.5  # 小跳直接接受（m / deg）
  soft_max_yaw_deg: 5.0
  hard_max_translation: 10.0 # 大跳确认 / 超限拒绝分界（与 v2 launch 一致）
  hard_max_yaw_deg: 20.0
  confirm_frames: 3          # 大跳需连续一致帧数
  confirm_consistency_trans: 0.5
  confirm_consistency_yaw_deg: 2.0
  confirm_window_sec: 1.0

manual_pose:                 # 人工位姿流程
  z_completion_enabled: true
  z_search_radius: 5.0
  map_pcd_path: ""
  settle_sec: 0.3            # 丢弃过期 GM 结果的短窗口
  verify_timeout_sec: 5.0    # 仍未验证 → 转恢复流程
  publish_gm_anchor: true

failure_detection:           # 快速失效（替代 50 帧 / 30s）
  gm_fail_fast_count: 3      # 连续失败/坏质 → TRACKING_LOST（≈0.3s）
  degraded_count: 20         # 连续 degraded → TRACKING_LOST（保留 v2 语义）
  map_miss_count: 3          # 出图快路径
  no_correction_timeout_sec: 3.0  # 无有效修正看门狗
  lost_timeout_sec: 3.0      # TRACKING_LOST → FAILURE
  slam_timeout_sec: 0.5      # SLAM 断流 → 立即 FAILURE
  reloc_failure_threshold: 10

recovery:
  gm_recovery_threshold: 3   # 恢复确认窗口（保持 v2）
  reloc_recovery_threshold: 2
  tracking_min_dwell_sec: 0.2      # v2=2.0，避免拖慢失效发现
  tracking_lost_min_dwell_sec: 1.0

initialization:              # 与 v2 一致
  require_initial_pose: false
  use_relocalization_init: true
  use_manual_initialpose: true
  use_global_matcher_init: true

relocalization:              # 与 v2 一致
  result_topic: /localization/relocalization/result
  command_topic: /localization_v2/relocalization/command
  enabled: true
  min_inlier_ratio: 0.69
  max_inlier_rmse: 0.4
  fallback_timeout: 8.0
  result_max_age_sec: 3.0    # 初始化仲裁中重定位结果时效窗

degraded:
  allow_degraded_output: true
```

---

## 7. 兼容性（不破坏定位效果）

| 项 | v2 | v3 | 说明 |
|---|---|---|---|
| 融合数学 | `map_odom ⊕ odom_base` | 不变 | — |
| 话题/消息/TF | — | **完全不变** | 下游（规划 loc_point、健康监测、GM、reloc）零改动 |
| 质量门限语义 | inlier≥0.69 & rmse≤0.4 | 不变 | 正常接受/拒绝行为不变 |
| jump guard 比较量 | fused 变化量 | 不变 | 保留 v2 已修正逻辑 |
| 小修正 <1.5m/5° | 直接接受 | 直接接受 | 不变 |
| 中等修正 1.5~10m | 直接接受 | 需 3 帧一致（≈0.3s） | 更稳；真实漂移修正仍生效 |
| 恢复期大跳变 | 无门限立即写 | **保持无门限** | 恢复不受阻（评审意见 1） |
| 人工位姿 | 无门限写入，1 帧内可被覆盖 | 无门限写入 + settle + 验证 | 修复目标 |
| 初始化仲裁 | reloc vs GM 评分择优 | 不变 | — |
| TRACKING_LOST 降级输出 | 允许 | 允许 | — |
| 恢复确认窗口 | GM 3 / reloc 2 | 不变 | — |

验收回归：同 bag v2/v3 对照（轨迹 ATE、状态时间线、失效检出时延、人工位姿注入后位姿保持）。

---

## 8. 测试与验收

| 用例 | 通过标准 |
|---|---|
| 正常行驶 bag（v2 表现好） | ATE 不劣化（≤+5%）；TRACKING 占比 ≥ v2 |
| 人工位姿注入（含点击粗略 ±5m） | 位姿立即生效且 settle 内不被覆盖；首个 good 结果被采纳（可与点击有差距）；GM 从该位姿继续跟踪 |
| 人工位姿 + 出图/点偏 | 验证不通过 → ≤1s 转 TRACKING_LOST → ≤3.5s 报 LOCALIZATION_FAILURE |
| 出地图 bag | ≤0.5s TRACKING_LOST、≤3.5s LOCALIZATION_FAILURE，`failure_reason="out of map"` |
| GM/SLAM 断流 | 看门狗按时报失效；age 字段真实 |
| 几何相似误匹配（若有 bag） | 单帧异常大跳被拦；3 帧一致的真实修正生效 |
| 恢复场景 | TRACKING_LOST 中 reloc 大跳变**不被限制**，正常恢复 |

状态流转检查：五态每条转移边至少一次实测 + 单测。

---

## 9. 实施步骤

| 阶段 | 内容 | 产出 |
|---|---|---|
| P0 | 本文档评审 | 方案确认 |
| P1 | 命名迁移 + core：`types`/`fusion_core`（含 P5、人工 settle/验证）/`jump_guard`/`quality_gate` | core 可编译 + 单测 |
| P2 | `failure_detector`/`state_machine`（快速失效）+ 全量单测 | core 完整 |
| P3 | ROS 层：`status_publisher`/`config`/节点瘦身 + launch/config | 可运行节点 |
| P4 | rosbag 回归（v2 vs v3）+ 阈值微调 | 回归报告 |
| P5 | 文档收尾（README/机制说明） | 交付 |

改动边界：只动 `src/localization/localization_v3/**`；v2 保留作对照。

---

## 10. 决策点（已按推荐确定）

| # | 决策 | 结论 |
|---|---|---|
| D1 | 失效时延 | TRACKING_LOST ≤0.5s、FAILURE ≤3.5s（可配置） |
| D2 | 人工验证期形态 | **不新增状态**；settle 短窗口 + 验证结果流程，验证失败走常规恢复 |
| D3 | 人工位姿不精确 | **不做距离比较**；只判匹配质量，距离由 GM `max_delta` 天然限定 |
| D4 | 大跳确认帧数 | 3 帧 |
| D5 | 话题命名 | 保持 v2 话题字符串（下游零改动） |
| D6 | 失效后 GM 自恢复 | 保留（3 次确认窗口） |

风险与缓解：

- **R1 失效变快可能状态抖动** → 瞬态拒因轻计数 + 恢复确认窗口 + LOST 降级输出；阈值可调。
- **R2 大跳 3 帧确认使大修正延迟 ≈0.3s** → 仅影响 >1.5m 修正；可调 `soft_max_*`/`confirm_frames`。
- **R3 人工位姿偏远时首个 good 结果即采纳** → 已过质量门限且由人工先验匹配而来；GM `max_delta(20m/20°)` 限定偏离范围；若仍误匹配，后续由 jump guard 拦跳变。
- **R4 节点重构行为差异** → core 单测锁定 + 同 bag 对照回归。

---

## 附录 A：v2 → v3 关键改动点速查

| v2 位置 | 动作 |
|---|---|
| `fusion_core.cpp:52-60` bypass 判定、`fusion_core.hpp:55-56,100` `requestJumpGuardBypass` | **删除**；由"settle + 验证结果"取代 |
| `localization_v2_node.cpp:571` | 替换为：发 gm_anchor + 启动 settle/验证（`FusionCore` 内） |
| `localization_v2_node.cpp:641` `setMapOdom(reloc_result_map_base_)` | **改** `updateMapOdomFromMapBase(...)`（P5） |
| `localization_v2_node.cpp:396-467` 恢复期写入 | **保持无 jump guard**（恢复即大跳变）；仅质量门限 + 确认窗口 |
| `localization_v2_node.cpp:315-380` 质量分级 | 抽出 `quality_gate.hpp` |
| `localization_v2_node.cpp:982-987` 假 age | `status_publisher` 真实时间戳 |
| `localization_v2_node.cpp:512-529` 被拒静默 | 拒绝计入 `FailureDetector` |
| `initialization.cpp:26-37` TRACKING→LOST 2s 驻留 | 默认 0.2s |
| `localization_v2.yaml:91,95,96`（50/50/30s） | 替换为 `failure_detection` 参数组 |
