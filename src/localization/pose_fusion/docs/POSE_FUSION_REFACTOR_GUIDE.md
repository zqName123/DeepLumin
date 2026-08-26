# pose_fusion 改造指南

> 目标：在保留现有模块边界（`map_odom` / `odom_base` / ROS 外壳 / `localization_manager` 策略）的前提下，把当前「异步平滑融合」升级为工业上可落地的 **时间有序 + 正确初始化 +（后续）ESKF** 方案。  
> 适用范围：`DeepLumin/src/localization/pose_fusion`  
> 关联模块：`dr_odometry`、`slam_odometry`、`localization_manager`、`global_matcher` / relocalization

---

## 1. 文档目的

本文汇总对当前 `pose_fusion` 的架构判断、坐标系语义、初始化正确流程、与工业实践的差距，以及推荐改造路径，便于后续按阶段改代码，而不是边改边猜。

阅读完应能回答：

1. 当前实现在做什么、哪里合理、哪里不合理  
2. DR / SLAM 两套 odom 如何统一  
3. 拿到初始位姿后正确流程是什么  
4. 上 ESKF 时调度与状态应如何改  
5. 改造分几步、每步验收标准是什么  

---

## 2. 模块定位与现状架构

### 2.1 职责

`pose_fusion` 负责把多源局部/全局观测合成对外定位结果：

- 输入：DR odom、SLAM odom、Global Matcher、accepted relocalization、`/initialpose`、ObserverPolicy  
- 输出：`fused_odom` / `fused_pose` / `fused_path` / `fusion_status` / TF(`map→odom`)

`localization_manager` 负责监控与策略（场景、降级、是否允许 reset），**不负责**位姿滤波本身。

### 2.2 当前分层

```text
pose_fusion.launch
  → pose_fusion.yaml (rosparam)
  → pose_fusion_node.cpp          # ROS 订阅/发布/定时器
       → ros_adapter / param_loader / tf_manager
       → PoseFusionCore           # 无 ROS 内核
            ├── QualityGate
            ├── ObservationBuffer   # 仅 latest
            └── FusionStateMachine  # 诊断用状态名
```

### 2.3 核心状态分解（应保留）

\[
T_{\text{map}\leftarrow\text{base}}
=
T_{\text{map}\leftarrow\text{odom}}
\,
T_{\text{odom}\leftarrow\text{base}}
\]

| 符号 | 代码字段 | 含义 | 谁更新 |
|------|----------|------|--------|
| \(T_{\text{map}\leftarrow\text{odom}}\) | `state_.map_odom` | map 相对融合 odom 的外参 | 初值 / 重定位 / GM |
| \(T_{\text{odom}\leftarrow\text{base}}\) | `state_.odom_base` | 车在融合 odom 下的局部位姿 | DR 递推；SLAM 纠偏 |
| \(T_{\text{odom}\leftarrow\text{slam}}\) | `fusion_odom_from_slam_odom_` | 融合 odom ← SLAM odom | 首次 SLAM 锁定 |

**重要**：DR / SLAM 上游消息虽都常写 `frame_id=odom`，物理上通常是**两套不同 odom**。融合时必须先统一到「融合 odom」，不能假设话题上的 `odom` 是同一个坐标系。

### 2.4 当前数据流（问题形态）

```text
回调到达 → feed*() 内立刻 propagate / update / reset
定时器   → tick() 组装输出 + 状态机刷新
```

特点：异步、来帧即处理、无时间有序队列、无「预测到观测时刻再更新」。

---

## 3. 当前实现要点（改造时对照）

### 3.1 初始化路径（现状）

| 路径 | 触发 | 结果 |
|------|------|------|
| A. `/initialpose` | `onInitialPose` → `reset()` | `map_odom=T_init`, `odom_base=I`, `MAP_CONSTRAINED` |
| B. accepted reloc | `feedRelocalizationReset` → `reset()` | 同 A（多 policy 门控） |
| C. 首帧 DR | `allow_dr_initialization` | `map_odom=I`, `odom_base=I`, `DR_ONLY` |
| D. 首帧 SLAM | `allow_slam_initialization` | `odom_base=slam`, `map_odom=I`, `LOCAL_FUSION` |

另：`tick()` 内还有惰性 init（先试 DR 再试 SLAM）。

量产问题：C/D 与回调竞态导致「谁先到谁初始化」；A 的 `odom_base=I` 与当时 DR 绝对位姿不连续。

### 3.2 DR 更新（现状）

```text
delta = inv(last_dr) ∘ curr_dr
odom_base = odom_base ∘ delta
P 对角累加过程噪声
```

只使用相对增量 → **正确避开了 DR 绝对系原点差异**。应保留这一思想。

### 3.3 SLAM 更新（现状）

首次：

```text
T_odom_from_slam = odom_base ∘ inv(T_slam)
```

之后：

```text
observed = T_odom_from_slam ∘ T_slam
odom_base = interpolate(odom_base, observed, alpha)
```

坐标系对齐思路 **正确**；`interpolate` 固定增益 **不是**工业滤波。

### 3.4 初值 reset（现状）

```text
map_odom = T_init
odom_base = I
清空 last_dr / last_slam / T_odom_from_slam
```

能自洽（后续靠相对增量），但是工业次优写法。

---

## 4. 问题诊断（改造动机）

| 编号 | 问题 | 影响 |
|------|------|------|
| P1 | 异步来帧即处理，无 stamp 有序调度 | 无法正确做 ESKF；高速残差含时间误差 |
| P2 | DR/SLAM 冷启动竞态 | init 语义不稳定 |
| P3 | 初值 `odom_base=I` 未与当前 DR 对齐 | 与上游绝对 DR / 其它 TF 易不一致 |
| P4 | `ObservationBuffer` 只存 latest | 无法延迟回放、无法按时间消费 |
| P5 | 平滑增益替代 Kalman | 无可信协方差，难自适应 |
| P6 | `T_odom_from_slam` 只锁一次、无重锁 | SLAM 重置后静默错误 |
| P7 | frame 同名 `odom` | 门控无法区分物理系 |
| P8 | 原始 `dr_odom`/`slam_odom` 轨迹方向不同 | **预期现象**；应用对齐后轨迹验证，勿误判为融合未工作 |

工业结论：

- **坐标系分层与 SLAM 外参思路：合理，应保留**  
- **调度与估计算法：偏预研/过渡，需改造后再当主定位**

---

## 5. 坐标系与命名规范（改造时统一口径）

### 5.1 推荐命名

| 名称 | 含义 | 是否常值 |
|------|------|----------|
| `T_map_odom` | map ← 融合 odom | 慢变（初值/GM/reloc） |
| `T_odom_base` | 融合 odom ← base | 高频变（DR/滤波） |
| `T_odom_from_slam` | 融合 odom ← SLAM 的 odom | 近似常值；SLAM 重置时重锁 |
| `T_map_base` | 对外输出 = `T_map_odom ∘ T_odom_base` | 输出量 |

### 5.2 `T_odom_from_slam` 含义与公式

含义：把 SLAM 局部系下的位姿变到融合 odom。

对齐时刻 \(t_s\)：

\[
T_{\text{odom}\leftarrow\text{slam}}
=
T_{\text{odom}\leftarrow\text{base}}(t_s)
\,
\bigl(T^{\text{slam}}_{\text{odom}\leftarrow\text{base}}(t_s)\bigr)^{-1}
\]

之后：

\[
z =
T_{\text{odom}\leftarrow\text{slam}}
\,
T^{\text{slam}}_{\text{odom}\leftarrow\text{base}}
\]

\(z\) 与当前 `T_odom_base` 同属融合 odom，才能比残差 / 做 update。

### 5.3 为何不能只用一个 `T_map_odom` 统一两路原始里程计

- `T_map_odom` 只连接 **一套** 融合 odom 与 map  
- DR odom ≠ SLAM odom 时，需要额外的 `T_odom_from_slam`  
- DR/SLAM **仍在局部 odom 下递推/观测**；不是在 `T_map_base` 下递推  
- 输出时才：`T_map_base = T_map_odom ∘ T_odom_base`

### 5.4 可视化验收

不要直接叠画原始 `/localization/dr_odom` 与 `/localization/slam_odom` 比方向。应画：

- 融合 `T_map_base`  
- 对齐后的 SLAM：`T_map_odom ∘ T_odom_from_slam ∘ T_slam`  
- 或融合局部系下：`T_odom_base` vs `T_odom_from_slam ∘ T_slam`

---

## 6. 工业正确初始化方案（目标行为）

### 6.1 量产模式开关建议

```yaml
initialization:
  allow_initialpose: true
  allow_dr_initialization: false   # 仅 debug / bag
  allow_slam_initialization: false # 仅 debug
```

开滤波条件：`has_valid_init_pose && predictor_ready(DR)`。

### 6.2 状态机（建议替换/增强现有 FusionStateName 语义）

```text
UNINITIALIZED
WAIT_INITIAL_POSE          # 等待 T_init
WAIT_PREDICTOR_READY       # 已有 T_init，等待 DR
RUNNING                    # 已对齐，正常递推
SLAM_ALIGN_PENDING         # RUNNING 但尚未锁 T_odom_from_slam
MAP_CONSTRAINED            # 有新鲜 GM（可与 RUNNING 正交，作标志位）
DEGRADED / FAILURE
```

现有 `DR_ONLY` / `LOCAL_FUSION` 可保留为**能力标志**，不要再承担「冷启动竞态入口」。

### 6.3 拿到 \(T_{\text{init}}\) 后的正确公式（推荐）

设 \(T_{\text{init}}=T_{\text{map}\leftarrow\text{base}}(t_0)\)，同时刻 DR 为 \(T_{\text{dr}}(t_0)\)：

\[
\begin{aligned}
T_{\text{map}\leftarrow\text{odom}}
&= T_{\text{init}}\,T_{\text{dr}}(t_0)^{-1} \\
T_{\text{odom}\leftarrow\text{base}}
&= T_{\text{dr}}(t_0)
\end{aligned}
\]

性质：

\[
T_{\text{map}\leftarrow\text{base}}(t_0)=T_{\text{init}}
\]

且与上游 DR 绝对轨迹连续。

**对比现状**：`map_odom=T_init, odom_base=I` 为可运行简化版；改造时应改为上式。

### 6.4 完整时序（目标）

```text
[事件] 收到 /initialpose 或 accepted_relocalization
  1) QualityGate 校验（map frame、有限位姿、stamp）
  2) 缓存 T_init, t0, map_id, covariance
  3) 若无可用 DR → 进入 WAIT_PREDICTOR_READY，先不宣称对齐完成
  4) 若有 DR：
       map_odom  = T_init ∘ inv(T_dr)
       odom_base = T_dr
       last_dr   = T_dr
       清空 T_odom_from_slam、过期观测队列
       P ← 初值协方差
       initialized = true
  5) 输出第一帧 fused ≈ T_init；发布 map→odom TF

[循环] RUNNING
  A. DR: 相对增量递推 odom_base，P += Q dt
  B. 首帧合格 SLAM:
       T_odom_from_slam = odom_base ∘ inv(T_slam)   # 尽量时间对齐
  C. 后续 SLAM:
       z = T_odom_from_slam ∘ T_slam
       残差门控 → ESKF update（目标）/ 平滑（过渡）
       连续大残差 → 重锁外参或隔离 SLAM
  D. GM（可选）:
       map_odom_hat = T_gm ∘ inv(odom_base)
       更新 map_odom
  E. 输出:
       T_map_base = map_odom ∘ odom_base

[事件] 再次初值 / reloc
  → 整套重初始化（回到步骤 1），禁止在旧外参上小修
```

### 6.5 正确初值的判定标准

- 语义：\(t_0\) 时刻 \(T_{\text{map}\leftarrow\text{base}}\)  
- `frame_id = map`  
- 位置与 **yaw** 与地图/真实朝向一致  
- stamp 合理；协方差可用于 \(P(0)\)  
- 尽量低速或静止时给定  

---

## 7. 目标运行时架构（ESKF 友好）

### 7.1 角色

| 源 | 角色 |
|----|------|
| DR（或底层 IMU） | predict |
| SLAM | 局部 update（经 `T_odom_from_slam`） |
| Global Matcher | 全局 update（更新 `map_odom` 或 map 位姿观测） |
| initialpose / accepted reloc | 硬重置 |

### 7.2 调度模型（必须改）

```text
回调：仅入队（按 timestamp 有序 deque），加锁
tick / 滤波线程：
  while 有可处理事件:
    t* = 下一 DR/IMU 或 下一观测 或 下一输出时刻
    predict 状态与 P 到 t*
    if 有观测 @ t*:
      gate → update
    if 输出节拍:
      发布
```

要点：

- **不需要** ApproximateTime 把 DR 与 SLAM 绑成一对  
- **需要** 时间有序 + predict-to-time  
- SLAM 延迟：短延迟丢弃/外推；长延迟需 DR 历史 replay（二期）

### 7.3 观测模型建议

短期可继续绝对位姿观测：

\[
z = T_{\text{odom}\leftarrow\text{slam}} T_{\text{slam}},\quad
h(x)=T_{\text{odom}\leftarrow\text{base}}
\]

更稳的中期方案：相邻 SLAM 关键帧的**相对运动观测**（对 SLAM 慢漂移更鲁棒）。

### 7.4 与 `localization_manager` 的接口（保持）

- 订阅 `observer_policy`：开关与权重  
- 订阅 `accepted_relocalization`：硬重置  
- 发布 `fusion_status`：供 manager 降级判定  

改造时不要把滤波逻辑塞进 manager。

---

## 8. 分阶段改造计划

### Phase 0 — 文档与开关（无行为大改）

- [x] 本文档落地  
- [ ] YAML 增加 `operation_mode: debug|production`  
- [ ] production 下默认关闭 DR/SLAM 冷启动  

**验收**：配置可读、行为可切换。

### Phase 1 — 正确初始化（高优先级，改动面中）

目标：拿到初值后的行为符合 §6。

改动建议：

1. `reset()` / 初值路径改为等待 DR 再对齐：  
   `map_odom = T_init ∘ inv(T_dr)`, `odom_base = T_dr`  
2. 增加 `WAIT_PREDICTOR_READY`  
3. production 关闭竞态冷启动  
4. reset 后强制清空并等待重锁 `T_odom_from_slam`  

**验收**：

- 给 `/initialpose` 后第一帧 `fused ≈ T_init`  
- 对齐后原始方向不同的 DR/SLAM，在「对齐后 map 轨迹」上初值处重合  
- 无初值时 production 模式不输出 valid fused（或明确 degraded）

### Phase 2 — 时间有序队列（ESKF 前置）

改动建议：

1. `ObservationBuffer` 升级为多源 time-ordered queue  
2. `feed*` 只 `push`  
3. `tick`（或专用线程）负责 `pop` + predict/update  
4. 暂可继续用平滑 update，但必须 **先推进到观测 stamp**  

**验收**：

- 人为延迟 SLAM stamp，残差不再系统性偏大  
- 单测：乱序注入 DR/SLAM，处理顺序严格按 stamp  

### Phase 3 — ESKF 替换平滑

改动建议：

1. 新增 `PoseFusionEskf`（或在 core 内），名义状态 = `odom_base` (+v, bias 可选)  
2. DR 相对增量 / 速度模型 → predict  
3. SLAM → update（H/R/K）  
4. 输出协方差给 `fused_odom`  
5. 大残差：拒观；连续失败：重锁 `T_odom_from_slam`  

**验收**：

- 静止时协方差收敛、轨迹不漂（有 SLAM/GM 时）  
- 关掉 SLAM 后按 Q 合理发散  
- 与仅 DR 对比，融合轨迹更贴地图/GM  

### Phase 4 — 工程硬化

- frame 物理分名或文档强制：`odom_dr` / `odom_lidar`（若生态允许）  
- 外参重锁策略、SLAM 健康度接入 R  
- 延迟观测 replay（可选）  
- 与 manager 的 fail-safe 联调用例  

---

## 9. 建议的代码触点清单

| 文件 | Phase | 改什么 |
|------|-------|--------|
| `config/pose_fusion.yaml` | 0/1 | production 开关、init 策略 |
| `types.hpp` | 1/2 | 状态枚举、队列元素类型 |
| `observation_buffer.hpp` | 2 | latest → 有序队列 |
| `pose_fusion_core.hpp/cpp` | 1/2/3 | reset 公式、调度、ESKF |
| `quality_gate.*` | 1/3 | 外参重锁、时间门控 |
| `pose_fusion_node.cpp` | 2 | 回调只入队；可选独立滤波频率 |
| `math_utils.hpp` | — | 一般保留 |
| `localization_manager` | 1 | 仅确认 policy/`allow_fusion_reset` 时序，少改 |

**尽量不改**：对外话题名、`map→odom→base` 对外语义、manager↔fusion 的消息契约（除非分帧改名）。

---

## 10. 现状 vs 目标对照表

| 项目 | 现状 | 目标 |
|------|------|------|
| 初值 | `map_odom=T_init, odom_base=I` | `map_odom=T_init∘inv(T_dr), odom_base=T_dr` |
| 冷启动 | DR/SLAM 可抢 init | production 禁止 |
| SLAM 对齐 | 首次锁外参 | 保留 + 大残差重锁 |
| 调度 | 回调直接更新 | 入队 + 按时间 predict/update |
| 估计 | `interpolate` | ESKF |
| Buffer | latest | time-ordered queue |
| 输出 | `map_odom∘odom_base` | 保持 |

---

## 11. 最小正确伪代码（Phase 1+2 骨架）

```text
onInitialPose(T_init):
  pending_init = T_init
  clear slam extrinsic
  if latest_dr valid:
    commitInit(T_init, latest_dr)
  else:
    state = WAIT_PREDICTOR_READY

commitInit(T_init, T_dr):
  map_odom  = T_init ∘ inv(T_dr)
  odom_base = T_dr
  last_dr   = T_dr
  P = P0
  initialized = true
  state = RUNNING

onDr(obs):  push(dr_queue)
onSlam(obs): push(slam_queue)

tick(now):
  if state==WAIT_PREDICTOR_READY and dr_queue:
    commitInit(pending_init, pop_aligned_dr)
  while event = next_by_time(dr_queue, slam_queue, now):
    predict_to(event.t)
    if event is DR: apply_dr_delta
    if event is SLAM:
      if no extrinsic: lock_extrinsic
      else: update_with_slam
  publish(map_odom ∘ odom_base)
```

---

## 12. 测试与验收用例（改造时必跑）

1. **仅初值 + DR**：fused 从 `T_init` 出发，随 DR 相对运动平滑延伸；无 SLAM 时不应被乱拉。  
2. **初值 + DR + SLAM**：锁外参后，对齐 SLAM 与 fused 在局部系残差小；原始两路 odom 仍可方向不同。  
3. **初值时刻 DR 不在原点**：验证 `map_odom = T_init∘inv(T_dr)` 后第一帧 fused 仍等于 `T_init`。  
4. **二次 `/initialpose`**：外参与队列清空，轨迹在新初值处跳变对齐，无沿用旧 `T_odom_from_slam`。  
5. **SLAM 中断**：融合继续 DR 递推；status 显示 slam unavailable / degraded。  
6. **乱序 stamp**（Phase 2+）：处理顺序按时间，不按到达顺序。  
7. **与 manager**：`allow_fusion_reset=false` 时拒绝 reloc reset；policy 关 SLAM 时不 update。

---

## 13. 参考代码位置（当前仓库）

| 逻辑 | 位置 |
|------|------|
| 节点入口 / `/initialpose` | `src/pose_fusion_node.cpp` |
| reset / DR / SLAM / GM | `src/core/pose_fusion_core.cpp` |
| SLAM 外参锁定 | `PoseFusionCore::updateWithSlam` |
| 质量门 | `src/core/quality_gate.cpp` |
| 配置 | `config/pose_fusion.yaml` |
| Manager 策略 | `../localization_manager/` |

---

## 14. 改造原则（避免返工）

1. **先改初始化与调度，再换 ESKF 公式** —— 否则滤波仍建立在错误时间语义上。  
2. **保留 `map_odom` / `odom_base` / `T_odom_from_slam` 三件套** —— 这是当前架构最有价值的部分。  
3. **不要**用「一个 `T_map_odom` 乘两路原始 odom」当统一方案。  
4. **debug 冷启动与 production 初值路径分离** —— 用配置门控，不要删 debug 能力。  
5. **每阶段可独立合并** —— Phase 1 即可显著改善「初值后行为」；Phase 2 是 ESKF 前置条件。

---

## 15. 一句话目标态

> 初值锁定 `T_map_odom`（相对当前 DR），局部状态在融合 odom 下由 DR 预测、SLAM（经 `T_odom_from_slam`）更新；一切按时间戳有序推进；对外始终输出 `T_map_base = T_map_odom ∘ T_odom_base`。

---

*文档版本：2026-07-20*  
*基于当时 `pose_fusion` / `localization_manager` 代码与设计讨论整理。*

## 16. 当前实现更新（2026-07-20）

本轮已经完成 Phase 1 的关键工程闭环：

1. `pose_fusion.yaml` 默认进入 `production` 初始化语义，`topics.initial_pose` 改为 `/localization/manager/initial_pose`。
2. 默认关闭 DR/SLAM 冷启动，避免“谁先到谁初始化”的竞态；调试时通过 launch 参数显式打开。
3. `reset()` 不再默认令 `odom_base=I`，而是缓存 `T_map_base_init`，等待或使用当前 DR，按 `T_map_odom = T_map_base_init * inverse(T_dr)` 对齐。
4. 收到二次 manager 初值或 accepted relocalization 时，会清空 SLAM 外参并重新等待/执行 DR 对齐，避免沿用旧 `T_odom_from_slam`。
5. 新增 `WAIT_INITIAL_POSE`、`WAIT_PREDICTOR_READY`、`RUNNING` 诊断状态。
6. 状态新鲜度使用观测接收时间，DR 增量仍使用消息 stamp，降低 bag 时间与系统时间不一致导致的误降级。
7. `pose_fusion_online_test.launch` 已接入 `localization_manager`，manager 和 global_matcher 使用同一个 `pcd_path`。

当前仍保留过渡融合算法：DR 相对增量递推、SLAM 对齐后平滑更新、global_matcher 平滑更新 `map_odom`。Phase 2 的完整时间有序队列和 Phase 3 的 ESKF 还没有在本轮落地，后续应在这个初始化语义稳定后继续替换。

推荐测试命令：

```bash
source /opt/ros/noetic/setup.bash
source /home/hhy/2004ros/DeepLumin/devel/setup.bash
roslaunch pose_fusion pose_fusion_online_test.launch
```

RViz 测试流程：

1. Fixed Frame 设为 `map`。
2. 用 `2D Pose Estimate` 在地图上给初值。
3. 观察 localization_manager 终端出现 `accepted initialpose ... z_source=map_height...`。
4. 观察 pose_fusion 状态从 `WAIT_INITIAL_POSE` / `WAIT_PREDICTOR_READY` 进入 `DR_ONLY`、`LOCAL_FUSION` 或 `MAP_CONSTRAINED`。
5. 对比 `/localization/fused_path`、`/localization/compare/fused_path_map`、`/localization/global_matcher/aligned_scan` 与全局地图。

只测试 pose_fusion 冷启动调试时可打开：

```bash
roslaunch pose_fusion pose_fusion_online_test.launch start_manager:=false debug_cold_start:=true
```

该模式不会验证 manager 统一初值，只用于确认 DR/SLAM/GM 输入和输出链路。


## 17. 当前实现更新（2026-07-21）

本轮在 Phase 1 基础上继续完成了 Phase 2/3 的第一版工程落地：

1. 新增 `PoseFusionEskf`，位于 `include/pose_fusion/core/pose_fusion_eskf.hpp` 与 `src/core/pose_fusion_eskf.cpp`，保持无 ROS 依赖。
2. 局部滤波状态为 `T_odom_base + 6x6 pose covariance`，误差维度为 `[position, rotation]`。
3. DR 不再手动对角累加协方差，而是作为相对位姿增量进入 ESKF predict：
   `odom_base <- odom_base * delta_dr`，`P <- P + Q(dt)`。
4. SLAM 不再使用固定 `smooth_gain` 插值作为主融合方法；当前主路径使用相邻 SLAM 帧相对增量作为 ESKF 局部运动观测。
5. SLAM 相对观测增加双端时间对齐门控：分别使用 `last_slam.timestamp` 和 `obs.timestamp` 查询 `odom_base_history`，比较同一时间段 `[t_{k-1}, t_k]` 的 SLAM 增量与融合预测增量。超过 `buffer/max_slam_time_offset` 则拒绝或重锁。
6. GM 路径保持原设计：GM 提供 `T_map_base(t_cloud)`，通过同一时刻 `T_odom_base(t_cloud)` 反解并更新 `T_map_odom`，不直接改 DR/SLAM 内部状态。
7. 新增 SLAM ESKF 观测噪声配置：

```yaml
observers:
  slam:
    observation_std_position: 0.30
    observation_std_yaw_deg: 5.0

buffer:
  max_slam_time_offset: 0.30
```

当前仍未完全替换为统一事件队列/replay 架构：DR/SLAM/GM 仍在回调触发路径中处理，但 SLAM/GM 观测已经使用 `odom_base_history` 做按 stamp 的关键时间对齐。后续若要支持明显延迟或乱序的 SLAM/GM，应继续把 `feed*` 改为只入队，并在 `tick()` 中按 stamp 统一消费和必要时 replay。

推荐验证：

```bash
source /opt/ros/noetic/setup.bash
source /home/hhy/2004ros/DeepLumin/devel/setup.bash
roslaunch pose_fusion pose_fusion_online_test.launch
```

看 `/localization/fusion_status`：

- DR 正常预测时：`DR:eskf_predict_delta`
- SLAM 建立相对参考时：`SLAM:align_slam_relative_origin_curr_dt=...`
- SLAM 正常参与时：`SLAM:eskf_relative_update_prev_dt=..._curr_dt=...`
- SLAM 时间不对齐时：`slam_curr_odom_time_offset_too_large` 或 `slam_prev_odom_time_offset_too_large`
- GM 正常参与时：`GLOBAL_MATCHER:map_odom_smooth_update_dt=...` 或 `hard_reanchor_...`


## 18. 时间同步更新（2026-07-21）

本轮针对 SLAM 相对增量观测补齐了双端时间对齐：

```text
prev_odom = lookupOdomBaseAt(last_slam.timestamp)
curr_odom = lookupOdomBaseAt(curr_slam.timestamp)
delta_slam = inverse(last_slam.pose) * curr_slam.pose
observed_at_curr = prev_odom * delta_slam
residual = between(curr_odom, observed_at_curr)
```

由于当前尚未实现延迟观测 replay，ESKF 更新前会把 `curr_slam.timestamp` 时刻的 correction 转成当前状态上的等效观测：

```text
correction_at_curr = inverse(curr_odom) * observed_at_curr
observed_at_state_time = state.odom_base * correction_at_curr
```

这样至少避免了直接用过去时刻的 SLAM pose 去更新当前 ESKF 状态。严格工业版仍应继续实现 `feed*` 入队、`tick()` 按 stamp 统一消费，以及延迟观测 replay。
