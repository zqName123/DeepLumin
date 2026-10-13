# localization_v3 多源融合结构现状（优化基线）

> 日期：2026-10-13
> 定位：**当前代码实现**的结构总结，作为后续持续优化的基线文档。
> 关系：机制描述以本文为准（取代 `gap_fill_switch_optimization.md` 的机制部分）；
> `local_odom_gateway_design.md` 是搁置中的后续重构方向（未实施）；
> `dr_odometry_fallback_design.md` 为 DR 接入原方案（背景）。

---

## 1. 总体链路

```
【运动链 · 输出】
 slam_odometry_v2 ──/slam_odometry/odom──┐
 dr_odometry_v2  ──/dr_odometry/odom────┤   （IMU/CAN 各自节点直订 dr，不经桥）
                                        ▼
                     source_bridge_node（薄 ROS 层，进程内 core）
                     ├ LocalOdomSource（纯 C++ 仲裁：源切换/粘合/连续性）
                     ├ /localization/odom_source   100Hz 统一纯源位姿 ──► fusion / GM
                     ├ /localization/source_cloud  统一源点云        ──► GM
                     └ /localization/source_anchor 观测锚（SLAM 样本事件式）──► dr 观测

【观测链 · DR 标定】
 source_anchor ──► dr_odometry_v2（15 维 ESKF：p/v/θ/bg/ba）
                   门控：status==TRACKING && slam_probe 新鲜(≤0.15s) && 稳定(≥1.0s)
                   标定 gyro/accel 零偏、速度、姿态（绝对位姿观测）

【融合 · 全局】
 localization_v3_node：map_base = map_odom ⊕ odom_base
   GM 匹配（10Hz）→ jump guard 三档 → 写 map_odom；状态机/看门狗管理输出有效性
```

设计原则（当前实现已落实）：
1. **纯 SLAM 期 DR 结构性不进输出链**——没有任何阈值/窗口能让 DR 参与运动输出（毛刺根除）；
2. **观测链与运动链隔离**——观测锚只在 SLAM 驱动输出期按样本事件发布，DR 输出（含自身增量）
   在结构上不可能成为自己的观测（防自回灌）；
3. **切换连续性由构造保证**——`glue_new = unified(t_sw⁻) ⊕ T_new(t_sw)⁻¹`，切换瞬间输出恒等。

---

## 2. 运动链：输出语义（单一纯源输出）

| 阶段 | 判据 | 输出 | DR 增量 |
|---|---|---|---|
| ① 纯 SLAM | SLAM 源可用（odom 新鲜 [&& 注册云新鲜]） | `glue_slam ⊕ T_slam` | **不应用** |
| ② DR 接管 | SLAM 判死（age > `slam_timeout_sec`=0.25s） | `glue_dr ⊕ T_dr` | 应用 |

- 接管锚定（2026-10-13 实施，运动量不变式）：`glue_dr = T0 ⊕ D(t0)⁻¹`
  （t0/T0 = 最后 SLAM 样本的戳与位姿；D(t0) 按样本戳在 DR 历史插值，`dr_history_sec`）
  ⇒ 输出 = **T0 ⊕ (D(t0)⁻¹ ⊕ D(t))**——检测窗 t0→t_sw 的真实运动一次性补进输出
  （接管瞬间前进台阶 v×(t_sw-t0)，替代旧"轨迹短一截 + GM 事后回拉"）。
- 回切：连续 N 帧（`switch_back_confirm_frames`=5，翻转保护时 ×2）SLAM 可用才切回，
  `glue_slam = unified(t_sw⁻) ⊕ T_slam⁻¹`。
- 冷启动（虚拟系未经 LIO 确立）：`glue_slam = I` 直接定系，不换算（此时 map_odom 未建立）。
- 双源皆失：无输出（fusion 看门狗判失效）。
- 接管/回切的取舍（2026-10-13 定稿）：回切严格恒等；接管取"运动量不变式"（不丢检测窗运动）。

---

## 3. 观测链：DR 标定（dr_odometry_v2）

| 环节 | 现状（2026-10-13 实施后） | 位置 |
|---|---|---|
| 观测输入 | `/localization/source_anchor` = **map_odom ⊕ anchor = map_base（GM 校正后全局位姿）** | `dr_odometry_v2.yaml topics/localization_pose`（红线：不得直连 fused_odom） |
| 观测合成 | 桥订阅 `/localization/map_to_odom`，`z = map_odom ⊕ anchor`；**map_odom 未建立时不发观测**（红线：纯锚在虚拟系≠map_base，禁止退化——曾致首帧锚定错帧后永久拒收，§6.5） | `source_bridge_node.cpp publishAnchor` |
| 发布时机 | **SLAM 样本事件式**（每个新样本一次，仅 active==kSlam 时发） | 同上 |
| 时间戳 | **扫描戳**（2026-10-13 实施 ④）：消 v·latency 错位；迟到由 DR 侧兜底（队列 `pose_late_tolerance` 放宽位姿迟到 + observePose 一阶外推 z+=v·Δt） | 同上 + `event_queue.cpp` / `eskf.cpp observePose` |
| 观测 twist | **锚位姿差分真实速度**（2026-10-13 实施 ⓪'）：连续锚样本位置差分/时间差旋到 body 系；只差分 anchor（不含 map_odom）→ GM 修正台阶不注入假速度；dt 异常 ZOH 保持上次速度 | `source_bridge_node.cpp publishAnchor` |
| 门控 | `status==TRACKING && output_valid`（质量门）+ 时效门 max_latency + 跳变三档 | `dr_odometry_v2_node.cpp onLocalizationPose` |
| ~~探针门~~ | **已删除**（slam_probe/slam_age_limit/slam_stable_sec）：自回灌由观测构造性排除（z 无 DR 成分、DR 驱动期无观测到达），顺带消除"恢复后 1s 不观测"遗留项 | 2026-10-13 |
| 跳变三档 | 小跳(≤0.5m/2°) 直收 / 大跳(≤5m/15°) 3 帧一致→台阶修正 / 超限拒 | `pose_gate.cpp` |
| 台阶修正 | `applyPoseStep`：直接置 p/θ（**可选置 v=0**）、协方差 p/θ 块重写、零偏块保留 | `eskf.cpp:463` |
| 速度项 | `localization/use_velocity: true`（⓪' 落地：twist 为差分真实速度，速度行从噪声变信号；**若观测源退回恒零 twist 必须改回 false**） | `eskf.cpp observePose` |

---

## 4. 时间戳与坐标系契约

| 量 | 时间戳 | 说明 |
|---|---|---|
| 透传 odom / cloud | **源原始戳** | 不重打、不重采样 |
| 观测锚 | **扫描戳**（2026-10-13 起） | 晚于 DR 水位属正常（处理链延迟）：队列 `pose_late_tolerance`(0.3s) 放行 + observePose 一阶外推 z+=v·Δt 对齐到状态时刻 |
| 桥统一输出 | `now()` | 100Hz ZOH |
| 虚拟系 | `odom` frame | `unified = glue ⊕ T_source`；云与里程计必须同系（桥契约） |
| DR 世界系 | 首帧观测 `applyPoseStep` 锚定到观测（≈虚拟）系 | 未观测前为自有初始系 |

---

## 5. 关键参数（source_bridge.yaml / dr_odometry_v2.yaml）

| 参数 | 当前值 | 红线/说明 |
|---|---|---|
| `odometry/slam_timeout_sec` | 0.25 | SLAM 判死 → 切 DR；同时是断流冻结窗长 |
| `odometry/dr_timeout_sec` | 0.5 | DR 判死 → 无输出 |
| `switch_back_confirm_frames` | 5 | 回切滞回（翻转保护时 ×2） |
| `slam_require_cloud` | true | 部分故障防护：odom 活但云断 → 也切 DR |
| `localization/use_velocity` | **true**（⓪' 落地，2026-10-13） | twist=桥差分真实速度；观测源若退回恒零 twist 必须改回 false（§6.1 教训） |
| `time_sync/pose_late_tolerance` | 0.3 | 位姿观测迟到容忍（④）；须 ≥ 实际处理延迟、≤ localization/max_latency(0.5) |
| `localization/pose_noise_*` | trans 0.05m / att 0.5° / yaw 0.5° | yaw 独立 R(5,5)（新接线） |
| jump 三档 | soft 0.5m/2°，hard 5m/15°，3 帧一致 | GM 伪跳变防护 |

---

## 6. 实测证据与已知问题（2026-10-13 实测，loclog/）

### 6.0 量测语义回归（最重要，2026-10-13 补，源自 dr_log.txt 复盘）

**早期（dr_log.txt 话题统一前，接管效果好）的量测是 map 系融合位姿**：

```yaml
# dr_odometry_v2.yaml（早期）
topics:
  localization_pose: /localization_v2/odom   # 量测：融合结果（map→base，GM 修正后 = map_base）
  slam_probe: /localization/slam_odom        # 门控探针：只判"量测是否独立于 DR"，不参与滤波
```

设计定稿（dr_odometry_fallback_design.md §Q2 方案 c）：**DR 世界系 = map 系**，
量测 z = T_map_base（GM 校正后的全局位姿，零偏学习信号质量最高），
接管用 `T_odom_base = map_odom(t0)⁻¹ ⊕ T_map_base_dr(t)`（map_odom(t0) 冻结）保证 map_base 连续。

**现在量测换成了 `/localization/source_anchor`（虚拟系纯 SLAM 位姿）**——丢失了 GM 修正成分：
DR 学到的是"LIO 的说法"（自带漂移），LIO 漂移与 GM 全局修正都不进滤波。
这是双通道改造引入的回归，gateway 设计文档 §1.2c 已列为缺陷（"观测值不是绝对位姿"）。
⇒ **②③④⑥ 均是二阶项，治本是把量测改回 map_base**：
`z = map_odom ⊕ anchor`（桥订阅 /localization/map_to_odom 合成后发布；map_odom 是外部信息、
anchor 不含 DR 增量 → 仍无自回灌），或直接订融合输出 + 保留 slam_probe 门控（早期形态）。

### 6.1 DR 段质量下降（接管后持续跳动）——主因在 dr_odometry_v2 侧

**症状**：切换 DR 后轨迹明显持续跳动/摆动，不如早期版本稳。

**主因（高置信）：`localization/use_velocity: true`**——与代码默认（false）、
参数注释（"全零 twist 不得当'停车'观测"）、设计文档（§3.4 默认 false）**三方矛盾**，
该文件 2026-10-13 10:23 被改动过。开启后的后果链：

```
观测锚不带 twist（全零）→ has_velocity=true → 10Hz"零速度观测"
  → 速度态反复拽向 0，applyPoseStep 每次台阶修正还把 v 直接清零
  → 状态落后于锚 → 创新超 soft 档(0.5m) → 3 帧一致 → applyPoseStep 硬拽回
  → 零偏（尤其 gyro bias）被污染 → 接管后航向漂移率上升
  → 非完整约束(v_lat≈0)与漂移互相拉扯 → 持续摆动/跳动
```

**实测计数器对比**（dr_odometry_v2.log 心跳 `gate(rej/pend/step)`）：

| 运行 | use_velocity | pose updates | step（硬拽回） | pend（待确认） | rej |
|---|---|---|---|---|---|
| 10:13~10:19（翻转前） | false | 1637 | **9** | 216 | 65 |
| 10:24~10:29（翻转后） | true | 1288 | **123** | 406 | 0 |

台阶修正次数 9 → 123，状态被反复拽动，与症状吻合。

**处置**：`use_velocity: false`（一行配置）。预计立即改善 DR 段质量。

### 6.2 切换时刻轻微轨迹跳变——新语义的已知代价（非 dr 侧）

**机制**（桥输出本身切换瞬间恒等，跳变来自下游找回）：

```
断流后输出冻结在最后 SLAM 锚（最长 slam_timeout=0.25s）
→ 检测窗内车辆真实运动（v×0.25s，2m/s≈0.5m）不进轨迹
→ 轨迹"短一截"，但点云/odom 自洽（同虚拟系）
→ GM 匹配发现扫描真实位置靠前 → 修 map_odom 把这截找回来
→ jump guard soft 档（≤1.5m）直接放行 → 表现为切换时刻附近的轻微跳变/回拉
```

**这是删除 gap 填充（换取纯 SLAM 期零毛刺）的直接代价**，两条路二选一（见 §7-②）。

### 6.3 两轮实测复盘（2026-10-13 15:23 日志，⓪'+④ 落地后）

**⓪'+④ 验证通过**：`drop(reordered)=0` 全程（扫描戳+迟到容忍生效）；健康期 `gate(step)` 4/2（原 9~123）、
`pend` 34/7（原 216/406）——两个主要噪声源确已消除。

**新发现问题：观测"永久拒收死锁"**（两轮各出现一次）：健康跟踪 60~70s → pend 短暂累积（偏差 0.5→5m **渐涨**）
→ 1~2 次 step（applyPoseStep 置差分速度）→ 之后每条观测超 hard(5m/15°) 被**无条件拒收且无恢复路径**
→ pose_update 永久冻结 → DR 开环漂移（终态 z 漂 +57m / +193m）。触发与 GM 动荡期强相关
（fusion jump guard 放行 ≤10m 的 map_odom 修正 > dr hard 5m；round2 实测 trans=12.12m 候选）。
**主嫌 = ⓪' 差分速度污染（kFdMinDt=1ms 未限幅）被 applyPoseStep 置入状态**——已处置：
`use_velocity: false`（用户回退，rej 基本消失）+ 零偏冻结（本表）。**遗留候选**（按需）：
gate 超 hard 档恢复路径（连续 N 帧一致→台阶）、dr hard 与 fusion guard(10m) 对齐、差分速度限幅。

### 6.4 其它风险点

| # | 风险 | 影响 |
|---|---|---|
| a | `applyPoseStep` 硬对齐：置位姿 + **清零速度** + 协方差 p/θ 块重写 | 观测期反复执行（本次 123 次）是 §6.1 的放大器 |
| b | 观测锚时间戳 = 发布时刻（非扫描戳） | 创新带 v·latency 错位，助推 pend/step |
| c | `slam_age_limit(0.15) < slam_timeout(0.25)` | 门控与桥判定不一致窗口 0.1s（当前无害，语义不干净） |
| d | fusion 对局部里程计跳变零防护（设计使然） | 连续性完全靠桥 + DR 自身平滑 |
| e | GM soft 档 ≤1.5m 直接放行 | 伪跳变/找回修正都直通，见 §6.2 |

---

### 6.5 首帧观测竞态：退化纯锚错帧致永久拒收（2026-10-13 run6，已修复）

**症状**：run6（08:47 轮）锚定后 `p=0`、`rej` 以 10Hz 持续增长且 **pend=0**；run5（08:21 轮，同 bag）正常。

**根因（与零偏冻结无关，冻结按设计工作：bg/ba 平直）**：桥旧代码"map_odom 未建立时退化纯锚"
的观测在**虚拟系**，而 map_odom 建立后的观测在 **map 系**——两帧数值差一个 map→odom 偏移
（实测 ~12m）。run6 首帧观测抢跑 map_to_odom 建立（status 刚 tracking 的几十 ms 竞态窗口）
→ DR 地图基准（`map_datum_set_`，一次性）被钉在虚拟帧 → 之后所有观测恒差 ~12m 超 hard(5m)
→ **无恢复路径永久拒收**（pend=0：恒定偏置从不进中间带）。run5 首帧观测晚于 map_odom 建立，未触发。

**修复（2026-10-13）**：`publishAnchor` 加红线——`map_odom` 未建立**不发观测**（删退化分支）。
无观测期 DR 纯递推 + 零偏冻结兜底。若后续仍出现"锚定即错帧"类问题，配套候选是 gate 超 hard
恢复路径（§7 遗留）。

---

## 7. 后续优化候选（按优先级）

| # | 项 | 内容 | 预期收益 |
|---|---|---|---|
| ⓪ | **量测改回 map_base（治本）** | `z = map_odom ⊕ anchor`（桥合成）或订融合输出 + slam_probe 门控；恢复 DR 世界系=map 系语义（§6.0） | 恢复早期标定质量，接管漂移显著下降 |
| ~~⓪'~~ | ~~量测带真实速度~~ | **✅ 已实施（2026-10-13）**：桥侧连续锚样本差分真实速度（body 系）接入 twist；use_velocity=true | 速度行从噪声变信号 |
| ~~①~~ | ~~use_velocity 与假速度观测的取舍~~ | **✅ 已随 ⓪' 解决**：twist 为真实速度，use_velocity=true；applyPoseStep 速度置位不再置零噪声 | — |
| ~~②~~ | ~~检测窗运动量保留~~ | **✅ 已实施（2026-10-13，方案 a）**：接管锚定 `T0 ⊕ D(t0)⁻¹`（DR 历史按样本戳插值），检测窗运动一次性补进输出 | 消除 §6.2 跳变/回拉 |
| ③ | applyPoseStep 软化 | 不清零速度、协方差只收 p/θ 块不切断交叉项；或台阶幅度限幅 | 减轻硬拽冲击 | 
| ~~④~~ | ~~观测锚用扫描戳 + 一阶外推~~ | **✅ 已实施（2026-10-13）**：桥用扫描戳；dr 队列 `pose_late_tolerance`(0.3s) 放宽位姿迟到 + observePose 一阶外推 z+=v·Δt | 消除 v·latency 错位 |
| ⑤ | GM soft 档收紧 / 修正平滑 | 防伪跳变直通 | 轨迹更平滑 |
| ~~⑥~~ | ~~`slam_age_limit` 与 `slam_timeout` 对齐~~ | **✅ 已随探针门删除解决** | — |

---

## 8. 验收方法

- **单测**：`test_v3_local_odom_source`（18 项，切换恒等/纯 SLAM 期 DR 不参与/接管运动量不变式），
  构建在 `ros_noetic3` 容器内：`catkin_make run_tests_localization_v3`；
- **实测观测点**：
  - `dr_odometry_v2.log` 心跳 `gate(rej/pend/step)`——台阶修正次数是 DR 质量的直接指标；
  - `source_bridge.log` `switch` 事件 + 5s 心跳 `slam_age/dr_age`；
  - 判别跳变来源：`/localization/odom_source` 跳 = 源侧；仅 `map_to_odom`/融合轨迹跳 = GM/找回侧。

---

## 9. 实施进度（2026-10-13）

| 项 | 状态 | 说明 |
|---|---|---|
| ⓪ 量测改回 map_base | ✅ 已实施 | 桥合成 `z = map_odom ⊕ anchor`，事件式发布；dr 订阅不变 |
| ⓪' 量测带真实速度 | ✅ 已实施（2026-10-13 第二批） | 桥侧连续锚样本位置差分/Δt 旋到 body 系接入 twist（只差分 anchor 不含 map_odom，GM 台阶不注入假速度；dt 异常 ZOH 保持）；`use_velocity: true` |
| ① use_velocity 取舍 | ✅ 已随 ⓪' 解决 | twist 为真实速度 → `true`（速度阻尼 + 与位置观测一致）；恒零 twist 场景仍须 false（§6.1 教训保留为红线注释） |
| ② 检测窗运动量保留 | ✅ 已实施（方案 a） | 接管锚定 `T0 ⊕ D(t0)⁻¹`（DR 历史按样本戳插值）；接管瞬间前进台阶 v×(t_sw-t0)，一次性 |
| ③ applyPoseStep 软化 | ⬜ 未做 | 现状仍硬对齐（has_velocity 时置观测速度）；仅在 step 计数仍频发时做 |
| ④ 观测锚扫描戳 | ✅ 已实施（2026-10-13 第二批） | 桥用扫描戳；dr 队列 `pose_late_tolerance: 0.3s` 放宽位姿迟到（水位不回退，仅位姿类型）+ observePose 一阶外推 z+=v·Δt 对齐到状态时刻 |
| ⑤ GM soft 收紧 | ⬜ 未做 | |
| ⑥ 门控参数对齐 | ✅ 已实施（随探针门删除） | 自回灌门整体删除，仅留质量门 |
| 防自回灌判断 | ✅ 结论：不必要 | 自回灌由观测构造性排除（z 无 DR 成分 + DR 驱动期无观测）；探针/稳定窗已删 |
| 阈值红线 | ✅ 已恢复 0.25 | 0.12 落在 LIO 节拍抖动（0.1~0.185s）内 → 误接管/毛刺复发风险 |
| 零偏冻结 | ✅ 已实施（2026-10-13 第三批） | `bias_freeze_timeout: 1.0`：无被接受标定观测（定位/GNSS姿态/加速度）窗内 → bg/ba【增量】置零（数值保持冻结前），防 DR 接管/拒收期轮速残差无依据归因污染零偏；解冻=首帧被接受观测，P_bg/P_ba 回充先验；心跳新增 `bias_frozen/bg/ba` |
| 退化纯锚红线 | ✅ 已修复（2026-10-13 第四批，§6.5） | map_odom 未建立不发观测（删"退化纯锚"分支）：纯锚≠map_base（不同帧差 ~12m），曾致首帧锚定错帧后永久拒收 |

单测：`test_v3_local_odom_source` 18 项 + v3 其余全量 + dr 50 项（新增：`LatePoseWithinToleranceAccepted` /
`LatePoseBeyondToleranceDropped` / `LateImuNeverTolerated`（队列迟到容忍仅限位姿）/
`LatePoseObservationExtrapolatedByVelocity`（一阶外推消 v·lag 错位）/
`BiasFrozenKeepsValueWhenNoCalibrationSignal`（零偏数值保持冻结前，非置 0）/
`AcceptedPoseUnfreezesBiasAndRefillsCovariance`（解冻 + P 回充先验））全绿。

**待实测验证**（下次 bag/实车）：接管瞬间台阶量 = v×(t_sw-t0) 是否可接受；DR 段 `gate(step)`/`pend` 计数
是否显著下降（⓪'+④ 落地后观测质量两个主要噪声源已消）；DR 段残余抖动是否随真实速度阻尼改善；
毛刺零复发。
