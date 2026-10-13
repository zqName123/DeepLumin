# lio_odometry_Voxlmap 问题分析（2026-09-29）

> 前提：本次已把 `lio_odometry`（iVox 版，纯点云效果较好）中**与地图无关的优化**全部移植到
> `lio_odometry_Voxlmap`，两包现在只在**地图表示**（iVox vs FAST-LIVO 风格 VoxelMap + 自适应 R）上不同。
> 因此本文的问题清单可以归因到地图/观测模型本身，而不再是时间同步等混杂因素。
> 合成回归 `lio_voxelmap_optimization_regression` 已通过（此前为链接失败+1 项断言失败）。

---

## 一、本次移植的非地图项（已完成，勿回退）

| 类别 | 内容 | 位置 |
|---|---|---|
| 时间同步 | `ResyncStreams`/`ResetTracking`（时间回退轻量对齐，保留滤波器与地图）；`ImuProcess::RestartStreaming`/`SetGapLimit`/`imu_gap_limit` | `laser_odometry_core.{h,cpp}`、`imu_processing.{h,cpp}`、yaml |
| 时间同步 | `SyncPackages` 弹 IMU 时**包含第一枚 `> lidar_end_time` 的样本**（否则突发到样时一枚都弹不到，跨窗间隔被算成 ~100ms 假 gap）；加 `mtx_buffer_` 锁 | `laser_odometry_core.cpp` SyncPackages |
| 时间同步 | `ProcessIMU`/`ProcessPointCloud`：非有限值校验、同戳丢弃、时间回退触发 resync、点级清洗（NaN 坐标/NaN time/负 time）、`imu_buffer_` 上限 5000 | 同上 |
| 去畸变 | 每帧重置 `scan_undistort_`，防止 IMU 空包时**复用上帧点云重复更新并重复入图**；空判顺序修正（先判空指针） | `Run()` |
| IMU 初始化 | 初始化完成时补 `last_lidar_end_time_`、`angvel_last_`、`acc_s_last_`（旧版缺失，导致初始化后首帧去畸变基准错误）；Welford 方差；非有限 IMU 拒帧 | `imu_processing.cpp` |
| GNSS/轮速 | `FeedGnssAttitude(status, nsv)`；GNSS 残差 >180° 跳过；更新前把 P 旋转块回填为传播值；1D 轮速+Huber（`use_wheel_1d`，默认 false）；全套 `[LIDAR]/[WHEEL]/[GNSS *]/[STATE]` 日志 | `laser_odometry_core.cpp`、`ros_adapter.cpp` |
| 基础设施 | syslog 日志、HealthChecker、`EIGEN_ALIGN16` 点类型、点云字段名 `timestamp`、轮速 `h_x_` 维度泛化、CMake `deeplumin_health_checker` 依赖 | `ros_adapter.*`、`types.h`、`eskf.cpp`、`CMakeLists.txt` |
| 配置对齐 | `filter_size_scan 0.5`、`use_aa false`、`acc/gyr_cov 0.1`、`b_*_cov 1e-4`、`extrinsic_est_en false`、`imu_gap_limit`、轮速/GNSS 开关与噪声与 lio_odometry 一致 | `config/lio_odometry.yaml` |

刻意**不**移植（属于地图差异或 B 侧更优）：iVox/`nearest_points_` 一族、`map_trim_*`、`kf_imu_`/`GetIMUState` 死代码、
`[TIME_LOG]` 性能日志、`use_adaptive_r`/`r_base`/pose-only 收敛选项、`keep_first` 回退后 `ChangeP(old_cov)` 并跳过建图（B 侧比 A 侧更安全，保留）。

---

## 二、本次发现并修复的 bug（含 2 个预存回归失败根因）

1. **`kf_.Update(ESKF::ObsType::LIDAR, lidar_update_r_)` 缺分号**（工作区半成品编辑）→ 包无法编译。已补。
2. **`converged_times_required` 是死参数**：`eskf.cpp` 退出条件写死 `converged_times > 0`，
   yaml 的 `converged_times_required` 从未生效（回归断言 `consecutive convergence requirement` 即查此）。
   已改为 `converged_times >= lidar_converged_times_required_ || i == maximum_iter_-1`。
3. **AA 回退判据在残差为 0 时恒触发**：`res >= last*1.01` 在 `res == last == 0` 时为真，
   会在收敛平台上误回退并提前结束迭代。已改为严格 `>`。
4. 回归测试目标未链接 `syslog`（core 引入 SYS_LOG 后测试二进制链接失败）。已在 CMake 补 `${catkin_LIBRARIES}`。

> 说明：`lio_odometry` 侧 `tests/regression.cpp` 当前也失败（`FAIL: plane through origin`），属该包自身问题，未动。

---

## 三、地图（VoxelMap）问题分析

### 3.1 文档中已指出、经本次核实**仍然存在**的问题

| # | 问题 | 现状核实 | 影响 |
|---|---|---|---|
| M1 | **混合平面**：不同结构（墙+地）进入同一根体素后被拟合成一个平面 | 仍在。回归 `pipeline(mixed=true)` 实测静止漂移：首帧 0.131 m，收敛后残余 **0.068 m**（与 `Optimization_Implementation_2026-09-20` 记录的 0.067 m 吻合） | 系统性位姿偏差，长走廊/隧道口最明显 |
| M2 | **退化方向无约束**：两平面场景沿交线方向缺约束 | 仍在。文档实测"当前格优先"策略累计 2.18 m、全邻域最高似然 0.61 m；无特征值退化诊断 | 退化场景漂移/发散 |
| M3 | **浅层平面初始化点数偏少**：`layer_init_num: [5,5,5,5,5]` | 仍在（yaml:80）。R-VoxelMap 推荐随层递减 {8,7,6,5,4} | 浅层虚假平面 → 错误约束 |
| M4 | **RANSAC 迭代不足**：`ransac_max_iter: 10`、`ransac_sample_num: 4` | 仍在（yaml:102）。4 点采样在离群点多时 10 次迭代内点覆盖率低 | 平面法向不稳 |
| M5 | **平面有效性检查只做根层**：`valid_check_max_layer: 0` | 仍在（yaml:107） | 子层平面无法剔除错误合并 |
| M6 | **无真实 bag 精度对照** | 仍是最大验证缺口（`Optimization_Implementation` 自己声明"不能声称达到或超过 R-VoxelMap"） | 无法定量评估地图替换收益 |
| M7 | 参数量级存疑：`max_voxel_size: 3.0`（根体素 3 m，室内/矿道常用 0.5~1.0）、`lru_capacity: 150000`、`sliding_thresh: 20`、`half_map_size: 50` | 仍在 | 根体素过大直接放大 M1；LRU 过大削弱静止防累积 |

### 3.2 文档中已指出、经本次核实**已解决**（文档已过时）

| 文档说法 | 实际现状 |
|---|---|
| 瓶颈 4 `UpdateMap` 全表遍历待实施 | **已实现** `update_cache`（`voxel_map_fastlivo.cpp:835-926`，只遍历本帧触达体素，日志可见 `cache_size`） |
| 瓶颈 5 `EigenSolver` 应换 `SelfAdjointEigenSolver` | **已实现**（`voxel_map_fastlivo.cpp:37`、`voxel_map_octree.h` 多处） |
| P1 协方差取块反了、P2 自适应 R 被注释 | 已按 `Optimization_Implementation` 修复（`point_covariance.h` + `use_adaptive_r`） |
| **P3 雅可比列映射错位**（d/doffT 落到 vel 列、轮速落 bg 列） | **不成立**。实测 `types.h` 状态布局为 `pos(0-2)|rot(3-5)|offR(6-8)|offT(9-11)|vel(12-14)|bg(15-17)|ba(18-20)|grav(21-22)`（`boxplus/boxminus/plus` 一致）。`ObsModel` 的 12 列 = pos/rot/offR/offT，`topLeftCorner<12,12>` 映射正确；轮速 `block(0,12,m,3)` 正是 vel 列。`code_problem_analysis.md` 的布局假设（offT 在 18-20）是错的，P3 由此误判 |
| **P6 残差符号存疑** | **不成立**。约定为 residual = `-(n·p+d)`（即 y−h，y=0），`H=+∂h/∂x`，与 ESKF `dx=K_r+(K_H−I)dx` 的更新式自洽；旋转块 `(p_i×(Rᵀn))ᵀ` 推导正确。回归的 weighted-update 用例（非对称方差）也验证了该约定 |
| E1 OpenMP 未生效 | 已有 `matching_threads` + 每线程副本 |

### 3.3 地图侧建议的修复优先级

1. **P0 缓解 M1 混合平面**（性价比最高）：`max_voxel_size` 降到 0.5~1.0；`layer_init_num` 改 {8,7,6,5,4}；
   `valid_check_max_layer` 提到 ≥1；`ransac_max_iter` ≥50。先用回归 `pipeline(mixed)` 的 0.068 m 指标做单变量消融。
2. **P1 退化诊断（M2）**：对信息矩阵 `P⁻¹+HᵀH` 的 12 维块做特征值检查，弱方向保留 IMU 先验（不叠加 H），
   并输出退化标志到 `[STATE]` 日志，便于 bag 上定位。
3. **P2 平面合并/细分与跨层关联**（M1 根治，改动大）：参考 `VoxelMap_Optimization_Design.md` 的 reinit 与
   `r_voxelmap_analysis.md` 的递归平面/有效性检查章节。
4. **P3 真实 bag A/B（M6）**：见第六节方法。

---

## 四、观测/滤波侧的问题（地图相关，保留未动）

1. **每轮 IEKF 迭代全量重建 `pv_list_`（含 P 传播）并全量重匹配**（`ObsModel`，瓶颈 1）。
   正确但昂贵：`[TIME_LOG]` 显示 ESKF 占单帧大头。可做：首迭代全量、后续迭代只更新对应点协方差/只对新位姿做增量匹配。
2. **唯一有效点门限是 `effect_num < 1`**。与原版一致，但退化/错配时整帧只有 1 个约束也照常更新；
   建议至少叠加"有效点数/匹配率"下限 + 与 `lidar_update_r` 联动放大 R，而不是整帧拒绝（整帧拒绝的旧方案已废弃）。
3. **自适应 R 的 σ_p 用传感器噪声（`var_nostate`）**，匹配门控用全协方差（含状态）。设计上自洽（避免把状态不确定性计入 R），
   但 `sigma_l`（平面方差）与 `r_base` 的量级需要 bag 标定——当前 `r_base=0.001` 偏小，坏点残差权重仍偏高。
4. **AA（`use_aa`）已对齐 lio 关闭**。注意 AA 回退分支在残差平台（残差严格不升）时不会触发（本次已改严格 `>`），
   若未来重新开启 AA，需回归验证转弯场景。

---

## 五、其它（非地图）遗留项

| # | 项 | 说明 |
|---|---|---|
| O1 | `keep_first_imu_estimation` 两包语义不同 | lio：回退 X 后**继续建图**；Voxlmap：回退 X+P 后 `return false` 不建图。默认均为 false。建议以 Voxlmap 版为准反向同步 lio |
| O2 | 1D 轮速依赖 `wheel_v_mean_`，但 `ConsumeWheelSpeedForFrame` 的调用被注释、`FeedWheelSpeed` 缓冲版也被注释 | 两包相同（照实移植）。若上车要用 `use_wheel_1d: true`，必须同时启用缓冲版 Feed + Run 中的 Consume 调用，否则 `wheel_v_mean_` 恒 0（约束 0 速度）。当前 yaml `use_wheel_1d: false`，不受影响 |
| O3 | ESKF `reject(code)` 用 `static mutex + std::cerr` 高频打印 | 建议换 SYS_LOG/节流，避免刷屏拖慢 200 Hz 循环 |
| O4 | GNSS `status<33 \|\| nsv<4` 门控仍是注释状态 | 两包一致；是否启用是策略决策 |
| O5 | `lio_odometry` 自身回归失败（`plane through origin`） | 属该包问题，与本次无关，建议单独排查 |

---

## 六、建议的 A/B 验证方法（回答"voxelmap 到底差在哪"）

1. 同一 bag、同一 yaml（仅 `voxel_map`/`ivox_*` 节不同）分别跑 `lio_odometry` 与 `lio_odometry_voxelmap`。
2. 用本次对齐的 `[STATE]` 行（`t/pos/yaw-pitch-roll/dpos/vel/effect/lidar_ok`）逐帧 diff：
   - `effect`（匹配点数）与 `lidar_ok`（更新是否被拒）→ 匹配质量差异；
   - `dpos` 与轨迹积分 → 漂移差异；
   - 静止段看 `dpos` 是否为 0（回归里分离平面 0.000 m，混合平面 0.068 m）。
3. 对 M1/M3/M4/M7 做单变量消融，每组跑同一 bag，指标：终点误差、`effect` 均值、静止段漂移、单帧耗时（`[TIME]`）。
4. 在退化段（长走廊/隧道）人工标注，对照 M2 的退化诊断输出。

---

## 附：本次改动文件清单

- `imu_processing.{h,cpp}`（整文件与 lio 对齐）、`types.h`、`lio_odometry_node.cpp`
- `laser_odometry_core.{h,cpp}`（时间同步/轮速/GNSS/日志/ResyncStreams）
- `ros_adapter.{h,cpp}`、`eskf.cpp`（轮速维度、AA 回退、收敛计数、AA 判据）
- `config/lio_odometry.yaml`（非地图参数对齐）、`CMakeLists.txt`、`package.xml`
