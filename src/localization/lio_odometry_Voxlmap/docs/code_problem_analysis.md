# lio_odometry_Voxlmap 执行效果问题分析报告

> 分析日期：2026-09-20
> 分析对象：`src/localization/lio_odometry_Voxlmap/src`
> 参考实现：FAST-LIVO2（`/home/hhy/2004ros/FAST-LIVO2`）、R-VoxelMap（`/home/hhy/2004ros/R-VoxelMap`）、lm_slam_ros lio（`/home/hhy/2004ros/lm_slam_ros/src/lightning-lm-ros1/src/core/lio`）
> 参考文档：`docs/fast_livo2_analysis.md`、`docs/porting_design.md`
> 状态：**仅分析，暂未修改代码**

---

## 〇、结论摘要

当前实现存在 **3 个严重 Bug（P1/P2/P3）**，集中在 ESKF 协方差取块、自适应 R 矩阵未启用、雅可比列映射错位三处；叠加 OpenMP 并行未生效（E1）与多处参数配置偏差，共同导致定位漂移/抖动、实时性不足。**建议修复顺序：P1 → P3 → E1 → P4/P5 → P2 → P6 → 参数调优。**

状态向量布局（由 `types.h` 的 `ToState()` 与 `eskf.cpp` 的 `vect_states_` 确认）：

```
pos(0-2) | rot(3-5) | offR(6-8) | vel(9-11) | bg(12-14) | ba(15-17) | offT(18-20) | grav(21-22)
```

> 注意：该布局与 FAST-LIO2 经典布局（vel 在 12-14）不同，多处代码按经典布局编写，是 P3 的根源。

---

## 一、数学理论问题

### 🔴 P1：ESKF 协方差取块错误（严重，最可能的根因）

**位置**：`src/core/laser_odometry_core.cpp` 的 `ObsModel()`、`MapIncremental()`、首帧建图三处。

```cpp
const Mat3d R_cov = kf_.GetP().block<3, 3>(0, 0);  // 注释写"姿态协方差"，实际是【位置】协方差
const Mat3d t_cov = kf_.GetP().block<3, 3>(3, 3);  // 注释写"位置协方差"，实际是【旋转】协方差
```

**问题**：两块取反了。正确应为 `block<3,3>(3,3)` 取旋转协方差、`block<3,3>(0,0)` 取位置协方差（FAST-LIVO2 原版 `cov_rot` 即取 rot 块）。

**后果链**：
1. `term2 = R·[p_i]×·R_cov·[p_i]×ᵀ·Rᵀ` 用位置协方差去传播姿态不确定性，量纲完全错误；
2. `pv.var = term1 + term2 + t_cov` 把弧度²量级的旋转协方差直接加到米²量级的位置协方差上；
3. 错误的点协方差流入 `UpdateMap` → 累积进 `plane_var_` → 污染平面拟合、3-sigma 门控、`sigma_l` 计算；
4. R-VoxelMap 的核心卖点（不确定性驱动建图）整条链路失真。

---

### 🔴 P2：自适应 R 矩阵被注释禁用

**位置**：`src/core/laser_odometry_core.cpp` 的 `ObsModel()`。

```cpp
// obs.R_ = Eigen::MatrixXd::Zero(effect_num, effect_num);  // 自适应 R 矩阵
// double adaptive_R = r_base_ + sigma_l + sigma_p;         // 全部被注释
```

**问题**：设计文档 `docs/fast_livo2_analysis.md:491` 要求 `R_i = 0.001 + σ_l + nᵀ·Var·n`，实际回退到 eskf.cpp 的标量 fallback 分支（`lidar_update_r_ = 0.001`）。

**后果**：
- 近处点/远处点、高置信平面/退化平面同等权重，无法降权退化观测；
- 标量 R=0.001 过小，过度信任观测，坏点直接打入状态估计；
- 注意：即使恢复代码，若 P1 未修，`sigma_l`/`sigma_p` 的输入仍是错的协方差，**必须先修 P1 再启用 P2**。

---

### 🔴 P3：雅可比列映射与状态布局错位（严重）

**位置**：`src/core/eskf.cpp` `Update()` 中各观测分支。

**问题 1（LiDAR 分支）**：

```cpp
h_x_cur.topLeftCorner(dof_measurement, 12) = custom_obs_model_.h_x_;
```

`ObsModel()` 生成的 `h_x_` 列布局为 `[d/dpos(0-2), d/drot(3-5), d/doffR(6-8), d/doffT(9-11)]`，直接拷贝到状态前 12 列导致：
- `d/doff_t` 落到了 **vel(9-11) 列** → 外参平移**永远不被估计**（真列 18-20 恒为 0），且速度列被污染；
- 当 `extrinsic_est_en_=true` 时直接破坏速度/偏置估计。

**问题 2（轮速分支）**：

```cpp
h_x_cur.block(0, 12, 3, 3) = custom_obs_model_.h_x_;
```

轮速观测雅可比被放到 **bg(12-14) 列**（陀螺偏置），而非 vel 列（9-11）。启用 `use_wheel_speed` 时更新的将是陀螺偏置而非速度。GPS 分支 `block(0,3)` 恰好正确（rot 在 3-5），说明轮速分支是按 FAST-LIO2 经典布局写的。

**修复方向**：`h_x_cur` 改为按实际状态布局逐块散填（offT → 列 18-20），轮速分支改为 `block(0, 9, 3, 3)`。

---

### 🟡 P4：AA（Anderson Acceleration）配置自相矛盾

**位置**：`src/core/laser_odometry_core.cpp:139`。

```cpp
// VoxelMap 模式下禁用 AA（与 FAST-LIVO2 一致）   ← 注释这么说
eskf_options.use_aa_ = true;                      // 代码却这么写
```

**附带问题**：`LoadParamsFromYAML()` 中读取的 `use_aa_` 成员变量（第 175 行）从未传给 `eskf_options`，yaml 开关 `use_aa: true` 形同虚设（死代码）。

**后果**：AA 回退条件（`eskf.cpp:140-144`：`residual_mean >= last*1.01` 即回退上一状态并 break）在 `max_iteration=3` 的低迭代上限下极易触发过早退出迭代，表现为大运动/快速转弯时跟丢。

**修复方向**：VoxelMap 模式下设 `use_aa_=false`，并把 yaml 配置真正接入。

---

### 🟡 P5：收敛准则配置与注释矛盾

**位置**：`src/core/laser_odometry_core.cpp:141`。

```cpp
eskf_options.use_pose_converge_for_lidar_ = false;  // 注释："与 FAST-LIVO2 一致使用 Pose-only"
```

**问题**：实际走全状态收敛检查（`eskf.cpp:311-323`），`bg/ba/grav/offset` 等分量的小扰动参与收敛判定，可能提前判定收敛（迭代不足），与 FAST-LIVO2 行为不一致。

**修复方向**：改为 `use_pose_converge_for_lidar_=true`。

---

### 🟡 P6：残差符号需数值验证

**位置**：`src/core/laser_odometry_core.cpp` 的 `ObsModel()`。

```cpp
obs.residual_(idx) = -(norm_vec.dot(point_world) + ptpl.d_);
```

**问题**：FAST-LIVO2 原版残差为 `norm_vec·(p_w - center)`（不带负号），H 矩阵对应块也差一个符号。残差与 H 必须符号配套；若整体取反而 `boxplus` 约定不变，IEKF 每步更新方向相反。

**验证方法**：单帧数值实验——若第 1 次迭代后残差均值**变大**，即为符号反了。

---

## 二、工程问题

### 🔴 E1：OpenMP 并行实际未生效

**位置**：`src/core/voxelmap/voxel_map_fastlivo.cpp:921` 依赖 `#ifdef MP_EN`；`CMakeLists.txt` 链接了 `OpenMP::OpenMP_CXX` 但**未定义宏、未加编译标志**。

```cmake
# 缺少：
target_compile_definitions(lio_odometry_voxelmap_core PUBLIC MP_EN)
target_compile_options(lio_odometry_voxelmap_core PRIVATE ${OpenMP_CXX_FLAGS})
```

**后果**：残差构建全程单线程（FAST-LIVO2 默认开启 `MP_EN`），单帧耗时超标 → 200Hz 主循环积压 → 位姿输出滞后、IMU 同步错位。

---

### 🔴 E2：生产级 std::cout 刷屏

`Run()` 和 `ObsModel()` 每帧多行 `[ TIME ]` 日志走终端 IO，在 200Hz 循环中是显著阻塞源。应改 `ROS_DEBUG` 或加开关控制。

---

### 🟡 E3：缓冲区无上限保护

`ProcessIMU/ProcessPointCloud` 中 `imu_buffer_`/`lidar_buffer_` 只在时间戳回退时 clear，处理不过来时无限增长 → 内存膨胀 + 延迟越滚越大（表现为"越跑越差"）。FAST-LIO 系通常有 `if (buffer.size() > 2000) clear` 类保护。

---

### 🟡 E4：首帧建图时序

首帧 `UpdateMap(pv_list_)` 使用的协方差来自**未收敛的初始 P**（且含 P1 的错误取块），初始 `plane_var_` 被污染后需多帧才能被增量更新纠正，起步阶段定位质量差。

---

## 三、参数配置问题（对照 `config/lio_odometry.yaml` 与参考实现）

| 参数 | 当前值 | 问题与建议 |
|---|---|---|
| `max_voxel_size` | 1.0 | FAST-LIVO2 室内常用 0.5；1.0 导致单 voxel 内点过多、平面拟合分辨率不足。建议 0.5 起调 |
| `max_iteration` | 3 | FAST-LIVO2 为 4；在 AA 可能早退（P4）+ 无自适应 R（P2）情况下明显不够。先修 bug，再提到 4 |
| `acc_cov` | 0.2 | 比常用值（0.1）大一倍。车载轮式平台 + MEMS IMU 需按 Allan 方差标定，而非照搬 |
| `gyr_cov` | 0.1 | 与 lm_slam_ros 对齐，但需按实际传感器验证 |
| `lidar_update_r` | 0.001 | 作为固定 R 过小 → 过度信任观测，配合 P1 的错误 P 会让坏点直接打入状态。自适应 R 修好后仅作基线 |
| `ransac_max_iter` | 10 | 对 4 点采样内点概率覆盖率很低（R-VoxelMap 默认更高），建议 ≥50，或先 `use_ransac: false` 跑通 EVD 基线 |
| `sigma_num` | 3.0 | 门控阈值本身合理，但门控输入（点协方差）被 P1 污染，当前等价于随机剔除 |
| `lru_capacity` | 100000 | R-VoxelMap 默认 500（代码 fallback 值）；10 万体素 LRU 可用于大场景但显著拖慢 `UpdateMap` |
| `layer_init_num` | 未配置 | 走默认 `{5,5,5,5,5}`，与 R-VoxelMap 推荐 `{8,7,6,5,4}` 不同，浅层平面初始化点数偏少，易拟合虚假平面 |

---

## 四、修复优先级建议

| 优先级 | 编号 | 内容 | 位置 |
|---|---|---|---|
| 1 | P1 | 协方差取块交换：`block<3,3>(0,0)/(3,3)` 三处修正 | `laser_odometry_core.cpp`（ObsModel / MapIncremental / 首帧建图） |
| 2 | P3 | `h_x_cur` 按状态布局散填（offT → 18-20），轮速分支 `block(0,9)` | `eskf.cpp` |
| 3 | E1 | CMake 加 `MP_EN` 宏 + OpenMP 编译标志，验证多线程 | `CMakeLists.txt` |
| 4 | P4/P5 | `use_aa_=false`、`use_pose_converge_for_lidar_=true`，yaml 开关真正接入 | `laser_odometry_core.cpp:139-141` |
| 5 | P2 | 恢复 `obs.R_` 对角自适应赋值（含 `sigma_l + sigma_p`），上下限保护 | `laser_odometry_core.cpp` ObsModel |
| 6 | P6 | 单帧数值实验验证残差符号与迭代方向 | `laser_odometry_core.cpp` ObsModel |
| 7 | 参数 | 先 `use_ransac:false`、`max_voxel_size:0.5` 建立基线，再按表逐项调 | `config/lio_odometry.yaml` |

**核心原则**：P1/P3 属于量纲与映射级错误，任何参数调优都无法掩盖；必须先修结构性 Bug，再谈参数。
