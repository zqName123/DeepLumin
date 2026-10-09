# TartanIMU × lio_odometry 融合设计方案

> 参考 Super Odometry 2.0「IMU 基础模型辅助的松耦合 LiDAR-Inertial 里程计」思想，
> 将 TartanIMU（IMU → 三维速度 + 协方差）引入 `lio_odometry`：
> **退化时用网络速度做观测约束；配准良好时用 SLAM 结果在线/离线微调网络。**
>
> 状态：设计稿（未改代码）
> 依据：`/home/hhy/ros2/TartanIMU` 源码实测 + `src/localization/lio_odometry` 现状分析
> 相关文档：`docs/design.md`（LIO 主设计）、`docs/porting_design.md`

---

## 目录

1. [背景与目标](#1-背景与目标)
2. [Super Odometry 2.0 思想映射](#2-super-odometry-20-思想映射)
3. [TartanIMU 能力与部署约束（代码实测）](#3-tartanimu-能力与部署约束代码实测)
4. [整体架构](#4-整体架构)
5. [数据流与线程/进程模型](#5-数据流与线程进程模型)
6. [IMU 预处理与推理窗口时间同步](#6-imu-预处理与推理窗口时间同步)
7. [网络推理方法](#7-网络推理方法)
8. [LiDAR 退化检测](#8-lidar-退化检测)
9. [ESKF 速度观测模型（融合核心）](#9-eskf-速度观测模型融合核心)
10. [融合策略与调度](#10-融合策略与调度)
11. [协方差解读与标定](#11-协方差解读与标定)
12. [SLAM → 网络的微调闭环](#12-slam--网络的微调闭环)
13. [接口与配置设计](#13-接口与配置设计)
14. [与现有缺陷的衔接](#14-与现有缺陷的衔接)
15. [分阶段实施计划与验收指标](#15-分阶段实施计划与验收指标)
16. [补充分析](#16-补充分析)
17. [风险与开放问题](#16-风险与开放问题)
18. [附录](#17-附录)

---

## 1. 背景与目标

### 1.1 现状

`lio_odometry` 是紧耦合 LiDAR-Inertial 里程计（Faster-LIO / lightning-lm 移植）：

- 23 维 ESKF：`pos / rot / ext_R / ext_t / vel / bg / ba / grav(S2)`
- 观测：点到面（iVox 局部地图）为主，可选轮速 / GNSS
- **点到面观测的雅可比只覆盖 `pos/rot/ext` 12 列，不直接观测速度**；
  `vel_` 仅靠 IMU 传播 + 与位置/姿态的协方差交叉项被间接修正

由此带来一个结构性弱点：**速度是整个滤波器里"最缺直接观测"的量**。
速度一旦漂了：

1. `imu_processing.cpp` 的运动补偿 `T_ei = pos + vel·dt + 0.5·a·dt²` 立刻出错 → 点云去畸变拉伸/重影
2. 下一帧预测初值偏 → 配准收敛变差
3. 在长隧道、巷道、开阔矿区等**几何退化**场景，点到面约束在弱方向上失效，速度/位置误差无人约束 → **建图分裂**

轮速本可以补这个洞，但轮速有打滑、换挡符号跳变、时间不同步等问题（见
`docs/` 前期分析：轮速路径存在雅可比符号错误、`wheel_v_mean_` 永不写入、
`MapIncremental` 用更新后位姿配更新前最近邻等缺陷）。

### 1.2 目标

引入 TartanIMU：**从纯 IMU 序列回归三维车体速度 + 逐轴不确定度**，作为
ESKF 的速度观测源，替代/增强轮速：

| 目标 | 说明 |
|---|---|
| **G1 退化兜底** | LiDAR 退化时，用网络速度约束 `vel_`，抑制飘飞与建图分裂 |
| **G2 全程稳速** | 非退化时也以低权重融合，改善 `vel_` 观测缺失问题，改善去畸变 |
| **G3 自适应** | 配准良好时用 SLAM 解算速度微调网络（LoRA + 输出头），适配矿车/IMU 安装 |
| **G4 不劣化** | 非退化场景精度不下降；网络失效（缺数据/异常）时自动回退纯 LIO |
| **G5 可部署** | 先用独立 Python 节点验证，后可 TorchScript/TensorRT 内嵌 C++ |

### 1.3 非目标（本阶段不做）

- 不做全局回环/建图（仍由 `slam_mapping_v2` 承担）
- 不做网络直接回归位姿（只用速度，保持松耦合、可观测、可回退）
- 不替换 IMU 传播模型（ESKF 的 `Predict` 不变）

---

## 2. Super Odometry 2.0 思想映射

Super Odometry 2.0 的核心是「**松耦合 + 基础模型 + 在线自适应**」三件套。
落到本工程的对应关系：

| Super-Odometry 2.0 要素 | 本工程落地 |
|---|---|
| IMU 基础模型回归运动量 | TartanIMU `Foundation_Model`，`car` 头，输出体坐标系速度 + log-σ |
| 松耦合：网络作为独立观测源 | 网络速度走 **独立的 `CustomObservationModel`**，不改 IMU 传播与点到面模型 |
| 退化感知的自适应加权 | `DegenerancyMonitor` 从点到面信息矩阵 + 平面法向分布算退化度，调度 `R_vel` |
| 配准质量作为监督信号 | 只在「配准良好」片段上采集 `(IMU 窗口, v_body)` 训练对 |
| 在线/离线适配平台 | LoRA（trunk 适配器）+ 输出头全量微调（`train_lora.py` 已具备） |
| 不确定度感知融合 | 用网络预测的逐轴协方差直接构造 `R_meas`（Gaussian NLL 训练保证标定性） |

**刻意保留的差异**（工程现实）：

- Super-Odometry 2.0 的完整实现含 Transformer 骨干；本工程只用已开源的
  `Foundation_Model`（ResNet-LSTM）。不影响融合架构。
- 本工程是**紧耦合**（点到面直接进 ESKF），而论文叙事偏松耦合。
  因此网络速度是"点到面之外的第二观测"，不是"替换 LiDAR 前端"。
  这反而更利于回退与 A/B。

---

## 3. TartanIMU 能力与部署约束（代码实测）

> 以下全部来自 `/home/hhy/ros2/TartanIMU` 源码与
> `tartanimu_weights/config/{unified,resnet_lstm_multihead}.yaml`，
> **不是推测**，部署必须严格对齐。

### 3.1 网络结构

```
输入 x: [B, seq_len=10, 6, 40]
   │
   ├─ ResNet 干路（每个 1s 窗口独立）
   │    input_block  : Conv1d(6→64, k7, s2) + BN + ReLU
   │    residual_groups: ResBlock×[2,2,2,2]  (64→128→256→512)
   │    resnet_post_pro: Conv1d(512→128, k1) + BN
   │    → embed [B*10, 128, 3]  →  flatten  [B, 10, 384]
   │
   └─ LSTM 干路（跨窗口时序）
        LSTM(input=384, hidden=128, layers=2, dropout=0.1, bidirectional=False)
        → out [B*10, 128]

输出头 OutputHead（按 motion_type 分头：car / dog / drone / human）
   output_block1    : FcBlock(128 → 2)   # vx, vy 均值
   output_block1_z  : FcBlock(128 → 1)   # vz 均值   (split_z=True)
   velocity_scale   : 可学习 (1,3) 缩放
   output_block2    : FcBlock(128 → 3)   # log σ 逐轴
```

- `motion_type` ID：`car=1, dog=2, drone=3, human=4` → **矿车用 `car` 头**
- 权重：`tartanimu_weights/checkpoints/unified.pt`（多头）或 `expert_car.pt`（车专家）
- 参数量 ~5.7M，官方报告 200 FPS

### 3.2 输入约定（**最容易出错的地方**）

| 项 | 值 | 来源 |
|---|---|---|
| 通道顺序 | **`(gx, gy, gz, ax, ay, az)`** —— 陀螺在前！ | `infer_trajectory.py:build_features`、`dataset_AirLab.py:161` |
| 坐标系 | 机体系（IMU 系），**重力保留在加计里，不减重力** | `use_local_coord: true`，`dataset_AirLab.py:128-130` |
| 加计单位 | **m/s²**（`GRAVITY = 9.8101`，不是 g） | `utils/constants.py:29` |
| 陀螺单位 | rad/s | 同上 |
| 采样率 | **必须 200 Hz 均匀网格** | `data.imu_freq: 200.0` |
| 归一化 | **无**（原始物理量直喂，BN 在网络内部） | dataloader 无标准化步骤 |
| 缺失帧 | 窗口内不足 200×1.0 样本则无效 | `seq_len < imu_freq*10` 直接丢弃 |

> ⚠️ 原始 npz 的 IMU 顺序是 `(ax,ay,az,gx,gy,gz)`，
> **喂网络前必须重排成 `(gx,gy,gz,ax,ay,az)`**。这是社区最常见的踩坑点。

### 3.3 输出约定

| 项 | 值 | 来源 |
|---|---|---|
| 速度 | **体坐标系** `(vx, vy, vz)`，m/s | `dataset_AirLab.py:169-175` |
| 速度标签语义 | **窗口内 1.0s 的体速度均值**（非瞬时！） | `vel_body_mean[i] = mean(velocity_body[i:i+interval])` |
| 协方差 | `pred_cov` = **log σ**；`σ = exp(pred_cov)`，`var = exp(2·pred_cov)` | `losses.py:loss_distribution_diag` |
| 协方差形式 | **对角 3×3**（逐轴独立） | `output_dim=3` |
| 返回结构 | `predict_cov=True` → `(dict[head→vel], dict[head→logσ])` | `heads.py:FoundationModel._compute_all_heads` |

NLL 损失（训练时用的就是这个，保证协方差是校准过的）：

```python
# tartan_imu/model/common/losses.py
pred_cov = clamp(pred_cov, -10, 10)
exp_term = exp(2 * pred_cov)          # = σ²
loss = (pred - targ)² / (2·exp_term) + pred_cov
```

### 3.4 推理窗口几何（**时间同步的关键**）

```
一个训练样本 = 2000 帧 @200Hz = 10.0 秒 IMU
             ┌────────┬────────┬────────┬─ ... ─┬────────┐
             │ win 0  │ win 1  │ win 2  │       │ win 9  │   ← 10 个非重叠 1s 窗口
             └────────┴────────┴────────┴─ ... ─┴────────┘
             每窗 200 帧 → 按 step_size=5 抽成 40 帧 → ResNet
输出        = 10 个速度，第 k 个对应 win k 的 1s 均值
样本滑动步长 = step_size = 5 帧 = 25 ms
LSTM 隐状态  = 每样本从 0 初始化（不是流式续接！）
```

- `window_time = 1.0`、`seq_len = 10`、`imu_freq = 200`、`sample_freq = 40`、`step_size = 5`
- `past_time = 0`、`future_time = 0` → **因果**，无未来信息
- 网络 `ResNetLSTMSeqNet.forward(x, compute_type, hn, cn)` **支持传入/返回 LSTM 隐状态**，
  但 `FoundationModel` 调用时把 `(hn2, cn2)` **丢弃**（`model_lstm` 只返回特征）。
  → 流式推理需要自己写一个薄封装。

### 3.5 微调能力（`train_lora.py` 已具备）

| 组件 | 策略 | 参数量 |
|---|---|---|
| trunk（ResNet + LSTM） | **LoRA 适配器**（rank r，target: Linear + Conv1d），基座冻结 | 小 |
| `output_block1` / `output_block1_z`（速度头） | **全量微调** | ~0.1M |
| `output_block2`（协方差头） | **全量微调** | ~0.1M |
| `velocity_scale` | 全量微调 | 3 |
| 损失 | `loss_distribution_diag`（速度 + 协方差联合 NLL） | — |

已有 `inject_lora / collect_lora_state_dict / load_lora_state_dict`，
8G 显存（RTX 3060 级）可训，**非常适合在线/近线适配**。

---

## 4. 整体架构

### 4.1 逻辑架构

```
                          ┌──────────────────────────────────────────┐
                          │              lio_odometry                 │
   /ouster/imu ──────────▶│                                          │
                          │  ┌──────────────┐    ┌────────────────┐  │
   /ouster/points ───────▶│  │ ImuBuffer    │    │ LidarBuffer    │  │
                          │  │ (native Hz + │    │                │  │
                          │  │  200Hz grid) │    └───────┬────────┘  │
                          │  └──────┬───────┘            │           │
                          │         │                    │           │
                          │         ▼                    ▼           │
   ┌──────────────────┐   │  ┌──────────────────────────────────┐   │
   │ tartanimu_node   │◀──┼──│  IMUWindowAssembler              │   │
   │  (Python / 后续  │   │  │  2000×6 @200Hz → ImuWindow       │   │
   │   TorchTRT 内嵌) │   │  └──────────────┬───────────────────┘   │
   │                  │   │                 │                        │
   │  TartanIMU 推理  │   │                 ▼                        │
   │  car 头          │   │  ┌──────────────────────────────────┐   │
   │  v_body, logσ    │   │  │ DegenerancyMonitor               │   │
   └────────┬─────────┘   │  │  HᵀH 特征值 + 法向散射 + effect#  │   │
            │             │  └──────────────┬───────────────────┘   │
            │ /imu_velocity                 │                        │
            ▼             │                 ▼                        │
   ┌──────────────────┐   │  ┌──────────────────────────────────┐   │
   │ VelocityObsBuf   │◀──┤  │ ESKF                             │   │
   │  时间对齐 + 门限  │   │  │  Predict(IMU)                    │   │
   └────────┬─────────┘   │  │  Update(LIDAR)   ← 点到面        │   │
            │             │  │  Update(VEL_NET) ← 新增          │   │
            └────────────▶│  │  Update(WHEEL)   ← 可选保留      │   │
                          │  └──────────────┬───────────────────┘   │
                          │                 │                        │
                          │                 ▼                        │
                          │        MapIncremental / 发布             │
                          └──────────────────────────────────────────┘
                                        │
                    低退化片段 (IMU窗, v_label, w) 落盘
                                        ▼
                          ┌──────────────────────────────────────────┐
                          │  adapt_worker（离线/近线）                │
                          │  train_lora.py：LoRA + 头微调 → adapter  │
                          │  回灌 tartanimu_node（热加载 / 重启）    │
                          └──────────────────────────────────────────┘
```

### 4.2 分层职责

| 层 | 位置 | 职责 | ROS 依赖 |
|---|---|---|---|
| **网络推理层** | `tartanimu_node`（Python 独立包，或后续内嵌） | IMU→速度+协方差，窗口装配，200Hz 重采样 | 是 |
| **观测适配层** | `lio_odometry/ros/ros_adapter` | 订阅 `/imu_velocity`，`FeedImuVelocity()` | 是 |
| **融合核心层** | `lio_odometry/core`（**零 ROS**） | 退化检测、`ObsModelImuVelocity`、ESKF 调度 | 否 |
| **自适应层** | `tartanimu_adapt`（Python 独立） | 样本采集、LoRA 微调、协方差标定、回灌 | 否 |

> 遵守 `docs/design.md` 的「核心算法零 ROS 依赖」原则：
> 网络只是**又一个观测源**，与轮速/GNSS 走同一个 `CustomObservationModel` 通道。

### 4.3 为什么不用"网络直接替代 IMU 传播"

- 传播必须高频（IMU 200Hz），网络推理窗口 1s、输出 1s 均值，**频率与时延都不匹配**
- 网络是黑盒，直接替代会失去可观测性与回退能力
- Super-Odometry 的精髓正是"网络作为**观测**"而非"网络作为**模型**"

---

## 5. 数据流与线程/进程模型

### 5.1 阶段 A（推荐起步）：独立 Python 节点

```
ROS 单进程 spinOnce ──┐
                      ├─▶ lio_odometry_node (C++)
tartanimu_node (Py) ──┘        │
   │  订阅 /ouster/imu          │ 订阅 /imu_velocity
   │  200Hz 重采样              │ FeedImuVelocity(t, v, logσ)
   │  每 T_stride 推理一次      │
   └─ 发布 /imu_velocity ──────┘
```

- **进程隔离**：网络崩溃/OOM 不影响里程计；里程计可独立回放
- **延迟**：ROS pub/sub 1–3 ms + 推理 5–15 ms ≈ **< 20 ms**，远小于 1s 窗口时延，可忽略
- **A/B**：`use_imu_velocity: false` 一键回退

### 5.2 阶段 B（量产）：TorchScript / ONNX → TensorRT 内嵌

```
lio_odometry_node
   └─ ImuVelocityEstimator (C++, 可选编译)
        ├─ libtorch / TensorRT  (unified.pt → model.ts → model.engine)
        ├─ IMU 环形缓冲 (200Hz 网格)
        └─ 后台 std::thread 推理，结果写 VelocityObsBuf
```

- 接口与阶段 A **完全一致**（同一个 `FeedImuVelocity`），ESKF 代码不动
- 参考工程内已有 CUDA/TensorRT 可选项（`deeplumin_cmake`）

### 5.3 线程/锁约定（**吸取轮速教训**）

| 变量 | 生产者 | 消费者 | 保护 |
|---|---|---|---|
| `imu_buffer_` | IMU 回调 | `SyncPackages` | 已有 `mtx_buffer_` |
| `imu_window_grid` | IMU 回调 | 网络节点/推理线程 | 独立 `std::mutex` 或 SPSC 环形队列 |
| `vel_obs_buf_` | `FeedImuVelocity` | `Run()` | **必须加锁**（轮速的 `wheel_speed_obs_` 曾裸写） |
| `degeneracy_score` | `Run()` | 发布/自适应采集 | 同 `mtx_buffer_` 或原子量 |

> 轮速路径的教训：`have_new_wheel_speed_` / `wheel_speed_obs_` 无同步、无时间戳、
> 末值锁存。**新观测源必须从第一天就带时间戳 + 队列 + 年龄门**。

---

## 6. IMU 预处理与推理窗口时间同步

这是本方案**最需要写清楚**的部分。

### 6.1 时间轴

```
ROS 时间 (s)
 ─────────────────────────────────────────────────────────────────▶
                 │◀────── 1.0 s ──────▶│
                 t_w0                  t_w1
   imu ... ──────┬──────────────────────┬────── ...
                 │   window k (200 样本) │
                 │                      │
                 │        标签时刻 t_label = t_w0 + 0.5 s   (窗口中心)
                 │        （标签 = 窗口内体速度均值）
                 │
                 └── 网络在 t_w1 之后才能算出 win k（需完整 1s 上下文）

 ESKF 状态时刻 t_state = lidar_end_time_ （扫描末端，通常 ≈ 当前 - 30~80 ms）
```

**三条时间必须区分清楚：**

| 记号 | 含义 | 典型值 |
|---|---|---|
| `t_imu` | IMU 样本戳 | 200 Hz |
| `t_label` | 网络输出的**有效时刻** = 窗口中心 `t_w0 + 0.5·T_win` | 落后当前 0.5 s |
| `t_state` | ESKF 时刻 = `lidar_end_time_` | 落后当前 30~80 ms |

### 6.2 原生 IMU → 200 Hz 均匀网格

Ouster 内置 IMU 常见 **100 Hz**，而网络训练在 **200 Hz**。
直接把 100Hz 数据当 200Hz 喂 → 时长只有一半，网络完全失配。

**做法：窗口装配时做时间轴重采样，而不是启动时一次性转换。**

```python
def assemble_window(imu_buf, t_start, T_win=1.0, fs=200):
    """
    imu_buf: [(t, gx,gy,gz, ax,ay,az), ...]   # 原生频率，已重排通道
    返回: [200, 6] float32, 时间网格 t_start + i/fs
    """
    n = int(round(T_win * fs))                       # 200
    t_grid = t_start + np.arange(n) / fs
    # 仅用 [t_start - pad, t_start + T_win + pad] 内的样本做插值
    seg = select_range(imu_buf, t_grid[0] - 0.02, t_grid[-1] + 0.02)
    out = np.empty((n, 6), np.float32)
    for c in range(6):
        out[:, c] = np.interp(t_grid, seg.t, seg[:, c])   # 或 PCHIP 样条
    return out
```

要点：

1. **逐通道线性/ PCHIP 插值**到均匀 200 Hz 网格（陀螺/加计都插值；不做姿态积分补偿）
2. 插值跨度设 `pad`（20 ms）防止边界外推
3. **样本不足判据**：`t_grid` 覆盖范围内原始样本数 < `0.5·n` 或最大间隔 > `3/fs` → 本窗标 `invalid`
4. 时间戳必须是**驱动层采样时刻**（`msg->header.stamp`），不是回调到达时刻；
   若 `header.stamp` 为 0（如 `CanReceiveInfo` 那种兜底），需与驱动约定统一
5. **禁止**用 `ros::Time::now()` 补戳（轮速路径就是这么做的，会引入调度抖动）

### 6.3 10 窗口上下文装配（对齐训练几何）

严格复刻 `ResNetLSTMSeqToSeqDataset.__getitem__` / `infer_trajectory.make_seq_feat`：

```python
SEQ_LEN      = 10      # train.seq_len
WINDOW_TIME  = 1.0     # model_param.window_time
IMU_FREQ     = 200     # data.imu_freq
STEP_SIZE    = 5       # imu_freq / sample_freq  (下采样因子)
WINDOW_SIZE  = int(WINDOW_TIME * IMU_FREQ)          # 200
CONTEXT      = SEQ_LEN * WINDOW_SIZE                # 2000 帧 = 10 s

def make_seq_feat(feats_200hz, start):
    """feats: [N,6] @200Hz, start: 起始帧号
       返回 [SEQ_LEN, 6, WINDOW_SIZE//STEP_SIZE] = [10, 6, 40]"""
    out = np.zeros((SEQ_LEN, 6, WINDOW_SIZE // STEP_SIZE), np.float32)
    for i in range(SEQ_LEN):
        w = feats[start + i*WINDOW_SIZE : start + (i+1)*WINDOW_SIZE, :]  # [200,6]
        out[i] = w[::STEP_SIZE, :].T                                      # [6,40]
    return out
```

**几何不变量（改代码时必须保持）：**

| 量 | 值 | 变了会怎样 |
|---|---|---|
| `window_time` | 1.0 s | 改变 → `window_size` 变 → `resnet_code` 变 → **权重作废** |
| `imu_freq` | 200 Hz | 同上 |
| `sample_freq` / `step_size` | 40 / 5 | 同上（`actual_win_size = 200/5 = 40`，`resnet_code = 128×3 = 384`） |
| `seq_len` | 10 | LSTM 时序长度；可小于 10 但需重训/验证 |
| 通道序 | `(g, a)` | 静默出错，精度崩塌 |
| 重力 | 保留 | 静默出错 |

> **`window_time` 与 `imu_freq` 共同决定 ResNet 输入长度，进而决定 LSTM 输入维度，
> 是"冻结结构超参"。** 只能改 `seq_len` 和标签定义，不能改这两个。

### 6.4 输出 → ESKF 时刻的对齐（**核心难点**）

网络输出的是 **窗口中心时刻 `t_label = t_w0 + 0.5 s` 的 1s 均值体速度**。
ESKF 需要的是 **`t_state = lidar_end_time_` 的瞬时体速度**。
两者相差 `Δt = t_state - t_label`，在加减速时速度差 ≈ `a·Δt`。

**三级方案（递进）：**

#### 方案 1（零重训，立即可用）：窗口均值 + IMU 递推补偿

```
v_net(t_label)  ──(用 ESKF/IMU 的加速度递推 Δt)──▶  v̂(t_state)
```

```cpp
// 伪代码：在 FeedImuVelocity 或 ObsModel 中做
double dt = t_state - obs.t_label;
if (std::fabs(dt) > max_align_gap) → 丢弃本观测
Vec3d a_world = state.rot_ * (imu_acc_mean - state.ba_) + state.grav_.vec_;
Vec3d v_comp  = obs.v_body_predicted_forward(dt, a_world);
// 等价地：把残差写成 z - R^T (v + a·dt) 的形式，在雅可比里加 ∂/∂a 项或直接补偿
```

- 优点：不动网络
- 缺点：`v_net` 本身是 1s 均值，**对急加减速天然滞后**；
  只能补偿"均值时刻"到"当前"的漂移，不能还原被平均掉的动态
- **适用**：巷道匀速、退化兜底（退化场景往往低速/匀速）

#### 方案 2（**推荐**，小规模 LoRA 重训）：改标签定义

保持 `window_time=1.0 / imu_freq=200 / step_size=5` **结构不变**，只改监督信号：

```
旧标签: mean(v_body[t : t+1.0s])                     ← 1s 均值
新标签: mean(v_body[t+0.8s : t+1.0s])                 ← 末 0.2s 均值
   或   v_body[t+1.0s]                               ← 窗口末端瞬时
```

- 网络仍看 1s 上下文（信息量足），但**回归的是末端速度**
- 输出时刻变成 `t_label = t_w1 = t_w0 + 1.0s`（窗口末端），
  与 `t_state`（扫描末端）只差几十毫秒 → **对齐误差从 ~0.5s 降到 ~50ms**
- 需要 `train_lora.py` 跑一轮自有数据（矿车数据），LoRA + 头微调
- **注意**：这会把网络从"1s 平滑速度"变成"更锐利的速度"，
  协方差头必须一起重训（NLL 联合训练已支持）

#### 方案 3（长期）：变长窗口 / 多尺度

- 头部加一个"短窗口分支"（200ms）专门出瞬时速度，1s 分支出平滑速度
- 超出本阶段范围，仅作路线图

**推荐落地顺序：方案 1 上线验证 → 方案 2 数据够了就切。**

### 6.5 采样节拍

| 项 | 值 | 理由 |
|---|---|---|
| 网络推理节拍 `T_stride` | **100 ms（10 Hz）** | 与 LiDAR 帧率一致；窗口滑动 100ms，一次推理 ~10ms，余量足 |
| 训练时的滑动步长 | 25 ms | 可作为上限；25ms 推理成本 4×，Phase 2 再考虑 |
| 上下文长度 | 10 s 固定 | 与训练一致（见 6.3） |
| 冷启动 | **前 10 s 输出 `valid=false`** | 训练样本必为完整 10s 且 LSTM 从零态起 |

> 冷启动 10s 是诚实的代价：训练几何就是 10s 上下文 + 零初始隐状态。
> 矿车开机后连续作业数小时，10s 可接受；这期间回退纯 LIO / 轮速。
> **不建议**用"重复首帧填充"之类的 OOD 技巧强行提前出数。

### 6.6 异常处理

| 情况 | 处理 |
|---|---|
| IMU 断流 > `imu_gap_limit`(0.15s) | 本窗 `invalid`；`ResyncStreams()` 时同步清空窗口缓冲 |
| 时间戳回退（PTP 重同步） | 清窗口缓冲 + 重置 LSTM 上下文，`valid=false` 持续 10s |
| 200Hz 网格插值跨度内样本过稀 | `invalid` |
| `‖v_net‖ > max_v_norm`(30 m/s) | `invalid`（网络输出爆炸的保护） |
| `logσ` 超界（\|logσ\|>10） | 钳位到 [-10,10]（与训练一致） |
| 推理线程超时（> 200 ms） | 丢弃本次结果，不阻塞 `Run()` |

---

## 7. 网络推理方法

### 7.1 推理封装（阶段 A，Python）

```python
class TartanIMUEstimator:
    """对齐 dataset_AirLab / infer_trajectory 的推理封装。"""

    def __init__(self, cfg_path, ckpt_path, motion_type="car", device="cuda"):
        self.cfg   = load_config(cfg_path)
        self.model = build_model(DummyArgs(), self.cfg)
        ckpt = torch.load(ckpt_path, map_location=device)
        self.model.load_state_dict(ckpt.get("model_state_dict", ckpt))
        self.model.to(device).eval()
        self.head = motion_type            # "car"
        self.mtid = {"car":1,"dog":2,"drone":3,"human":4}[motion_type]

    @torch.no_grad()
    def infer_window(self, seq_feat: np.ndarray):
        """
        seq_feat: [10, 6, 40]  float32, 通道序 (g,a), 200Hz 下采样后
        返回 (v_body[3], log_sigma[3])
        """
        x = torch.from_numpy(seq_feat).unsqueeze(0).to(self.device)  # [1,10,6,40]
        label = torch.tensor([self.mtid], device=self.device)
        vel, cov = self.model(x, motion_type=label,
                              predict_cov=True, compute_all_heads=False)
        v   = vel[self.head][0, -1].cpu().numpy()    # 取最后一个窗口 = 最新 1s
        lsg = cov[self.head][0, -1].cpu().numpy()
        return v.astype(np.float64), lsg.astype(np.float64)
```

要点：

1. `compute_all_heads=False` + `motion_type` → 只跑 `car` 头，省算力
2. 取 `[-1]`（最后一个窗口）= 最新 1s 上下文，对应 `t_label = t_w0+0.5s`
   （方案 2 切换后改为"末端标签"，仍取 `[-1]`）
3. `predict_cov=True` 一定要开 —— **协方差是这个方案相对轮速的最大优势**
4. `torch.no_grad()` + `eval()`（关 dropout，BN 用运行统计量）
5. 若用 LoRA adapter：先 `inject_lora(model)` 再 `load_lora_state_dict`，见 `train_lora.py`

### 7.2 推理封装（阶段 B，C++）

```
导出链:
  python export_ts.py --ckpt unified.pt --out model.ts        # torch.jit.trace/script
  trtexec --onnx=model.onnx --saveEngine=model.engine          # 可选 TensorRT

C++ 接口:
  struct ImuWindow { double t_label; float data[10][6][40]; };
  struct VelObs    { double t_label; Vec3d v_body; Vec3d log_sigma; bool valid; };
  class ImuVelocityEstimator {
    bool Infer(const ImuWindow&, VelObs& out);
  };
```

- `torch.jit.trace` 对本模型足够（无动态控制流）
- 注意 `FoundationModel.forward` 返回 **dict**，导出前要包一层 `nn.Module`
  只返回 `(vel_car, logσ_car)`，否则 trace 会失败
- TensorRT 精度：FP16 即可（速度输出量级 ~10 m/s，无下溢风险）；
  **协方差头建议 FP32**（`exp(2·logσ)` 对精度敏感）

### 7.3 算力预算

| 平台 | 单次推理（10 窗口） | 10 Hz 占用 | 备注 |
|---|---|---|---|
| RTX 3060 / Orin | ~3–8 ms | 3–8 % | 阶段 A/B 均可行 |
| Jetson Orin NX | ~8–15 ms | 8–15 % | 建议 FP16 TensorRT |
| 无 GPU（CPU i7） | ~60–120 ms | 勉强 | 需降到 5 Hz 或只在退化时推理 |

**无 GPU 时的退化触发式推理**：`DegenerancyMonitor` 从纯 LiDAR 免费算出退化度，
只在退化时才请求网络推理 → 算力从"常开"变"按需"。作为 Phase 1.5 的优化项。

---

## 8. LiDAR 退化检测

### 8.1 信号来源（**几乎零成本，数据已在手**）

| 信号 | 现有代码位置 | 含义 |
|---|---|---|
| `effect_feat_num_` | `ObsModel` 统计 | 有效点面配对数 |
| `obs.lidar_residual_mean_ / max_` | `ObsModel` 末尾 | 残差平方中位数 / 最大 |
| `obs.h_x_` | `ObsModel` 输出 | 点到面雅可比 (N×12) |
| 平面法向 `corr_norm_` | `esti_plane` | 匹配平面的法向分布 |

### 8.2 主指标：位姿信息矩阵特征值

```
Λ_pose = H_poseᵀ · H_pose           # H_pose = h_x_ 前 6 列 (pos 3 + rot 3)
```

**单位不一致问题**：`pos` 列是 m 量纲，`rot` 列是 rad 量纲，直接合 6×6 求特征值无意义。
两种处理：

**做法 A（推荐）：分别做 3×3**

```
Λ_t = H_posᵀ H_pos   (3×3)  →  λ_t1 ≤ λ_t2 ≤ λ_t3
Λ_r = H_rotᵀ H_rot   (3×3)  →  λ_r1 ≤ λ_r2 ≤ λ_r3
退化方向 = 对应小特征值的特征向量
```

**做法 B：特征长度缩放后合 6×6**

```
L = RMS(‖p_i‖)   # 匹配点到传感器的均方根距离，典型 5~20 m
Λ_scaled = [H_pos, L·H_rot]ᵀ [H_pos, L·H_rot]   → 6×6
```

### 8.3 辅助指标：平面法向散射

```
S = Σ_i n_i n_iᵀ          (3×3, n_i = corr_norm_[i].head<3>())
特征值 μ1 ≤ μ2 ≤ μ3
μ3/μ1 > 50 且 μ2/μ1 > 20  →  法向近似共面/共线 → 强退化（长走廊/巷道）
```

这是**最直接对应矿井巷道**的退化判据：巷道内所有墙面法向都垂直于巷道轴，
沿轴方向点到面零信息。

### 8.4 综合退化度

```cpp
struct DegeneracyInfo {
  double score;          // 0 = 完全约束, 1 = 强退化  (标量，用于调度 R)
  Vec3d  dir_weak;       // 最弱方向（世界系），用于方向选择性加权
  double lambda_min_t;   // 平移最小特征值
  double lambda_min_r;   // 旋转最小特征值
  double normal_aniso;   // 法向各向异性 μ3/μ1
  int    effect_num;
  double resid_median;
  bool   valid;
};
```

判据（可调，需在实车数据上标定阈值）：

```
score = max(
    step(τ_t,  λ_min_t),        # 平移信息不足
    step(τ_r,  λ_min_r),        # 旋转信息不足
    w_n · saturate(log(normal_aniso) / log(aniso_max)),
    step(τ_num, τ_cnt - effect_num)
)
```

**标定方法**：在已知良好场景（有柱子、有设备的硐室）录 2 min，
统计 `λ_min` 分布的 5% 分位作为 `τ`；在长直巷道录 2 min，
统计分布的 95% 分位作为强退化阈值。

### 8.5 与配准质量的关系

退化 ≠ 配准差：

| | 约束充分 | 约束不足 |
|---|---|---|
| **残差小** | ✅ 良好（可作微调监督） | ⚠️ **假收敛**（退化下残差天然小，最危险） |
| **残差大** | ⚠️ 运动模糊/错配 | ❌ 真失败 |

**关键：不能只用残差判退化。** 必须看信息矩阵特征值 ——
退化巷道里点到面残差可以很小（墙面贴合很好），但沿巷道方向完全无约束。
这也是为什么 8.2/8.3 是主判据，残差只是辅助。

---

## 9. ESKF 速度观测模型（融合核心）

### 9.1 观测方程

网络给出**体坐标系速度** `z = v_b^net ∈ R³`，体协方差 `R_b = diag(σ₁², σ₂², σ₃²)`。
ESKF 状态速度 `v` 在**世界系**，姿态 `R = state.rot_`。

```
h(x) = Rᵀ v                    # 预测的体坐标系速度
残差  r = z − Rᵀ v
```

**为什么在体系写而不是世界系**：

- 网络的协方差是**体系对角**（逐轴独立）。体系残差下 `R_meas = diag(σ²)` 保持对角，
  **不需要旋转协方差**。世界系写法需要 `R_w = R diag(σ²) Rᵀ`（稠密）。
- 网络的误差特性（前向准、侧向差、垂向差）在体系里是各向异性的，对角协方差正好表达。

### 9.2 雅可比（右扰动 `R ← R·Exp(δθ)`，与 `NavState::boxplus` 一致）

```
h(x) = Rᵀ v

∂h/∂δv = Rᵀ                       # 3×3  → 写入 h_x_ 的 vel 列块 (cols 12-14)
∂h/∂δθ = [Rᵀ v]× = skew(v̂_b)     # 3×3  → 写入 h_x_ 的 rot 列块 (cols 3-5)
                                    # v̂_b = Rᵀ v = 当前预测的体速度
```

推导：

```
(R·Exp(δθ))ᵀ v = Exp(−δθ)·Rᵀ v ≈ (I − [δθ]×)·Rᵀ v
              = Rᵀ v − [δθ]×(Rᵀ v)
              = Rᵀ v + [Rᵀ v]× δθ
⇒ ∂h/∂δθ = [Rᵀ v]×  ✓
```

**完整观测雅可比（3×23，只填 rot 与 vel 两块）：**

```
H = [ 0_{3×3} | [v̂_b]× | 0_{3×3} | 0_{3×3} | Rᵀ | 0_{3×3} | 0_{3×3} | 0_{3×2} ]
     └ pos ─┘ └ rot ─┘ └ ext_R ┘ └ ext_t ┘ └ vel ┘ └ bg ─┘ └ ba ─┘ └ grav ┘
```

> **这是本方案相对现有轮速路径的关键修正**，三点：
> 1. `H_θ = [v̂_b]×` **必须填**（现有三维轮速路径漏了这项，误差 ∝ ‖v‖·δθ）
> 2. `H_v = Rᵀ` **符号正确**（现有一维轮速路径 `h_x_ = −x_axis_wᵀ` 是发散反馈）
> 3. `R_meas` 用 **σ² = exp(2·logσ)**，不是 σ（现有三维路径 `R_ = σ·I` 少平方一次）

### 9.3 ESKF 侧实现要点

现有 `ESKF::Update` 的 WHEEL_SPEED 分支（`eskf.cpp:166-177`）：

```cpp
h_x_cur.block(0, 12, m, 3) = custom_obs_model_.h_x_;   // 只填 vel 块
```

需要扩展为**允许同时填 rot + vel 两块**：

```cpp
// 新增 ObsType::IMU_VELOCITY 分支
// custom_obs_model_.h_x_ : 3×6  = [ H_theta(3×3) | H_v(3×3) ]
Eigen::MatrixXd h_x_cur = Eigen::MatrixXd::Zero(3, state_dim_);
h_x_cur.block<3,3>(0,  3) = custom_obs_model_.h_x_.leftCols(3);   // rot
h_x_cur.block<3,3>(0, 12) = custom_obs_model_.h_x_.rightCols(3);  // vel
```

其余沿用标准 EKF：

```
K  = P Hᵀ (H P Hᵀ + R_meas)⁻¹
dx = K · r
x  = x ⊞ dx
P  = (I − K H) P ;  P = ½(P + Pᵀ)
```

**建议同时做**（轮速路径欠缺的）：

| 项 | 建议 |
|---|---|
| 迭代 | 至少 2 次重线性化（`r` 对 `R` 敏感，`‖v‖` 大时尤其） |
| 步长限幅 | `‖δθ‖ < 0.5°`、`‖δv‖ < 0.3 m/s` 每帧，超限则本帧拒绝 |
| 卡方门 | `γ = rᵀ (H P Hᵀ + R)⁻¹ r`，若 `γ > χ²_{3,0.99}=11.34` 则拒绝本观测 |
| 有限性 | `dx.allFinite()` 与 `P.allFinite()` 检查（已有） |
| 后验切空间重置 | 与 lidar 路径一致的 `J` 重置（`eskf.cpp:308-320`） |

### 9.4 观测模型实现（`ObsModelImuVelocity`）

```cpp
void LaserOdometryCore::ObsModelImuVelocity(NavState& s, ESKF::CustomObservationModel& obs) {
    const Mat3d R  = s.rot_.matrix();
    const Vec3d vb = R.transpose() * s.vel_;          // v̂_b

    obs.residual_.resize(3);
    obs.residual_ = vel_obs_.v_body - vb;             // z − Rᵀ v

    obs.h_x_ = Eigen::MatrixXd::Zero(3, 6);
    obs.h_x_.block<3,3>(0,0) = SKEW_SYM_MATRIX(vb.cast<float>()).cast<double>(); // H_θ
    obs.h_x_.block<3,3>(0,3) = R.transpose();                                     // H_v

    // 体系对角协方差 + 退化调度 + 全局温度
    Vec3d var = vel_obs_.log_sigma.array().exp().square();   // exp(2·logσ)
    var = (var.array() * vel_reinflation_).max(min_var_);    // 下限保护
    obs.R_ = var.asDiagonal();

    // 退化方向加权：弱方向压方差（提权），强方向放方差（降权）
    if (degeneracy_.valid && use_directional_weight_) {
        const Mat3d U = degeneracy_.weak_basis;              // 列 = 弱方向基
        const Vec3d w = degeneracy_.weight_per_axis;         // 0..1，弱方向→1
        // R_body_eff = U diag(w) Uᵀ + ... 简化实现见 §10.3
    }

    obs.valid_ = vel_obs_.valid && std::isfinite(vel_obs_.v_body.norm());
}
```

### 9.5 与轮速的关系

| | 轮速 | TartanIMU |
|---|---|---|
| 观测量 | 1D 前向速度（或 3D 带强假设） | **3D 体速度** |
| 侧向/垂向 | 依赖非完整约束假设（转弯侧偏会错） | 直接回归（车数据学到侧偏） |
| 打滑 | 严重失真 | 网络不看轮，不受打滑影响 |
| 协方差 | 无（手工 σ） | **网络预测 + NLL 校准** |
| 时延 | 低（CAN 直读） | 高（1s 窗口） |
| 覆盖 | 任何时候有 CAN | 需 10s 上下文 |

**建议：两者并存，按协方差自动加权。**

- 低速/急加减速（网络 1s 均值滞后大）→ 轮速可信（若修好轮速路径）
- 打滑/退化（轮速失真）→ 网络可信
- 二者都失效 → 纯 LIO

轮速路径的既有缺陷（雅可比符号、`wheel_v_mean_` 未写入、时间不同步、
`MapIncremental` 位姿不一致）必须先修，否则并存会互相污染 —— 见 §14。

---

## 10. 融合策略与调度

### 10.1 每帧时序（**修正后的顺序**）

```
Run()
 ├─ 1. SyncPackages()
 ├─ 2. ImuProcess::UndistortPcl        # 用上一帧 vel_ 做去畸变
 ├─ 3. voxel 降采样
 ├─ 4. kf_.Update(LIDAR)               # IEKF，收敛到与地图最佳匹配
 │       └─ 同时产出 DegenerancyInfo   # 从 h_x_ 算，零额外代价
 ├─ 5. 门控判断
 │       ├─ lidar 拒绝？     → 跳过 6-8，但保留网络观测做兜底（见 10.4）
 │       └─ 卡方/年龄/有效性 → 决定 R 缩放
 ├─ 6. kf_.Update(IMU_VELOCITY)        # ★ 新增
 ├─ 7. kf_.Update(WHEEL) / GPS        # 可选
 ├─ 8. 步长限幅 & keep_first 判断
 ├─ 9. state_point_ = kf_.GetX()
 └─ 10. MapIncremental()               # ★ 必须在所有 Update 之后
```

**相对现状的改动点：**

1. 退化检测在 **step 4** 内顺带算出（用 `ObsModel` 已有的 `h_x_`）
2. **`MapIncremental()` 必须在全部观测更新之后**（现状已是如此，但轮速会在此前推动位姿，
   造成最近邻与点变换用不同位姿 —— 见 §14）
3. 网络观测更新后若位姿变化超过阈值（`> 0.3·filter_size_map`），**本帧跳过加点**
   或重算最近邻。这是防地图分裂的硬保险。

### 10.2 退化调度（三档递进）

#### 档 1：二值门控（Phase 1，最简单）

```
if (degeneracy.score > 0.6)      R_scale = 1.0      # 全权重
elif (degeneracy.score > 0.3)    R_scale = 4.0      # 降权 2σ
else                             R_scale = 100.0    # 仅极弱约束（仍有 G2 收益）
```

#### 档 2：连续调度（Phase 2）

```
R_scale = lerp(100.0, 1.0, saturate((score - 0.2) / 0.6))
```

#### 档 3：方向选择性（Phase 3，最优雅）

```
Λ_lidar = H_poseᵀ H_pose            # 6×6 (已缩放)
特征分解  Λ = U diag(λ) Uᵀ
弱方向权重  w_i = exp(−λ_i / λ_ref)   # λ 小 → w→1（提权）
R_world = U diag(w_i · σ_i²) Uᵀ
R_body  = Rᵀ R_world R               # 转回体系（与体系残差一致）
```

效果：**只在 LiDAR 看不见的方向上信任网络**，看得见的方向几乎不动。
这是 Super-Odometry「退化感知加权」的精髓，也是防止"网络拖垮好配准"的关键。

### 10.3 残差与门控细节

```cpp
// 观测接入前的硬门
bool accept =
    vel_obs_.valid                    &&
    (t_state - vel_obs_.t_label) < max_align_gap   (默认 0.3 s) &&
    vel_obs_.v_body.norm() < max_v_norm            (默认 30 m/s) &&
    vel_obs_.log_sigma.minCoeff() > -8             &&   # 网络过度自信时降级
    chi2 < 11.34;                                      # 3 自由度 99%

// 无效时：不更新，但不清除缓冲（等下一帧）
```

### 10.4 LiDAR 拒绝时的兜底

现状：`kf_.Update(LIDAR)` 返回 false 时（`dx` 非有限），后续照跑并加点 —— 危险。
新设计：

```
if (!lidar_accepted) {
    if (vel_obs_valid && degeneracy.score > 0.5) {
        kf_.Update(IMU_VELOCITY);     # 纯速度兜底，抑制飘飞
    }
    跳过 MapIncremental();            # 绝不用未验证位姿污染地图
    return true;
}
```

---

## 11. 协方差解读与标定

### 11.1 从网络输出到 `R_meas`

```
logσ_net ∈ R³  (网络 output_block2 输出)
σ_net    = exp(logσ_net)                    # 逐轴标准差，m/s
var_net  = σ_net² = exp(2·logσ_net)         # 与训练 NLL 的 exp_term 一致
```

**必须用 `exp(2·logσ)`**，与 `losses.py:loss_distribution_diag` 的 `exp_term` 完全对应。
用 `exp(logσ)` 或 `logσ` 本身都会导致量纲错误、权重差几个数量级。

### 11.2 为什么可以信任网络协方差

训练损失就是 Gaussian NLL：

```
L = (v_pred − v_gt)² / (2σ²) + logσ
```

这是**极大似然**形式，最优解处 `σ² ≈ E[(v_pred−v_gt)²]`，
即协方差头被直接训练成"预测误差的均方"。理论上是校准的。

**但实际会偏**（平台迁移、加减速、振动环境），所以需要：

### 11.3 后验标定（强烈推荐，成本极低）

在**配准良好**片段上收集残差：

```
e_i = v_net,i − v_ref,i        # v_ref = SLAM/ESKF 解算的体速度
s_k = sqrt( mean(e_k²) / mean(exp(2·logσ_k)) )     # 逐轴缩放因子
R_meas = diag( (s ⊙ exp(logσ))² )
```

通常 `s` 在 1.5~4 之间（网络往往过度自信）。标定一次，写进 yaml。

进阶：做 **NIS（归一化创新平方）一致性检查**

```
NIS = rᵀ (H P Hᵀ + R)⁻¹ r
理想 E[NIS] = 3（观测维度）
实车 NIS 长期 > 6 → R 偏小（过自信）；< 1.5 → R 偏大（浪费信息）
```

把 NIS 做成在线监控量（`system` 层日志），持续校准。

### 11.4 兜底下限

```
min_var = (0.05 m/s)²      # 无论网络多自信，不低于 5 cm/s
max_var = (5.0  m/s)²      # 无论网络多不确定，不高于 5 m/s（否则等于没用）
```

---

## 12. SLAM → 网络的微调闭环

### 12.1 监督信号构造

**标签来源必须独立于网络自身**，否则自我强化。

```
在【配准良好】帧 k：
  R_k = state.rot_          # ESKF 姿态（LiDAR 主导）
  v_k = state.vel_          # ESKF 世界速度（LiDAR + IMU，未被网络污染时）
  v_b,k = R_kᵀ v_k          # 体速度

对覆盖 [t_w0, t_w0+1.0s] 的窗口：
  v_label = mean({ v_b,k : t_k ∈ 窗口 })       # 与训练标签语义一致（1s 均值）
                                               # 方案2切换后改为末 0.2s 均值
```

**关键约束（防自举）：**

1. **只在 `degeneracy.score < 0.3` 且 `lidar_accepted` 的帧上取标签**
   （退化帧的 `v_k` 本身就不可信）
2. **该窗口的网络输出不得参与过 `v_k`** —— 实现上：
   微调采集时临时把 `R_vel` 设为无穷（即禁用网络观测）跑一段，
   或用"lidar-only 后验"作为标签（在 `Update(LIDAR)` 之后、`Update(IMU_VELOCITY)` 之前快照）
3. 标签帧数不足（窗口内 `< 60%` 有效帧）→ 丢弃该样本
4. 标签速度与网络输出差 `> 3σ` 的样本先入"异常池"人工看，不直接进训练

### 12.2 样本包格式（复用 TartanIMU 的 npz 约定）

```python
# 与 dataset_AirLab 兼容，可直接喂 train_lora.py
retargetted_ts   : [N]      float64   # 200Hz 时间戳
retargetted_imu  : [N, 6]   float64   # ⚠️ 原始顺序 (ax,ay,az,gx,gy,gz)
retargetted_pos  : [N, 3]   float64   # 可用 NaN（infer_trajectory 已支持）
retargetted_quat : [N, 4]   float64   # xyzw，可用 NaN
# 另存：
slam_velocity_body : [N, 3] float64   # 自定义：SLAM 解算体速度（标签源）
slam_confidence    : [N]    float64   # 退化度倒数 / effect_num 归一
segment_meta.json          : {bag, start_t, end_t, degeneracy_mean, ...}
```

> 用 `retargetted_*` 命名是为了**复用现有 dataloader 与 `train_lora.py`**，
> 避免重写训练管线。`slam_velocity_body` 是新增字段，供后续改造标签用。

### 12.3 微调策略（`train_lora.py` 现成）

| 项 | 配置 | 说明 |
|---|---|---|
| 基座 | `tartanimu_weights/checkpoints/unified.pt` | 多头基座 |
| motion_type | `car` | |
| trunk | LoRA rank 8（`inject_lora(..., rank=8, target_modules=("linear","conv1d"))`） | 防遗忘 |
| 速度头 | **全量微调** | 任务输出，必须适配 |
| 协方差头 | **全量微调** | 误差统计必须适配 |
| `velocity_scale` | 全量 | |
| 损失 | `loss_distribution_diag` | 联合速度 + 协方差 |
| LR | 1e-4 AdamW, wd 0.01 | 与默认一致 |
| batch | 32–64 + AMP | 8G 显存友好 |
| 数据混合 | 自有 70% + 原始 car 30% | **防灾难性遗忘** |
| 早停 | 固定验证集 loss 不降则停 | |
| 产出 | `lora_best.pt` + `finetuned_full_model_best.pt` | 两种加载方式 |

### 12.4 在线 vs 离线

| 模式 | 做法 | 风险 | 建议 |
|---|---|---|---|
| **离线（推荐先做）** | 落盘样本 → 每天/每班跑 `train_lora.py` → 人工验收 → 回灌 | 低 | **Phase 2** |
| **近线** | 车端小批量增量微调，固定验证集守护，自动回滚 | 中 | Phase 3 |
| **在线** | 实时梯度更新 | **高**（反馈回路、灾难性遗忘、算力） | 不推荐 |

**近线自动回滚机制（若做）：**

```
每积累 5 min 优质样本 → 后台微调 20 step
→ 在固定验证集上评估 NLL
   ├─ 变好 → 原子替换 adapter（版本号 +1），保存上一版
   ├─ 持平 → 保留当前
   └─ 变差 → 丢弃本次，连续 3 次变差 → 回滚到上一稳定版并告警
```

### 12.5 验证闭环（数据层面）

定期用离线轨迹回放做回归测试（工程内已有 `tests/regression.cpp` 模式可扩展）：

| 指标 | 基线 | 目标 |
|---|---|---|
| 退化段 ATE（长巷道） | 纯 LIO | **降低 ≥ 30%** |
| 退化段速度 RMSE（对比轮速/组合导航） | — | < 0.3 m/s |
| 非退化段 ATE | 纯 LIO | **不劣化（±2%）** |
| 建图重影/分裂帧数 | — | 显著下降 |
| NIS 均值 | — | 2~4 |

---

## 13. 接口与配置设计

### 13.1 消息（新增，放 `deeplumin_msgs`）

```msg
# deeplumin_msgs/msg/localization/ImuVelocity.msg
std_msgs/Header header          # stamp = t_label（窗口中心，方案2后为窗口末端）
string frame_id_body            # "imu_link"
geometry_msgs/Vector3 velocity  # 体坐标系 m/s
float64[3] log_sigma            # 网络原始 log σ
float64[3] sigma                # 标定后 σ = s ⊙ exp(log_sigma)，m/s
float64[3] variance             # σ²，ESKF 直接用
bool valid
uint32 window_id                # 单调递增，调试用
float32 latency_ms              # 推理端到端延迟
float32 context_coverage        # 窗口内有效 IMU 占比 0~1
```

> 用自定义 msg 而非 `geometry_msgs/TwistWithCovarianceStamped`：
> 需要传 `log_sigma`、`window_id`、`valid`、覆盖率等诊断量。
> 遵守 `deeplumin_msgs` 作为唯一业务消息包的架构约定。

话题：`/lio_odometry/imu_velocity`（或 `/tartanimu/velocity`）。

### 13.2 core 层 API

```cpp
// laser_odometry_core.h
void FeedImuVelocity(double t_label, const Vec3d& v_body,
                     const Vec3d& log_sigma, bool valid);

// 私有
struct ImuVelObs {
    double t_label = -1.0;
    Vec3d  v_body  = Vec3d::Zero();
    Vec3d  log_sigma = Vec3d::Constant(-2.0);
    bool   valid   = false;
};
std::deque<ImuVelObs> imu_vel_buf_;   // 带时间戳队列，不是末值锁存！
std::mutex mtx_imu_vel_;

void ObsModelImuVelocity(NavState& s, ESKF::CustomObservationModel& obs);
bool SelectImuVelObsForFrame(double t_state, ImuVelObs& out);
DegeneracyInfo ComputeDegeneracy(const Eigen::MatrixXd& h_x, int effect_num) const;
```

### 13.3 YAML 配置

```yaml
imu_velocity:
  enable: true
  topic: "/lio_odometry/imu_velocity"
  motion_type: "car"

  # ---- 时间对齐 ----
  max_age: 0.30            # t_state - t_label 超此值丢弃 (s)
  max_align_gap: 0.30      # 同上，用于 IMU 递推补偿
  lag_compensation: true   # 是否做方案1的加速度递推补偿

  # ---- 协方差 ----
  sigma_scale: 2.5         # 后验标定的全局缩放 s（§11.3）
  min_sigma: 0.05          # m/s，下限
  max_sigma: 5.00          # m/s，上限
  reinflation: 1.0         # 额外方差膨胀（保守起步可设 4.0）

  # ---- 门控 ----
  chi2_gate: 11.34         # 3 自由度 99%
  max_v_norm: 30.0         # m/s
  max_dtheta_per_frame: 0.5  # deg，步长限幅
  max_dv_per_frame: 0.30     # m/s

  # ---- 退化调度 ----
  degeneracy:
    enable: true
    mode: "binary"          # binary | continuous | directional
    lambda_pos_ref: 200.0   # 标定得到
    lambda_rot_ref: 500.0
    normal_aniso_max: 50.0
    score_high: 0.6         # 全权重
    score_low: 0.3          # 几乎不加权
    r_scale_high: 1.0
    r_scale_low: 100.0

  # ---- 网络（推理节点侧，镜像一份便于校验） ----
  network:
    config: "/home/hhy/ros2/TartanIMU/tartanimu_weights/config/unified.yaml"
    checkpoint: "/home/hhy/ros2/TartanIMU/tartanimu_weights/checkpoints/unified.pt"
    lora_adapter: ""        # 空 = 不用 adapter
    device: "cuda"
    imu_freq: 200           # 冻结
    window_time: 1.0        # 冻结
    seq_len: 10
    sample_freq: 40         # 冻结
    stride_hz: 10           # 推理节拍
    label_policy: "window_mean"   # window_mean | window_end  (方案1/2)
```

### 13.4 诊断话题（强烈建议，用于标定与回归）

| 话题 | 内容 | 频率 |
|---|---|---|
| `/lio_odometry/degeneracy` | score, λ_min_t, λ_min_r, normal_aniso, effect_num | 每帧 |
| `/lio_odometry/imu_velocity_raw` | 网络原始输出 + σ | 10 Hz |
| `/lio_odometry/imu_velocity_used` | 实际进更新的观测 + R_scale + NIS | 每帧 |
| `/lio_odometry/adapt_samples` | 微调样本触发（含 confidence） | 事件 |

---

## 14. 与现有缺陷的衔接

> 前期对 `laser_odometry_core.cpp` 轮速路径的分析发现若干缺陷。
> 新观测源必须**不复现**这些坑，并顺手修掉会被网络放大的部分。

| # | 已知缺陷 | 新方案如何规避 |
|---|---|---|
| 1 | 三维轮速 `H_θ` 缺失，误差 ∝ ‖v‖·δθ | §9.2 明确 `H_θ = [v̂_b]×`，必填 |
| 2 | 一维轮速 `h_x_ = −x_axis_wᵀ` 符号反，发散反馈 | §9.2 推导给出正确符号；并加卡方门 + 步长限幅 |
| 3 | `wheel_v_mean_` 永不写入（`ConsumeWheelSpeedForFrame` 未接线） | 新观测源带时间戳队列 + `SelectImuVelObsForFrame` **必须接线并写单测** |
| 4 | 轮速"末值锁存"，无时间同步 | §5.3 队列 + §6.4 时间对齐 + `max_age` 门 |
| 5 | `R = σ·I` 少平方一次 | §11.1 强制 `exp(2·logσ)` |
| 6 | 三维轮速无 Huber/无门限 | §10.3 卡方门 + σ 下限 + NIS 监控 |
| 7 | `MapIncremental` 用更新后位姿配更新前最近邻 | §10.1 把所有 Update 放在加点前；位姿变动超阈值则**跳过加点** |
| 8 | lidar 拒绝后仍加点 | §10.4 拒绝即跳过 `MapIncremental` |
| 9 | `FeedWheelSpeed` 忽略 timestamp、不查 `msg->valid` | 新接口强制传 `t_label`；推理端 `valid`/`coverage` 必须检查 |
| 10 | `residuals_` 复用上一帧小残差的 lightning-lm 怪癖 | 网络更新推动位姿后会放大该问题；§10.1 的"位姿变动跳过加点"是硬保险。长期建议修掉该怪癖 |

**最低限度的前置修复（在接网络之前做）：** #7、#8。
否则网络更新一动位姿，地图立刻分裂，会把网络"背锅"。

---

## 15. 分阶段实施计划与验收指标

### Phase 0：数据采集与离线验证（1 周，不改 lio_odometry）

| 任务 | 产出 |
|---|---|
| 录 3~5 段矿车 bag（含长巷道、硐室、转弯、加减速） | `data/*.bag` |
| 离线跑 `infer_trajectory.py`，验证网络在矿车 IMU 上的速度质量 | 速度曲线 vs 轮速 |
| 统计网络误差：`e = v_net − v_wheel`（轮速作弱参考） | RMSE 报表，定 `sigma_scale` 初值 |
| 确认 IMU 频率、单位、坐标系、时间戳质量 | 6.2 的重采样参数 |

**验收**：网络速度 RMSE < 0.5 m/s（匀速段）；协方差 NIS 落在 2~4。

### Phase 1：融合主链路（2 周）

| 任务 | 文件 |
|---|---|
| 修 §14 的 #7 #8（MapIncremental 一致性） | `laser_odometry_core.cpp` |
| `ImuVelocity.msg` + 订阅 + `FeedImuVelocity` | `deeplumin_msgs`、`ros_adapter` |
| `ImuVelObs` 队列 + `SelectImuVelObsForFrame` | `laser_odometry_core.{h,cpp}` |
| `ObsModelImuVelocity`（正确雅可比） | `laser_odometry_core.cpp` |
| `ESKF::Update(IMU_VELOCITY)` 分支（rot+vel 双块） | `eskf.{h,cpp}` |
| 档 1 退化检测（λ_min + effect_num） | 新 `degeneracy_monitor.{h,cpp}` |
| 档 1 二值调度 | `laser_odometry_core.cpp` |
| `tartanimu_node`（Python）：200Hz 重采样 + 窗口装配 + 推理 | 新包 `tartanimu_ros` |
| 诊断话题 + YAML 配置 | — |
| 单测：雅可比数值校验（`math::num_diff`）、时间对齐、协方差量纲 | `tests/` |

**验收**：

- 退化段（长巷道）ATE 较纯 LIO 降低 ≥ 20%
- 非退化段 ATE 不劣化（±2%）
- 无建图分裂回归（对比纯 LIO 的 registered_scan）
- 卡方门拒绝率 < 5%（拒绝率过高说明 R 或时延有问题）

### Phase 2：微调闭环 + 标定（2 周）

| 任务 | 产出 |
|---|---|
| 样本采集器（按 §12.2 格式落盘） | `tartanimu_adapt/collector` |
| 后验协方差标定（§11.3） | `sigma_scale` 写入 yaml |
| 方案 2 标签改造（末 0.2s 均值）+ LoRA 微调 | `adapter_v2.pt` |
| 档 2 连续退化调度 | — |
| 回归测试集 + A/B 脚本 | `tests/` |

**验收**：方案 2 上线后，**加减速段**速度误差较方案 1 降低 ≥ 30%；
NIS 均值 2~4。

### Phase 3：部署优化（可选）

- TorchScript/TensorRT 内嵌 C++（`ImuVelocityEstimator`）
- 档 3 方向选择性加权
- 近线自动微调 + 回滚
- 无 GPU 时的退化触发式推理

---


### 16. 补充分析
# 结合Super Odometry2.0思想的TartanIMU时间对齐分析
> Super Odometry2.0核心思想：学习得到的IMU网络输出并非瞬时观测，而是一段IMU窗口的统计量；不要强行将窗口均值速度伪装成瞬时速度；借助退化感知与不确定性机制，将窗口统计量作为**一段时间区间上的约束**，而非单点时刻约束；同时严格保证训练侧与推理侧时序语义对齐，这是解决时间滞后的根本手段。

TartanIMU原生输出为1s窗口内机体速度均值，有效时刻为窗口中心`t_label = t_start + 0.5s`；而lio‑odometry的ESKF状态点取激光扫描末端`t_state`，二者存在固有的**0.5s语义滞后**。该滞后属于网络输出语义带来的模型滞后，不等同于硬件传感器时间同步偏差。

论文未直接给出工程实现代码，但从论文设计、损失函数定义、样本构造、自适应融合策略，可以提炼出Super Odometry2.0三条时序设计原则，并映射到本ESKF架构的工程实现路径。

## Super Odometry2.0 三条核心时序原则
### 原则1：训练‑推理观测语义必须严格对齐（根源约束）
训练集标签本身就是1s窗口的平均机体速度，NLL损失针对窗口平均速度做优化，并非瞬时速度。
网络本质学习的是1s窗口统计均值，不是某一时刻瞬时速度。若推理阶段直接把窗口均值当作窗口中心时刻的瞬时速度送入滤波器，在加减速工况下会引入固有模型误差。

SuperOdometry2.0处理思路：**优先修改训练标签定义，而不是推理后做信号补偿；网络输入窗口几何参数保持不变，仅改变监督标签，让网络回归窗口末端统计量（末0.2s均值/窗口末端瞬时速度）。**
> 注意：窗口输入几何不可改动（1s窗口、200Hz采样、step=5），改动会造成预训练权重分布偏移失效，仅允许修改标签，通过LoRA微调完成适配。

### 原则2：网络观测属于区间约束，而非单点观测（因子图原生优势）
SuperOdometry2.0采用固定滞后平滑因子图架构，图中维护历史一段时间的状态节点。网络输出对应的时间区间为$[t_{w0}, t_{w0}+1.0]$，可直接对该时间区间内的状态建立平均速度残差因子，不需要绑定单一离散时刻。
残差形式：
$$
r=\left( \frac{1}{T}\int_{t_{w0}}^{t_{w0}+T} R(\tau)^T v(\tau) d\tau\right)-v_{net}
$$
直接匹配网络输出的窗口均值语义，消除时间补偿的需要。

> 本工程现状：使用ESKF滤波，仅维护当前时刻单一状态，不保存完整历史状态链，**无法直接实现区间平均因子**，这是架构带来的硬性约束。

### 原则3：时间对齐残余误差纳入不确定性，交由自适应退化加权消化
SuperOdometry2.0不追求绝对完美的时间对齐：
1. 网络输出`log‑σ`本身包含运动动态带来的预测不确定性；
2. 将时间对齐引入的模型误差进一步做协方差膨胀；
3. 结合退化检测做动态调度：
    - 强退化场景：激光约束匮乏，容忍模型误差，信任网络观测；
    - 几何良好场景：放大网络观测协方差，以激光观测为主，网络仅作为弱正则项。

即：时间对齐无法做到完全零误差，残余模型误差不能依靠硬补偿掩盖，一部分需要体现在观测不确定性中，由自适应融合机制吸收。

## 映射至ESKF工程两套方案（对照SuperOdometry2.0思想做优劣评判）
### 方案A：推理后IMU加速度递推补偿（文档方案1，Phase1快速原型）
逻辑：拿到窗口中心`t_label`的均值速度$v_{net}$，利用ESKF内部IMU加速度积分$\Delta t$，外推至ESKF状态时刻`t_state`。

优点：无需重训网络，直接使用官方预训练权重，开发代价低。

存在两点和SuperOdometry2.0设计思想冲突：
1. 网络输出为窗口统计均值，并非`t_label`瞬时速度；加速度外推隐含窗口内加速度恒定假设，急加减速工况假设失效，引入额外模型误差。
2. 外推带来的模型误差**必须叠加到观测协方差R做方差膨胀，不可直接将补偿后速度作为无误差观测**。

适用范围：Phase1原型验证，巷道低速匀速场景；不适用于矿车急加减速大机动工况，不作为最终上线方案。

### 方案B：LoRA微调修改标签（文档方案2，对齐SuperOdometry2.0的优选上线方案）
保持网络输入窗口几何全部参数不变（10s上下文、子窗口1s、200Hz采样、step=5），仅修改监督标签：
- 旧标签：完整1s窗口全部样本速度均值；
- 新标签：窗口末尾0.2s均值 / 窗口末端瞬时机体速度。

网络仍然接收完整1s IMU上下文，保留原有感知能力，回归物理量切换为靠近窗口末端的速度，输出有效时刻更新为窗口末端$t_{w1}=t_{w0}+1.0\mathrm{s}$，推理完成时刻与ESKF`t_state`的时间差压缩至几十毫秒级别。

该方案贴合SuperOdometry2.0核心设计：
1. **训练语义与推理语义保持一致**，不在推理阶段做风险较高的信号外推，时序问题在训练侧解决；
2. 协方差头同步参与LoRA微调，输出`log‑σ`可表征新标签下全部预测不确定性；
3. 残余几十毫秒时差可当作普通传感器时间偏移，仅需简单一阶补偿甚至直接通过`max_age`门限过滤。

代价：需要使用矿车真实数据集执行LoRA+输出头微调，安排在Phase2实施。

## 架构鸿沟对比：SuperOdometry2.0因子图 vs 本工程ESKF‑lio_odometry
|项目|SuperOdometry2.0（Fixed‑lag因子图）|当前ESKF lio_odometry|
|---|---|---|
|状态存储|维护过去1~2s历史状态链|仅保存当前时刻单状态|
|网络观测处理|支持区间平均速度因子，数学完全匹配窗口均值输出，几乎无需补偿|观测只能绑定某一离散时刻，必须处理时间差|
|时间对齐难度|低，原生支持区间约束|高，受滤波架构限制|
|迁移改造代价|高，需要替换为固定滞后平滑器，现有代码几乎无法复用|低，维持ESKF架构，优先采用LoRA改标签折中方案|

> 工程现实约束：本项目不改造为因子图；**最优折中方案为方案B LoRA修改标签**，而非持续优化推理后外推补偿。

## 完整落地时间对齐策略
### Phase‑1（原型验证，使用官方预训练权重）
1. 网络输出语义保持1s窗口均值，时间戳标记窗口中心`t_label`；
2. 采用IMU加速度递推做一阶时间补偿；
3. 将时间补偿引入的模型误差叠加进观测协方差R（`reinflation`方差膨胀），不能直接信任补偿后速度；
4. 设置硬时间门限：$|t_{state}-t_{label}|>0.3\mathrm{s}$直接丢弃本次观测；
5. 启用退化自适应加权：动态剧烈的加减速场景自动放大R、降低网络权重，由修复后的轮速做信息互补；允许卡方门拒绝网络异常观测。

> 阶段认知：加速度补偿仅为缓解手段，无法消除“窗口均值映射瞬时状态”带来的模型不匹配。

### Phase‑2（对齐SuperOdometry2.0的上线方案）
1. 网络输入窗口几何参数全部保留不变，执行LoRA微调，标签切换为窗口末端附近速度统计量；输出时间戳更新为窗口末端$t_{w1}$；
2. 残余几十毫秒时差作为普通传感器时间偏移，做小量一阶补偿；
3. 协方差头同步微调，网络输出`logσ`覆盖运动动态带来的预测不确定性；
4. 保留方差膨胀、时间门限、卡方检验、退化自适应加权作为兜底保护。

## 两条必须遵守的工程戒律（源自SuperOdometry2.0）
1. **禁止修改网络输入窗口超参（`window_time=1s`、`imu_freq=200Hz`）**，改动将导致预训练权重OOD失效；仅允许修改训练标签。
2. 时间对齐残余误差不能仅依靠算法硬补偿，误差必须反映到观测不确定性，交由退化自适应加权消化，不假设时间对齐完美无偏差。

## 冷启动时序补充
SuperOdometry2.0中TartanIMU需要完整10s IMU上下文才输出第一个有效观测，该10s是LSTM时序上下文需求，并非硬件延迟。工程实现中前10s标记观测无效，自动回退纯LIO，禁止零填充伪造输入，避免分布漂移。

> 核心总结：SuperOdometry2.0对时间对齐的核心观点：时序滞后根源来自网络输出属于窗口统计量而非瞬时观测；优先在训练侧修正标签语义，而非推理端强行外推瞬时速度；残余时序误差进入观测不确定性，由退化自适应加权消化；因子图架构可直接构建区间残差，受限于ESKF架构，本项目采用LoRA修改标签作为最优工程折中。 


---

### 17. 风险与开放问题

| # | 风险 | 影响 | 缓解 |
|---|---|---|---|
| R1 | **1s 窗口均值标签在急加减速时滞后** | 速度约束偏，可能加剧而非缓解 | 方案 2（改标签）；急加减速时靠卡方门拒绝；轮速补位 |
| R2 | **网络在矿车振动/电磁环境下的域差距** | σ 校准失效、速度偏置 | Phase 0 先验证；LoRA 微调；`sigma_scale` 后验标定 |
| R3 | **IMU 单位/通道序搞错** | 完全失效但不报错 | 单测：静止时 ‖acc‖≈9.81；`imu_velocity_raw` 与轮速对比告警 |
| R4 | **Ouster 100Hz → 200Hz 插值引入失真** | 网络输入分布偏移 | Phase 0 对比 100Hz 原生微调版本；PCHIP 优于线性 |
| R5 | **微调自我强化**（退化段标签不可信） | 网络越训越歪 | §12.1 严格只在低退化段取标签；lidar-only 快照 |
| R6 | **冷启动 10s 无观测** | 启动段纯 LIO | 接受；或开机静止初始化期间完成 |
| R7 | **算力不足（无 GPU）** | 推理 100ms+ | 退化触发式推理；TensorRT FP16 |
| R8 | **网络与轮速互相打架** | 位姿抖动 | 修好轮速路径再并存；按协方差加权；卡方门各自独立 |
| R9 | **LSTM 流式续接与训练分布不符** | 流式方案精度下降 | Phase 1 用全上下文重跑（严格对齐训练）；流式留 Phase 3 并单独评估 |
| R10 | **退化阈值标定依赖场景** | 误判 | 双场景标定 + NIS 在线监控 + 可热调 yaml |

**开放问题（需要实车数据回答）：**

1. 矿车 IMU 的实际频率、抖动、时间戳质量？（决定 §6.2 重采样实现）
2. `car` 头 vs `expert_car.pt`，哪个在矿车上更好？（Phase 0 A/B）
3. 网络速度的**偏置**（bias）有多大？是否需要在观测里加一个可估的 `v_bias` 状态？
4. 侧向速度 `vy` 在矿车转弯（铰接/滑移转向）下的可信度？可能需要按轴给不同的 `sigma_scale`
5. 退化判据阈值：需要巷道/硐室两类场景的数据分布

---

## 17. 附录

### A. 关键代码事实索引

| 事实 | 位置 |
|---|---|
| 输入通道序 `(g,a)` | `TartanIMU/tartan_imu/dataloader/dataset_AirLab.py:161`、`infer_trajectory.py:build_features` |
| 重力保留在加计 | `dataset_AirLab.py:128-130`（`use_local_coord: true`） |
| 标签 = 窗口 1s 体速度均值 | `dataset_AirLab.py:169-175` |
| `pred_cov` = log σ | `tartan_imu/model/common/losses.py:18-30` |
| 窗口几何 `2000 帧 / 10 窗 / 40 采样每窗` | `dataset_AirLab.py:284-293`、`infer_trajectory.py:139-156` |
| trunk 支持 `(hn,cn)` 但 `FoundationModel` 丢弃 | `model_lstm/trunks.py:236-262`、`model_lstm/heads.py:108-115` |
| LoRA 注入/导出 | `tartan_imu/model/lora_layers.py`、`train_lora.py` |
| ESKF 状态布局 `vel@12`、`rot@3` | `lio_odometry/include/lio_odometry/core/types.h:244-255` |
| ESKF WHEEL 分支只填 vel 块 | `lio_odometry/src/core/eskf.cpp:166-177` |
| ESKF lidar 分支 `topLeftCorner(_,12)` | `lio_odometry/src/core/eskf.cpp:242-249` |
| 右扰动 boxplus | `types.h:329-342` |
| 去畸变用 `vel_` | `imu_processing.cpp:232` |

### B. 雅可比数值校验单测模板

```cpp
// tests/test_imu_vel_jacobian.cpp
TEST(ImuVelJacobian, MatchesNumericDiff) {
    NavState x; x.rot_ = SO3::exp(Vec3d(0.1, -0.2, 0.05));
    x.vel_ = Vec3d(1.5, -0.3, 0.2);
    ESKF::CustomObservationModel obs;
    core.ObsModelImuVelocity(x, obs);           // 填 residual_ / h_x_

    const double eps = 1e-6;
    // 对 δv 三轴、δθ 三轴分别做数值微分，与 h_x_ 对比
    for (int i = 0; i < 3; ++i) {
        NavState xp = x, xm = x;
        Vec3d d = Vec3d::Zero(); d[i] = eps;
        xp.vel_ += d; xm.vel_ -= d;
        Vec3d rp = residual_of(xp), rm = residual_of(xm);
        Vec3d num = -(rp - rm) / (2 * eps);     // = ∂h/∂δv 的第 i 列
        EXPECT_NEAR((num - obs.h_x_.col(3 + i)).norm(), 0.0, 1e-4);
    }
    for (int i = 0; i < 3; ++i) {
        NavState xp = x, xm = x;
        Vec3d d = Vec3d::Zero(); d[i] = eps;
        xp.rot_ = x.rot_ * SO3::exp(d);
        xm.rot_ = x.rot_ * SO3::exp(-d);
        Vec3d num = -(residual_of(xp) - residual_of(xm)) / (2 * eps);
        EXPECT_NEAR((num - obs.h_x_.col(i)).norm(), 0.0, 1e-4);
    }
}
```

### C. 时间同步时序图（方案 1）

```
t(s)   0     1     2     3     4     5     6     7     8     9    10
       ├─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴────┤
       │                 10 s 上下文 (2000 帧 @200Hz)                │
       │  win0  win1  win2  win3  win4  win5  win6  win7  win8  win9 │
       ▲                                                             ▲
     t_w0                                                          t_w10
       │                                                             │
       └── t_label(取 win9) = t_w9 + 0.5s = 9.5s                     │
                                        网络推理时刻 ≈ 10.0s + latency
                                        ESKF t_state ≈ 10.0s
                                        Δt = t_state − t_label ≈ 0.5s
                                        → 用 a·Δt 递推补偿（方案1）
                                        → 或改标签为窗口末端（方案2，Δt≈0）
```

### D. 术语

| 术语 | 含义 |
|---|---|
| `t_label` | 网络输出的有效时刻（窗口中心或窗口末端） |
| `window` | 1.0 s 的 IMU 片段（200 帧），ResNet 的最小处理单元 |
| `context` | 10 个 window = 10 s，LSTM 的完整输入 |
| `log_sigma` | 网络协方差头输出，`σ = exp(log_sigma)` |
| NIS | Normalized Innovation Squared，`rᵀ S⁻¹ r`，一致性指标 |
| LoRA | Low-Rank Adaptation，只训低秩增量，基座冻结 |
| 退化 | LiDAR 观测在某些自由度上信息不足（长走廊/巷道） |

---

*文档版本 v1.0 · 2026-09-25 · 设计稿，未改代码*
