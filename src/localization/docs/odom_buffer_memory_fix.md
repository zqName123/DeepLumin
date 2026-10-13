# 里程计内存占用与数据积压治理 —— 改动方案与实施步骤

> 适用包：`lio_odometry`、`slam_odometry_v2`
> 状态：待实施
> 原则：**只做最小改动、每项可独立回退、不引入新机制使问题难排查**

---

## 0. 背景与问题

实测两个现象：

1. **运行时间越长，内存占用越大**（长时间静止时也增长）；
2. **odom 系点云发布突然长时间卡顿**。

根因定位（详见分析过程）：

| # | 根因 | 影响 |
|---|---|---|
| R1 | ROS 订阅队列 `200000`（IMU/LiDAR 各一个） | 卡顿时按 `条数 × 单条体积` 蓄水：LiDAR 单帧 ~2 MB，理论上限数十 GB |
| R2 | `lidar_buffer_` / `time_buffer_` **无界** | 处理跟不上时单调增长，且只进不出地"播旧账" |
| R3 | `slam_odometry_v2` 的 `imu_buffer_` **无界** | LiDAR 断流时 `SyncPackages` 直接 return，IMU 一枚不消费 |
| R4 | **无追帧策略**：掉实时后永远追不回 | 发布点云时间戳持续落后 → 卡顿 |
| R5 | 单线程 `spinOnce() + tick()`，一次卡顿 → 回调堆积 → 队列与缓冲同时涨 | 卡顿与内存互为因果 |

**本次只解决 R1~R4。** R5（单线程结构）不动，避免引入线程同步问题使排查复杂。

---

## 1. 范围

### 1.1 本次实施

| Stage | 内容 | 对应根因 |
|---|---|---|
| **Stage 1** | 订阅队列长度配置化并收敛 | R1 |
| **Stage 2** | 缓冲区时间裁剪 + 条数兜底 + drop-to-latest 追帧 | R2/R3/R4 |

### 1.2 本次暂缓（原因与后续入口见 §8）

| Stage | 内容 | 暂缓原因 |
|---|---|---|
| Stage 3 | 地图插入门控（静止不插点） | 涉及估计器输入，误判静止风险大；且静止误判本身少见，收益优先级低于内存/积压 |
| Stage 4 | iVox 三轴淘汰（距离+年龄+容量） | 依赖 Stage 3 先堵住无效插入，否则易颠簸 |
| Stage 5 | 主循环阻塞 IO（`std::cout` / 逐行 flush）清理 | 属卡顿诱因但不属内存/积压，改动分散 |

---

## 2. 回退设计总则

| 原则 | 做法 |
|---|---|
| **开关化** | 每个 Stage 有 yaml 开关；关闭后**走原逻辑分支**，旧代码保留在 `else` 中不删除 |
| **加法式配置** | 只新增 yaml key，不改/不删旧 key；不写新 key 时用代码默认值 |
| **提交隔离** | Stage 1、Stage 2 各一个 commit，可单独 `git revert` |
| **改动集中** | 新逻辑集中在**新增私有方法** `PruneBuffers()`，对现有函数只在开头加一行调用 |
| **不引入新文件/新包** | 配置结构体放进各包已有的 config / core 头文件，避免额外的构建与依赖面 |
| **不改 syslog 库** | 全项目共用，改动回退面太大；只在调用侧控制日志量 |

**运行时一键回退**（粘回 yaml 即恢复旧行为，无需重编译）：

```yaml
queue_size:
  enable_override: false
buffer_policy:
  enable: false
```

---

## 3. Stage 1：订阅队列收敛

### 3.1 问题

```cpp
// slam_odometry_v2_node.cc:122-123
imu_sub_   = nh_.subscribe(config_.sensors.imu,   200000, ...);
lidar_sub_ = nh_.subscribe(config_.sensors.lidar, 200000, ...);
// ros_adapter.cpp:46,52 同样是 200000
```

队列单位是**条**，不是字节。LiDAR 200000 × 2 MB ≈ 40 GB 理论上限。同时单线程 `spinOnce()` 会一次性执行队列里所有回调，队列越大恢复越慢。

### 3.2 收敛原则

```
queue_size ≈ 消息频率(Hz) × 可容忍最大卡顿(s)      // 卡顿容忍取 0.5~2 s
```
对实时里程计而言，**迟到的数据无价值**，宁可丢帧（LiDAR 丢帧安全，IMU 会跨空档积分）。

### 3.3 推荐值

| 话题 | 频率 | 单条体积 | 采用值 | 旧值（lio_odometry） | 对应时间 |
|---|---|---|---|---|---|
| IMU | 200 Hz | ~0.5 KB | **200** | 200000 | 1 s |
| LiDAR | 10 Hz | ~2 MB | **10** | 200000 | 1 s |
| 轮速 | 50 Hz | ~0.2 KB | **100** | 1000 | 2 s |
| GNSS | 10 Hz | ~0.2 KB | **50** | 100 | 5 s |
| Correction | 10 Hz | ~0.2 KB | **50** | 100 | 5 s |

> **说明**：`slam_odometry_v2` 在本文档编写前已把 IMU/LiDAR 队列手动改为 `200`/`10`（已提交状态），本次沿用该值作为两包统一默认，仅做**配置化**，不改变其行为。若希望更宽松的抖动容忍（如 2 s），把 `queue_size.imu` 调到 400 即可，无需改代码。

### 3.4 改动点

| 包 | 文件 | 位置 | 改动 |
|---|---|---|---|
| lio_odometry | `include/lio_odometry/ros/config.h` | `NodeConfig` 内 | 新增 `QueueSizeConfig` 结构体字段 `queue` |
| lio_odometry | `src/ros/config.cpp` | `LoadFromYAML` | 读 `yaml["lio_odometry"]["queue_size"]` |
| lio_odometry | `src/ros/ros_adapter.cpp` | `:45-77` 五处 `subscribe` | 字面量 → `config_.queue.*`（带 override 分支） |
| slam_odometry_v2 | `include/slam_odometry_v2/ros/slam_odometry_v2_config.hpp` | 结构体区 | 新增 `QueueSizeConfig` + `NodeConfig::queue` |
| slam_odometry_v2 | `src/ros/slam_odometry_v2_config.cpp` | `loadNodeConfig` | `pnh.param("queue_size/...")` |
| slam_odometry_v2 | `src/slam_odometry_v2_node.cc` | `:122-129` 四处 `subscribe` | 同上 |
| 两包 | `config/*.yaml` | 新段 `queue_size` | 见 §5 |

### 3.5 新增结构体（两包字段一致）

```cpp
struct QueueSizeConfig {
    bool enable_override = true;   // false → 恢复旧常量
    int imu = 400;
    int lidar = 8;
    int wheel = 100;
    int gnss = 50;
    int correction = 50;
};
```

### 3.6 使用方式

```cpp
const int q_imu = config_.queue.enable_override ? config_.queue.imu : 200000;
const int q_lidar = config_.queue.enable_override ? config_.queue.lidar : 200000;
// wheel: 1000 / gnss: 100 / correction: 100 同理
```

> **保留旧常量在代码中**（200000/1000/100），`enable_override: false` 时逐字恢复，回退不需要重编译。

### 3.7 验收

- 正常运行：`rostopic hz` 无掉频（队列足够吸收抖动）；
- 主循环人为卡 3 s 后恢复：无崩溃，`rostopic hz` 恢复 10 Hz；
- `VmRSS` 增量 < 20 MB（原为数百 MB 量级）。

### 3.8 回退

`queue_size.enable_override: false`；或 `git revert` Stage 1 commit。

---

## 4. Stage 2：缓冲区治理

### 4.1 问题

| 缓冲 | slam_odometry_v2 | lio_odometry |
|---|---|---|
| `lidar_buffer_` / `time_buffer_` | 无界（`laser_odometry.cc:1105`） | 无界（`laser_odometry_core.cpp:350`） |
| `imu_buffer_` | 无界（`:1130`） | 5000 上限，超了静默 `pop_front`（`:327`） |
| pop 速率 | `SyncPackages` 每 tick 只弹 **1 帧 LiDAR** | 同 |

`pop` 只是按消费速率排水，**不消除积压**。进 > 出时缓冲单调增长；一旦掉实时，永远播旧账。

另外 `SyncPackages` 有三条会让 pop **完全停摆**的早退路径：
1. `lidar_buffer_.empty()` → 直接 return，**IMU 一枚不消费**（LiDAR 断流时 IMU 无限涨）；
2. `last_timestamp_imu_ < lidar_end_time_` → 等 IMU，LiDAR 只进不出；
3. （相关）`slam_odometry_v2` 的 `SyncPackages` **未加 `mtx_buffer_`**，与 `lio_odometry` 不一致。

### 4.2 策略定义

**核心：时间裁剪为主 + 条数兜底 + drop-to-latest 追帧，两 buffer 联合裁剪。**

```
① LiDAR 时间窗   丢弃 time_buffer_.front() < t_newest_lidar - lidar_keep_sec
② LiDAR 条数上限 while (size > lidar_max_frames) 丢最旧
③ 追帧          if (size > lidar_catchup_frames) 只留最新 1 帧，其余丢弃
④ 作废半状态     ③/①/② 丢过帧时: lidar_pushed_ = false; measures_.scan_.reset();
⑤ IMU 下界      t_floor = (LiDAR 非空) ? time_buffer_.front() - imu_keep_margin_sec
                                       : imu_buffer_.back()->ts - imu_orphan_keep_sec
                 丢弃 timestamp < t_floor 的 IMU
⑥ IMU 条数兜底  while (size > imu_max_samples) 丢最旧
```

**为什么 LiDAR 可以丢、IMU 不能乱丢**：LiDAR 丢帧由 IMU 跨空档积分（现有 `imu_gap_limit: 0.15` 已支持）；IMU 丢在窗口内会破坏去畸变与预测。因此 ⑤ 的下界必须跟随**最旧保留的 LiDAR 帧**，而不是 IMU 自身的时间窗。

**为什么需要 ④**：`lidar_pushed_ == true` 时 `measures_.scan_` 指向的是 `lidar_buffer_.front()`。一旦 ①②③ 丢掉了 front，`measures_.scan_` 与 `lidar_end_time_` 就成了悬空/过期状态，下一帧会用错误时间窗消费 IMU。**这是本 Stage 最容易写错的点，单测必须覆盖。**

### 4.3 改动点

| 包 | 文件 | 位置 | 改动 |
|---|---|---|---|
| lio_odometry | `include/lio_odometry/core/laser_odometry_core.h` | 私有区 | 新增 `BufferPolicyConfig buffer_policy_`、`BufferDropCount`、`PruneBuffers()` 声明、`dropped_*` 计数 |
| lio_odometry | `src/core/laser_odometry_core.cpp` | `SyncPackages()` 开头（`:368` 锁之后、空判之前） | 加一行 `auto dropped = PruneBuffers();` + 限频日志 |
| lio_odometry | 同上 | `LoadParamsFromYAML` | 读 `yaml["buffer_policy"]` |
| slam_odometry_v2 | `include/faster_lio/laser_odometry.h` | 私有区 | 同上 |
| slam_odometry_v2 | `src/faster_lio/laser_odometry.cc` | `SyncPackages()` 开头（`:1184`） | 加锁（对齐 lio）+ 同上调用 |
| slam_odometry_v2 | 同上 | `LoadParamsFromYAML` | 读 `yaml["buffer_policy"]` |
| 两包 | `config/*.yaml` | 新段 `buffer_policy` | 见 §5 |

> **调用点只有一处**（`SyncPackages` 开头），不改回调、不改 `Run()`、不改 `MapIncremental`，保证 diff 最小。

### 4.4 新增类型（两包逻辑逐行一致，类型按各自 IMU 类型实例化）

```cpp
struct BufferPolicyConfig {
    bool   enable                 = true;   // false → PruneBuffers 直接 return，等于不存在
    double lidar_keep_sec         = 0.8;    // LiDAR 时间窗
    int    lidar_max_frames       = 10;     // LiDAR 条数硬上限（时间戳冻结时兜底）
    int    lidar_catchup_frames   = 3;      // 超过即 drop-to-latest
    double imu_keep_margin_sec    = 0.2;    // IMU 下界 = 最旧 LiDAR 时间 - margin
    int    imu_max_samples        = 3000;   // IMU 条数硬上限
    double imu_orphan_keep_sec    = 2.0;    // LiDAR 断流时 IMU 相对自身的保留窗
};

struct BufferDropCount {
    int lidar = 0;
    int imu   = 0;
};
```

### 4.5 `PruneBuffers()` 伪代码

```cpp
BufferDropCount LaserOdometryCore::PruneBuffers() {
    BufferDropCount drop;
    if (!buffer_policy_.enable) return drop;

    // ---- LiDAR 侧 ----
    const double t_newest = time_buffer_.empty() ? 0.0 : time_buffer_.back();
    while (!time_buffer_.empty() &&
           (t_newest - time_buffer_.front()) > buffer_policy_.lidar_keep_sec) {
        time_buffer_.pop_front();
        lidar_buffer_.pop_front();
        ++drop.lidar;
    }
    while (static_cast<int>(lidar_buffer_.size()) > buffer_policy_.lidar_max_frames) {
        time_buffer_.pop_front();
        lidar_buffer_.pop_front();
        ++drop.lidar;
    }
    if (static_cast<int>(lidar_buffer_.size()) > buffer_policy_.lidar_catchup_frames) {
        drop.lidar += static_cast<int>(lidar_buffer_.size()) - 1;
        time_buffer_.erase(time_buffer_.begin(), time_buffer_.end() - 1);
        lidar_buffer_.erase(lidar_buffer_.begin(), lidar_buffer_.end() - 1);
    }
    if (drop.lidar > 0) {
        lidar_pushed_ = false;          // ★ 作废半状态
        measures_.scan_.reset();        // ★ 断开对已丢帧的引用（slam_v2 是 measures_.lidar_）
    }

    // ---- IMU 侧（下界跟随最旧保留的 LiDAR）----
    if (imu_buffer_.empty()) return drop;
    double t_floor;
    if (!time_buffer_.empty()) {
        t_floor = time_buffer_.front() - buffer_policy_.imu_keep_margin_sec;
    } else {
        t_floor = imu_buffer_.back()->timestamp - buffer_policy_.imu_orphan_keep_sec;
    }
    while (!imu_buffer_.empty() && imu_buffer_.front()->timestamp < t_floor) {
        imu_buffer_.pop_front();
        ++drop.imu;
    }
    while (static_cast<int>(imu_buffer_.size()) > buffer_policy_.imu_max_samples) {
        imu_buffer_.pop_front();
        ++drop.imu;
    }
    return drop;
}
```

> **IMU 时间戳字段差异**：`lio_odometry` 是 `IMU::timestamp`，`slam_odometry_v2` 是 `sensor_msgs::Imu::header.stamp.toSec()`。两份实现分别写，其余逐行一致。

### 4.6 日志（限频，复用已有 `SHOULD_LOG`）

两包 `.cc` 中已存在 `ThrottleByLine` / `SHOULD_LOG(rate)` 节流器，直接复用：

```cpp
const BufferDropCount dropped = PruneBuffers();
if (dropped.lidar > 0 || dropped.imu > 0) {
    dropped_lidar_frames_ += dropped.lidar;
    dropped_imu_samples_ += dropped.imu;
    if (SHOULD_LOG(1.0)) {
        SYS_LOG(syslog::LogSeverity::WARNING, true)
            << "[BUFFER] drop lidar=" << dropped.lidar
            << " (total=" << dropped_lidar_frames_ << ")"
            << " drop imu=" << dropped.imu
            << " (total=" << dropped_imu_samples_ << ")"
            << " buf_lidar=" << lidar_buffer_.size()
            << " buf_imu=" << imu_buffer_.size();
    }
}
```

> 丢弃是异常工况，用 `WARNING` 便于在日志里检索；限频 1 Hz，正常工况完全不打。

### 4.7 验收

| 用例 | 通过标准 |
|---|---|
| 正常回放 | 无 `[BUFFER] drop` 日志（或仅在启动瞬间） |
| 主循环 sleep 2 s ×3 次 | 每次恢复后 1 s 内 `rostopic delay` 回落到 ~100 ms；日志有 drop 记录 |
| 断 LiDAR 10 s | `imu_buffer_` 尺寸有界，内存平稳 |
| 断 IMU 10 s | `lidar_buffer_` 尺寸有界，无崩溃，恢复后正常出图 |
| 长跑 30 min | `VmRSS` 不再单调上涨 |

### 4.8 回退

`buffer_policy.enable: false` → `PruneBuffers()` 首行 return，等于不存在。
出问题则 `git revert` Stage 2 commit。

### 4.9 风险与注意

1. **④ 作废半状态**是关键正确性点：漏写会导致 IMU 时间窗错乱。单测覆盖"丢帧发生在 `lidar_pushed_==true` 时"。
2. **追帧阈值 3** 意味着正常缓冲不超过 2 帧。若实车希望更宽松（如容忍 400 ms 抖动），调大 `lidar_catchup_frames` 即可，无需改代码。
3. **丢帧会让输出频率下降**（例如处理耗时 200 ms → 实际 5 Hz 输出），这是**预期行为**：优先保证时间戳实时，而不是补齐历史帧。轨迹精度由 IMU 跨空档积分保证。
4. **不要在回调里裁剪**：回调不知道消费侧状态，且会延长回调耗时。统一在 `SyncPackages` 开头、持锁状态下裁剪。
5. `slam_odometry_v2` 的 `SyncPackages` 补加 `std::lock_guard<std::mutex> lock(mtx_buffer_);` —— 当前单线程下行为中性，但与 `lio_odometry` 对齐，消除隐患。**此改动写在同一 commit 里并单独列出**，便于出问题时定位。

---

## 5. yaml 汇总

### 5.1 `lio_odometry/config/lio_odometry.yaml` 新增（顶层）

```yaml
# ---------- Stage 1：订阅队列（条数；enable_override=false 恢复旧常量）----------
queue_size:            # 位于 lio_odometry: 段内（NodeConfig::LoadFromYAML 读取）
  enable_override: true
  imu: 200
  lidar: 10
  wheel: 100
  gnss: 50
  correction: 50

# ---------- Stage 2：缓冲区治理（enable=false 恢复无界缓冲）----------
buffer_policy:         # 顶层（LaserOdometryCore::LoadParamsFromYAML 读取）
  enable: true
  lidar_keep_sec: 0.8
  lidar_max_frames: 10
  lidar_catchup_frames: 3
  imu_keep_margin_sec: 0.2
  imu_max_samples: 3000
  imu_orphan_keep_sec: 2.0
```

### 5.2 `slam_odometry_v2/config/slam_odometry_v2.yaml` 新增

`queue_size` 放在**节点层**（rosparam 读取）；`buffer_policy` 放在**算法层**（`LoadParamsFromYAML` 直接读文件）：

```yaml
# -------------------- 节点层：订阅队列 --------------------
queue_size:
  enable_override: true
  imu: 200
  lidar: 10
  wheel: 100
  gnss: 50
  correction: 50

# -------------------- 算法层：缓冲区治理 --------------------
buffer_policy:
  enable: true
  lidar_keep_sec: 0.8
  lidar_max_frames: 10
  lidar_catchup_frames: 3
  imu_keep_margin_sec: 0.2
  imu_max_samples: 3000
  imu_orphan_keep_sec: 2.0
```

### 5.3 legacy 回退片段（粘回 yaml 即恢复旧行为，无需重编译）

```yaml
queue_size:
  enable_override: false
buffer_policy:
  enable: false
```

---

## 6. 提交切分

| # | Commit message | 文件范围 | 可独立 revert |
|---|---|---|---|
| 1 | `perf(odom): 订阅队列长度配置化并收敛（200000→400/8）` | 4 个 config 文件 + 2 个 yaml | ✅ |
| 2 | `perf(odom): 缓冲区时间裁剪+条数兜底+drop-to-latest 追帧` | 2 个 core 头文件 + 2 个 core .cc + 2 个 yaml | ✅ |

Stage 1 与 Stage 2 无依赖，可独立发布/回退。

---

## 7. 测试清单与实测结果

测试 bag：`/home/hhy/2004ros/data/2026-08-31-15-51-24.bag`
（6:16 s，`/ouster/imu` 100 Hz + `/ouster/points` 10 Hz；无轮速/GNSS 话题）
仅播 `/ouster/imu /ouster/points`。指标 **backlog = 最新输入点云时间戳 − 输出点云时间戳**（与播放倍速无关）。

| # | 用例 | 通过标准 | 结果 |
|---|---|---|---|
| T1 | 1x 全量回放（376 s） | 10 Hz、无 drop、无积压 | ✅ 输出 **10.00 Hz**（3751 帧），backlog 抖动 ±50 ms，`[BUFFER] drop` **0 次** |
| T2 | SIGSTOP 节点 8 s 后恢复 | 恢复快、drop 触发 | ✅ 恢复后 backlog 回到 ~0；`[BUFFER] drop imu=20` 触发（IMU 孤儿路径生效）；恢复后 drift 均值 29 ms（对照 86 ms） |
| T3 | 8x 过载（输入 80 Hz） | 积压有界、内存有界 | ✅ **见 §7.1 对照数据** |
| T4 | 长跑内存 | 不无界上涨 | ⚠️ 见 §7.2：iVox 地图增长仍在（Stage 3/4 范围） |
| T5 | legacy 回退片段 | 行为与改动前一致 | ✅ 8x 对照即以 `buffer_policy.enable: false` 复现旧行为 |
| T6 | `lidar_pushed_==true` 时丢帧 | 无空 IMU 组/假 gap | ✅ 8x 过载持续触发丢帧（total lidar=454 / imu=3119），无 IMU 组异常 |

### 7.1 8x 过载对照（核心验证）

同一 bag、同一 8 倍速（输入 80 Hz LiDAR，处理能力约 38~55 Hz），仅切换 `buffer_policy.enable`：

| 指标 | `enable: true`（修复后） | `enable: false`（旧行为） |
|---|---|---|
| backlog 时间序列 | **257~347 ms，有界** | **2581 → 8064 → 15587 → 23401 → 33057 → 42307 ms，单调增长** |
| backlog 峰值 | 700 ms | **45 298 ms（45 秒）** |
| `buf_lidar` / `buf_imu` | **1 / 31~41**（始终有界） | 无界 |
| `VmRSS` | 51 MB → **113 MB** | 53 MB → **735 MB** |
| `[BUFFER] drop` | 11 条（lidar 累计 454、imu 累计 3119） | 0 条 |

**结论**：旧行为精确复现了实测的两个症状——
- 积压单调增长到 45 秒 ⇒ 对外表现为"odom 系点云长时间卡顿/停住"；
- RSS 涨到 735 MB ⇒ 对应"内存占用越来越大"。
修复后二者同时被压住（积压 ~300 ms、RSS 113 MB）。

### 7.2 长跑内存（1x 全量 376 s）

| 指标 | 起始 | 结束 |
|---|---|---|
| `VmRSS` | 52 MB | 204 MB |
| `Map grid num` | 3 263 | 113 443 |

输出始终 10.00 Hz、零丢弃、零积压 ⇒ **本次改动未引入内存问题**。
RSS 的增长来自 iVox 地图本身（行驶中地图增长属预期），属**暂缓的 Stage 3/4**范围：
静止不插点、iVox 三轴淘汰（含 `capacity_` 从 100 万收敛）尚未实施。

### 7.3 `lio_odometry` 同套验证

同一 bag。输出话题 `/slam_odometry/odom`、`/slam_odometry/cloud_registered`。

| 用例 | 结果 |
|---|---|
| **T1** 1x 全量 376 s（原配置） | backlog **恒为 0**（3755 帧），`[BUFFER] drop` **0 次**，RSS 22.6 → 40.1 MB |
| **T2** SIGSTOP 8 s @30 s | 卡顿窗内 backlog_max 7901 ms，**SIGCONT 后 2~4 s 内回到 0**；`[BUFFER] drop lidar=0 drop imu=100 buf_lidar=0 buf_imu=109`（IMU 孤儿路径生效） |
| **T3** 8x 过载 + 开启建图（`use_gnss: false`） | 见下表 |

**T3 对照**（仅切换 `buffer_policy.enable`，且关闭 `use_gnss` 使 `MapIncremental` 每帧执行，见 §7.4）：

| 指标 | `enable: true` | `enable: false` |
|---|---|---|
| backlog 时间序列 | **82 → 250 ms，有界** | **127 → 425 → 2423 → 5575 → 9838 ms，单调增长** |
| backlog 峰值 | 500 ms | **12 001 ms** |
| `[BUFFER] drop` | 7 条（lidar 累计 149、imu 累计 951），`buf_lidar=1` | 0 条 |
| `VmRSS` | 22.8 → **87.3 MB** | 22.7 → **202.7 MB** |

结论与 `slam_odometry_v2` 一致：修复把无界积压压到 ~250 ms，内存峰值降低 57%。

### 7.4 验证过程中发现的既有问题（**不属于本次改动**）

1. **静止门控误判（暂缓的 Stage 3 范围）**
   `laser_odometry_core.cpp:524` 的判据是
   `speed_for_map = use_wheel_speed_ ? |wheel_speed_obs_| : |state.vel|`。
   `wheel_speed_obs_` 只在收到 CAN 时更新（`FeedWheelSpeed:950`），**无新鲜度判断**。
   本 bag 没有 `/can_receive_info` → `wheel_speed_obs_` 恒为 0 → 判定静止 →
   全程 3755 帧中有 **3726 帧跳过 `MapIncremental`**（stdout 打印 `--------skip mapincrement--------`）。
   **即：车在动、地图却不更新**，正是 §8 Stage 3 里预警的"误判静止"风险，且在缺轮速时**必现**而非少见。

2. **`Map grid num` 中途归零**
   T1 日志中 `Map grid num` 从 1788 突然变为 0 并保持。原因：
   该 bag 上估计器发散（`pitch≈50°`、位置持续外推、状态速度 ~19 m/s），
   `RemoveFarPoints(center, 50.0)`（`map_trim_radius: 50.0`）把远离漂移后位姿的体素全部剔除；
   由于问题 1 已冻结插点，地图无法重建 → 归零。
   属 **Stage 3（插点门控）+ Stage 4（淘汰半径）** 范围，与 Stage 1/2 无关。

3. **逐帧 `std::cout`（暂缓的 Stage 5 范围）**
   上述 `--------skip mapincrement--------` 是 `laser_odometry_core.cpp:612` 的逐帧 `std::cout`+`endl`，
   建图关闭期间每帧刷屏——与 §8 Stage 5 列的阻塞 IO 是同一处。

---

## 8. 暂缓项（后续入口）

### Stage 3：地图插入门控（暂缓）
- **原方案**：静止时不插点；含 `map_insert_min_speed_` 与 `use_gnss_` 解耦、`point_no_need_downsample` / 空近邻两条无条件插入通道收敛。
- **暂缓原因**：静止误判少见，但一旦**误判静止**（行驶时停止插点）会导致地图不更新 → 匹配失效 → 漂移，风险大于收益。简化策略（纯速度判据）在无轮速或状态速度不准时不可靠；多证据融合又与"保持简单、便于排查"冲突。
- **后续入口**：若实测静止内存仍增长且定位到插点通道，再按多证据（IMU 动态为主 + 位姿增量为辅 + 位移安全网）实施。
- **注意**：`lio_odometry` 现有静止门控绑定在 `use_gnss_` 上（`laser_odometry_core.cpp:611`），且 `wheel_speed_obs_` 无新鲜度判断（CAN 停发会冻结在最后值）—— 这两个是已知缺陷，留待 Stage 3 一并处理。

### Stage 4：iVox 三轴淘汰（暂缓）
- **原方案**：`Prune(center, radius, max_age)` 一次遍历做距离+年龄淘汰，`capacity_` 从 100 万收敛到 20~30 万。
- **暂缓原因**：应先堵住无效插入（Stage 3），否则淘汰与插入互相颠簸。当前 `RemoveFarPoints` 已提供距离淘汰，配合 Stage 2 后内存应已可控。
- **后续入口**：长跑内存仍超预期时实施。注意 `ivox3d.h` 在两包各有一份，改动必须同步。

### Stage 5：主循环阻塞 IO（暂缓）
- **原方案**：删除逐帧 `std::cout`（`laser_odometry_core.cpp:612`、`ros_adapter.cpp:111,152`）；每帧 5+ 条 SYS_LOG 加 `every_n_frames` 门控；local_map 发布降采样。
- **暂缓原因**：属卡顿诱因，不属内存/积压；改动分散且会降低可观测性。
- **后续入口**：Stage 1+2 落地后若仍有偶发卡顿，按此排查。

---

## 9. 实施状态

- [x] 本文档编写
- [x] Stage 1：`lio_odometry` 配置结构体 + 加载 + subscribe
- [x] Stage 1：`slam_odometry_v2` 配置结构体 + 加载 + subscribe
- [x] Stage 1：两包 yaml 新增 `queue_size`
- [x] Stage 2：`lio_odometry` `BufferPolicyConfig` + `PruneBuffers()` + 调用点
- [x] Stage 2：`slam_odometry_v2` 同上（含 `SyncPackages` 补锁）
- [x] Stage 2：两包 yaml 新增 `buffer_policy`
- [x] 编译通过（`catkin_make --pkg lio_odometry slam_odometry_v2`）
- [x] `slam_odometry_v2` T1/T2/T3 验收（bag `2026-08-31-15-51-24.bag`，结果见 §7.1~7.2）
- [x] `lio_odometry` T1/T2/T3 验收（同 bag，结果见 §7.3）
- [ ] 轨迹精度回归（ATE < 2 cm）—— 需参考轨迹，本次仅验证了实时性/内存，**未验证定位精度**
- [ ] Stage 3/4 后重测 lio 地图内存（当前被 §7.4 的误判静止掩盖）

### 已改动文件清单

| 包 | 文件 | Stage |
|---|---|---|
| lio_odometry | `include/lio_odometry/ros/config.h` | 1 |
| lio_odometry | `src/ros/config.cpp` | 1 |
| lio_odometry | `src/ros/ros_adapter.cpp` | 1 |
| lio_odometry | `include/lio_odometry/core/laser_odometry_core.h` | 2 |
| lio_odometry | `src/core/laser_odometry_core.cpp` | 2 |
| lio_odometry | `config/lio_odometry.yaml` | 1+2 |
| slam_odometry_v2 | `include/slam_odometry_v2/ros/slam_odometry_v2_config.hpp` | 1 |
| slam_odometry_v2 | `src/ros/slam_odometry_v2_config.cpp` | 1 |
| slam_odometry_v2 | `src/slam_odometry_v2_node.cc` | 1 |
| slam_odometry_v2 | `include/faster_lio/laser_odometry.h` | 2 |
| slam_odometry_v2 | `src/faster_lio/laser_odometry.cc` | 2 |
| slam_odometry_v2 | `config/slam_odometry_v2.yaml` | 1+2 |

### 备注

- `slam_odometry_v2_node.cc` 的 IMU/LiDAR 队列在本文档编写前已是 `200`/`10`（已提交状态），本次沿用该值作为两包统一默认，仅做配置化，**未改变其运行行为**。
- `PruneBuffers()` 在两包中逻辑逐行一致，仅 IMU 时间戳字段不同（`IMU::timestamp` vs `Imu::header.stamp`）与 `measures_` 字段名不同（`scan_` vs `lidar_`）。
