# 启动垂直漂移修复与验证

本次修改针对纯 LiDAR + IMU 路径，保留实际设备外参。后续按原版 default_vbr.yaml 对齐了采样、迭代次数与噪声参数。

## 已修复

- `math::exp` 恢复 MTK 四元数半角约定，返回 SO3；旋转向量应传 scale=0.5。去畸变使用 `SO3::exp(omega * dt)`，避免双倍旋转。`A_matrix` 小角度阈值恢复 1e-5。
- S2 数值更新与雅可比一致；后验协方差用实际更新前后的切空间变换重置，并对称化。
- 显式存储观测模型的旋转矩阵和平移向量，避免 Eigen 惰性表达式引用临时对象。
- IMU 初始化至少 0.5 秒且至少 20 样本；检查角速度和加速度波动，输出初始化重力与偏置。传播起点使用状态真实时间，拒绝有明显 IMU 断流的扫描，空 IMU 不再复用旧点云。
- 扫描起点与 IMU 区间边界重合时也进行去畸变；首点只补偿一次。
- 近邻查询先清空缓存并检查返回值。并行筛选使用独立字节而非 vector<bool>；带 mutex 的地图循环使用 par 而非 par_unseq。
- 平面拟合使用中心化协方差，支持通过原点的平面，拒绝共线邻居。
- 激光更新失败恢复预测状态和协方差，并禁止该帧入图；默认至少 20 个有效点、5% 有效比例。此门槛不是完整退化检测。
- 点云入队保留原始点顺序，按原版最后一个保留点的时间及历史扫描时长回退同步。全部点时间为零时不拒绝，保留扫描几何。时间回跳重置整个里程计和地图；移除激光间隔超过 0.5 秒后永久跳过的分支。
- AA 保持关闭；启用时残差恶化拒绝整次更新，避免错误回退。修复其历史缓存循环下标。

## 编译与回归

在仓库根目录执行：

```bash
cmake --build build --target lio_odometry_node lio_odometry_regression -j2
(cd build && ctest -R '^lio_odometry_regression$' --output-on-failure)
```

测试覆盖 S2 往返、有限差分雅可比、半角/负角/小角、原点平面和共线点、约 10 秒静止 IMU、旋转去畸变、后验协方差对称与半正定、失败更新回滚、原版扫描结束时间回退、Ouster 字段转换及全零点时间、34 帧静止匹配、激光断流恢复、失配拒绝和时间回跳。全部通过。合成静止匹配位置为 `(0,0,0)`，不能作为真实数据漂移指标。

## 实机复测

使用刚编译的工作空间：

```bash
source devel/setup.bash
roslaunch lio_odometry lio_odometry.launch
```

启动时保持静止至少 1 秒，等待 `[ImuProcess] initialized`，再开始运动。确认实际输入使用正确的点时间字段（Ouster `t` 为相对扫描起点的纳秒），点云 header 和 IMU 为同一时钟。全零点时间扫描现在按原版兼容处理，不会被拒绝；这不代表能够恢复缺失的逐点采样时间。

实际 LiDAR 到 IMU 外参仍为 `fasterlio.extrinsic_R/T`；没有依据设备标定擅自修改。初始化静止判定不能识别所有恒定线加速度。在线外参估计保持原配置，若设备已有可靠标定，可另做关闭外参估计的对照。

目前没有真实 rosbag 回放结果，无法保证已消除现场全部上漂。若仍漂移，保留初始化输出和 `[odometry] ... effect num` / `LiDAR update rejected` 日志，并检查标定、时钟及数据单位。

## 2026-09-20 原始工程对照复核

基准：`/home/hhy/2004ros/lm_slam_rosori/src/lightning-lm-ros1`。
用户已确认回放仅有 LiDAR/IMU；没有将 wheel/GNSS 配置开关视为本次差异原因。

确认并修正的输入行为差异：

- 上一轮新增的 `missing per-point timestamps; scan rejected` 不在原版中，是本次连续拒帧的直接原因，已删除。
- 原版 `Oust64Handler` 和移植版均按 `uint32 t / 1e6` 转成毫秒；未发现该路径单位换算或字段类型不一致。
- 原版内部 PointXYZIT 注册 `(double, time, timestamp)`；移植版曾注册为 `time`，现已对齐。该差异涉及内部格式/发布格式，不是 Ouster `t` 为零的直接解释。
- 原版扫描时间使用最后一个保留点；短于历史均值一半或点数不足时回退均值。历史一直为零时同步至 header 时间。已恢复该行为，不使用人为指定的扫描周期。
- 全零时间时跳过逐点补偿，避免上一轮边界修复对所有零偏移点产生额外补偿。
- 对齐 `point_filter_num=1`、`max_iteration=6`、`filter_size_scan=0.2`、`filter_size_map=0.5`、`acc_cov=0.1`、`b_acc_cov=b_gyr_cov=0.0001`。AA 保留 false，wheel/GNSS 保留 false。

状态对照：原版重力长度也是 9.809，初始化也是负平均加速度，状态排列和 LiDAR 雅可比列顺序相同，外参行列填充一致。没有发现移植版额外翻转 Z 轴或重力符号的证据。原版 math::exp 确为四元数半角约定，上一轮对此修复有依据。

必须区分来源：AA 回退、vector<bool> 并行写、Eigen 临时表达式和近邻缓存问题也存在于原始代码，不能把它们单独当成“原版正常而移植版上漂”的确定解释。此前报告未作这个区分。

本轮仅对齐输入链和上述参数，不声称全算法逐行等价。上一轮的静止初始化门槛、协方差重置、平面拟合和失败入图保护仍在；原版去畸变还直接使用半角 exp(omega,dt)，本地保留已用旋转测试验证的 SO3::exp(omega*dt)。没有回放真实 bag，垂直漂移根因尚未获得逐帧状态证据，合成静止测试不能证明现场问题已解决。

本轮构建实际输出为仓库根目录 `devel/lib/lio_odometry/lio_odometry_node` 和 `devel/lib/liblio_odometry_core.so`。请使用对应 `devel/setup.bash`，避免启动旧的 `build/devel` 产物。
