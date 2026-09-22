# VoxelMap LIO 优化实施记录（2026-09-20）

## 范围与结论

修改对象：`src/localization/lio_odometry_Voxlmap`，ROS 包名 `lio_odometry_voxelmap`。
参考代码：`/home/hhy/2004ros/r-voxelmap_ws/src/R-VoxelMap`。
本次修改实际编译的 `voxel_map_fastlivo.cpp`，没有修改不参与当前构建的备用 `voxel_map_manager.cc` 和 `voxel_map_octree.h`。

已修复确定的移植/实现问题，接通概率观测权重，并通过构建和专项回归。**尚无真实 bag 精度对照结果，不能声称达到或超过 R-VoxelMap。混合平面和退化场景仍有误差，见验证与限制。**

## 1. 配置控制

`config/lio_odometry.yaml` 中的开关会真正传入算法，AA 不再被硬编码为 true。

| 配置 | 本次默认值 | 用途 |
|---|---:|---|
| `fasterlio.use_aa` | false | false 普通 IEKF，true Anderson acceleration |
| `fasterlio.use_adaptive_r` | true | 使用每点平面与测距不确定性；false 回退到 `lidar_update_r` |
| `fasterlio.use_pose_convergence` | false | false 全状态收敛，true 只检查位姿 |
| `fasterlio.converged_times_required` | 1 | 连续满足收敛条件的次数，达到迭代上限仍结束 |
| `fasterlio.rotation_convergence_deg` | 0.01 | pose 模式旋转阈值，度 |
| `fasterlio.position_convergence_m` | 0.00015 | pose 模式平移阈值，米 |
| `fasterlio.min_effective_points` | 20 | 有效匹配数量门槛 |
| `fasterlio.min_effective_ratio` | 0.05 | 有效匹配占降采样点比例门槛 |
| `voxel_map.neighbor_search_radius` | 1 | 搜索邻根体素半径，允许 0、1、2 |
| `voxel_map.neighbor_search_on_failure` | true | 当前体素匹配失败才搜索邻居；false 全邻域比较似然 |
| `voxel_map.matching_threads` | 4 | OpenMP 匹配线程数 |
| `voxel_map.sample_seed` | 42 | RANSAC 原有随机种子现在从 YAML 读取 |

启动输出 AA、自适应 R、收敛模式、连续次数、邻域半径及线程数。加入了层数/每层点数、重初始化阈值序列、噪声和匹配参数的基本范围检查。
保留已有体素大小、最大深度、RANSAC 次数及外参，避免在数学修复时混入大范围场景调参。

## 2. 统一点协方差传播

新增 `include/lio_odometry/core/voxelmap/point_covariance.h`，首帧建图、观测匹配和后验入图共同使用。

本地状态是 `[position, rotation, extrinsic_rotation, extrinsic_translation, velocity, ...]`，与 R-VoxelMap 的 `[rotation, position, ...]` 不同。原代码错误地把 P(0:3,0:3) 用作姿态方差，现通过统一雅可比消除索引错用。

对于 `p_i = R_il p_l + t_il`：

```text
R_wl = R_wi R_il
J_pos = I
J_rot = -R_wi hat(p_i)
J_ext_rot = -R_wl hat(p_l)  # 仅估计外参时
J_ext_trans = R_wi         # 仅估计外参时
Sigma_world = R_wl Sigma_lidar R_wl^T + J P J^T
```

保留状态交叉协方差，修复原先传感器协方差遗漏 LiDAR→IMU 旋转。外参固定时不传播外参估计块。每次 IEKF 更新保存先验 P 快照，匹配不再读取上一轮留在 P_ 中的中间矩阵；后验入图使用最终 P。

CalcBodyCov 改为径向/切向投影公式，避免 z=0 时提前返回未赋值矩阵；不再为了避开除零而修改点的 z 坐标。

## 3. 概率观测与滤波

原先计算 sigma_l/sigma_p 后把 R 赋值注释掉，已恢复为：

```text
sigma_plane = J_nq Sigma_plane J_nq^T
sigma_point = n^T R_wl Sigma_lidar R_wl^T n
R_i = r_base + sigma_plane + sigma_point
```

匹配 3σ 门控用包含状态不确定性的总方差；观测噪声用传感器与地图平面方差，避免再次把当前状态先验计入测量噪声。

LiDAR 方差只保存 N 个标量，白化 H 与 residual 后累计信息矩阵，通过 LDLT 求解，不再分配 N×N 稠密 R。白化存储约为 N×12，求解矩阵为 23×23。wheel/GNSS 原有小矩阵接口保留。

修复外参旋转雅可比缺少外参旋转及符号不一致。连续收敛次数现在真实参与结束判断；原先只要收敛一次就结束的问题已修复。

ESKF::Update 返回 bool；无效匹配、非有限更新等恢复预测 x/P。AA 残差恶化拒绝整次 LiDAR 更新；实际 AA 状态增量用于切空间协方差重置。修复 AA 历史缓存下标取模维度。

更新失败不入图、不作为成功里程计输出。首次姿态保持分支也同时回滚 x/P 并跳过入图。有限匹配数量门槛不等于完整的退化检测。

## 4. 数学修复

`math::exp` 恢复四元数半角约定；S2 和协方差中原有 scale=0.5 调用恢复正确语义。去畸变明确用 `SO3::exp(omega*dt)`，不会因半角语义恢复而把旋转量翻倍。A_matrix 小角度阈值为 1e-5。
本次没有把另一个 lio_odometry 包的所有 IMU 初始化/同步改动复制过来，尤其没有引入“全零点时间拒帧”。

## 5. 平面与地图

- 连通域剔点后先将 center 乘回旧点数，再减剔除点并除新点数，修复明确的移植遗漏。
- 点数不足、特征值间隔过小、非有限平面方差等路径增加基本数值保护，避免共线点法向方差除零。
- reinit 先检查索引再访问阈值；支持一批点跨过全部阈值及空阈值列表。
- `update_size_threshold` 原来没有初始化/传给节点；现在根和新建子节点都继承配置。
- EVD 模式初始化后正确转移到 plane/not_plane 点集合，避免统计量和点存储不一致。
- 根体素键、滑窗和连通域网格用 floor，修复负整数边界偏移。
- 当前根不存在也能搜索邻根，统一使用整数格坐标；默认当前格优先，邻格内选择最高似然。
- OpenMP 匹配使用独立字节标记和每线程点副本，移除 const_cast；批量更新命中体素 LRU，避免每点加锁。
- 外层与管理器的重复滑窗触发收敛为管理器单处控制。

## 6. 构建和验证

在仓库根目录：

```bash
cmake --build build --target lio_odometry_voxelmap_node lio_voxelmap_optimization_regression -j2
(cd build && ctest -R '^lio_voxelmap_optimization_regression$' --output-on-failure)
```

最终结果：节点与测试目标构建成功；CTest 1/1 通过（约 0.25 秒，为本机合成测试耗时，不代表实机帧处理性能）。
测试源码：`tests/optimization_regression.cpp`。

覆盖：

- 使用非单位外参、非对角状态 P 与有限差分验证完整点协方差传播。
- z=0 和原点的测量方差赋值、有限性与半正定性。
- S2 更新往返一致性。
- 不同方差的冲突高度观测按理论权重更新；AA 两种模式均验证。
- 连续收敛次数与失败回滚。
- 连通域移除孤立点后，剩余 100 点平面中心为 `(1.18,1.18,2)`。
- 跨过最后 reinit 阈值后继续更新不越界。
- 当前根缺失时邻格搜索，以及负整数格边界。
- 从临时 YAML 分别设置 AA=false/true，读取实际 ESKF 开关确认生效。
- 三独立平面静止 LiDAR+IMU 流程，连续 24 帧接受，分离体素场景最大位置偏移显示为 0.000000000 m。

### 保留的失败/限制证据

不能用上述 PASS 声称“所有静止场景都无漂移”。混合平面诊断用例只断言有限状态和足够接受帧，**没有把精度阈值算作通过条件**。

三独立平面、其中墙地混合进入根体素的场景：24 帧接受，最大位置偏移约 **0.067030843 m**，最后位置约 `(-0.000431242,-0.033859177,0.008138699)`。
最初仅两个平面的退化用例沿交线方向缺少约束，当前格优先策略下曾累计约 2.18 m 位置偏移；全邻域最高似然策略也出现约 0.61 m 偏移。不能将两种数值作为真实 bag 性能比较，二者都说明退化方向和混合平面仍需处理。

因此后续精度工作应针对：

1. 混合平面时的 RANSAC 支持点范围、平面合并/细分和跨层关联，避免把不同结构拟合成同一平面。
2. 对位姿观测信息矩阵做尺度归一化后的退化诊断；在弱约束方向保留 IMU 先验，不能只用有效点数判断可靠性。
3. 同一真实 bag 固定初始段、回放速率，先对比 adaptive_R=true/false，再单独比较深度 2/3/4、采样尺度；记录轨迹、有效点率、残差、处理时间和内存。
4. max_root_points_size 当前仍为 5000000，未实测长期内存；日志频率、RANSAC 随机数全局状态及统计点数的淘汰一致性仍可后续优化。

## 7. 使用

```bash
source /home/hhy/2004ros/DeepLumin_zq/devel/setup.bash
roslaunch lio_odometry_voxelmap lio_odometry.launch
```

AA 默认关闭，但可在 YAML 设置 true，代码不会覆盖它。改配置后重启节点。
本次未修改原始 R-VoxelMap、未提交 git commit、未回放用户实车 bag。实际精度改善幅度仍待真实数据确认。
