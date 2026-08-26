# global_matcher_v2

新一代全局点云匹配定位节点，基于 `global_matcher`（localization_v2 链路中使用）
重构：

- **地图管理**：`single`（一次加载整图）/ `chunked`（分片实时加载）两种模式；
- **匹配器**：icp / gicp / vgicp 统一基于 **small_gicp**（含 Huber / Cauchy
  鲁棒核），另提供 **fast_gicp 粗配准 + vgicp 精细配准**混合分支；
- **性能**：活动分片集合变化时才重建 target 结构（协方差点云 / KDTree /
  VGICP 体素地图），匹配帧内零 target 预处理；
- **精简**：调试话题全部收敛到 `publish.*` 开关，配置文件去除了冗余项。

> 详细匹配流程、初始位姿来源与参数说明见 [docs/matching_flow.md](docs/matching_flow.md)。

## 一、地图加载模式（多地图 vs 单图）

### 1.1 两种模式

| | `map.mode: single` | `map.mode: chunked` |
|---|---|---|
| 加载方式 | 启动时一次性加载整张 PCD | 解析清单，按车辆位置实时加载周边 `(2r+1)^2` 个分片 |
| 启动耗时 | 高（83MB 级 PCD 读取 + 降采样 + 协方差） | 低（只解析清单，首个分片 ~0.2MB） |
| 内存 | 整图常驻（GB 级原始 + 预构建结构） | 仅活动分片常驻（LRU 上限 `max_loaded_chunks`） |
| 运行期 I/O | 无 | 边界跨越时后台异步加载邻接分片 |
| 实现 | 整图等价于一个永不卸载的分片 | 同上 + 分片集合管理 |

### 1.2 多地图加载是否影响性能？

**结论：分片加载对匹配性能的影响可以忽略，而且对大图反而更优。**

- 匹配用的 target 结构（协方差 / KDTree / 体素地图）只在**活动分片集合变化**
  时重建一次（比如 `load_radius_chunks=1` 时是 3×3=9 个分片），匹配帧内只做
  source 降采样与配准，与单图模式完全一致；
- 分片加载在**后台线程**完成，只有"当前所在分片"未加载时会同步等待（首次进入
  某分片，通常 < 100ms）；邻接分片预加载不阻塞匹配；
- 内存占用可控（分片级 LRU），不会出现单图模式整图驻留；
- 需要注意的点：`load_radius_chunks` 至少取 1，保证跨分片边界时邻接分片已就位，
  避免"进入新分片才等加载"；分片缺失会跳过加载并告警（不阻塞匹配）。

## 二、分片构建脚本分析（`/home/hhy/2004ros/data/build_chunked_maps_from_keyframes.py`）

### 2.1 脚本是否完整？实时加载依据什么判断加载新地图？

脚本生成：

- `global.pcd`：整图；
- `index.txt`：`chunk_id gx gy rel_path`（首行为占位 `0 0 0`），`gx/gy` 为分片
  网格坐标，约定 `grid = floor(x / chunk_size + 0.5)`；
- `N.pcd`：各分片点云（已按 `pcd_voxel_size` 降采样）。

实时加载的判定条件 = **车辆当前位置的分片网格坐标 + 加载半径**：

```text
当前分片 (gx, gy) = floor(pose.x / chunk_size + 0.5), floor(pose.y / chunk_size + 0.5)
需加载分片 = {(gx', gy') : |gx'-gx| <= load_radius_chunks 且 |gy'-gy| <= load_radius_chunks}
```

这部分**脚本已生成**（grid 坐标在 `index.txt` 里）。但原脚本有几点不完整：

1. `index.txt` 首行 `0 0 0` 是占位符，没有说明性头部；
2. 没有分片边界（min/max）与点数元数据，运行时只能按网格约定推算；
3. `rel_path` 带 `./data//` 前缀且是双斜杠，路径约定不透明。

### 2.2 已做的优化

在保持 `index.txt` 完全不变（lightning-lm 的 `tiled_map.cc` 仍在消费它）的前提下，
新增 `chunks.txt` 清单：

```text
# global_matcher_v2 chunk manifest
chunk_size 100.000000
chunk <id> <gx> <gy> <min_x> <min_y> <min_z> <max_x> <max_y> <max_z> <points> <path>
```

- 包含 `chunk_size`、每个分片的**真实边界**与点数，运行时不再依赖网格推算；
- 路径为相对 `map_dir` 的简单文件名；
- `global_matcher_v2` 优先读 `chunks.txt`，缺失时自动回退解析旧 `index.txt`
  （跳过首行、按网格推算边界）。

脚本支持任务开关，PCD 分片与 PGM 栅格可单独构建（默认两者都构建）：

```bash
# 只构建分片 PCD 地图（PGM 不需要 z 轴滤波行为）
python3 build_chunked_maps_from_keyframes.py --build-pcd
# 只构建 PGM 栅格地图
python3 build_chunked_maps_from_keyframes.py --build-pgm
```

单独运行时只清理本任务的产物，不会覆盖另一个任务已生成的文件。

## 三、匹配器（small_gicp 统一 + 混合分支）

`matcher.type`：

| type | 粗配准 | 精细配准 | 说明 |
|---|---|---|---|
| `icp` | small_gicp ICP | small_gicp ICP | 距离最近邻因子 |
| `gicp` | small_gicp GICP | small_gicp GICP | 点对分布因子 |
| `vgicp` | small_gicp VGICP（预构建体素） | small_gicp VGICP（预构建体素） | target 为 GaussianVoxelMap |
| `fast_gicp` | fast_gicp | fast_gicp | 原 global_matcher 行为（默认，最稳） |
| `fast_gicp_vgicp` | **fast_gicp** | **small_gicp VGICP** | 混合分支：宽收敛域粗配准 + 精细体素精配准 |

要点：

- **鲁棒核**：`matcher.robust_kernel`（`none`/`huber`/`cauchy`）作用于 small_gicp
  的 gicp / vgicp / icp 因子（`RobustFactor` 包装），抑制动态物体 / 植被等离群
  对应点对配准的拉偏；fast_gicp 类型不适用；
- **VGICP 预构建体素**：`map.mode=chunked` 时按活动分片集合预构建 coarse/fine
  两张 GaussianVoxelMap（协方差在分片加载时一次性估计），匹配帧内不再
  做 target 协方差估计与体素化——这是相对 v1（每帧从子图现建两张体素图）
  最大的性能提升；
- **quality gate**：`max_fitness_score` 仅对 `fast_gicp` 生效（PCL fitness
  语义）；small_gicp 的 `error` 量纲不同，统一以内点率 / RMSE / 位移偏航门控，
  与 v1 对 vgicp 的处理一致。

## 四、配置冗余分析（相对 v1）

### 删除 / 收敛

| v1 配置 | v2 处理 | 原因 |
|---|---|---|
| `map/segments`、`map/default_map_id` | 删除 | 由 `map.mode=chunked` + 清单替代 |
| `runtime/viz_test_mode` | 删除 | 由 `publish.*` 开关替代 |
| 注释掉的第二段 vgicp matcher 配置 | 删除 | 重复配置 |
| `matcher/fast_gicp_*`、`matcher/vgicp_*` 平铺 | 收敛到 `matcher.*` | 分组清晰 |
| `topics/global_map_publish_voxel_leaf` 等 | 移到 `map.*` | 属于地图管理 |
| `map/publish_voxel_leaf` | 更名 `map/global_map_publish_voxel_leaf` | 语义明确 |

### 保留但说明

- `frames/lidar` + `extrinsics/base_to_lidar`：仅当 source 点云位于 lidar 系时使用；
- `crop.*`：v2 中只用于 `publish.target_submap` 的可视化裁剪（匹配 target 用
  活动分片全量 + KDTree/体素，不再每帧裁剪），保持配置以兼容旧可视化；
- `runtime/use_receive_time_for_cloud_age`：bag 测试鲁棒性，保留。

### 发布话题量

`publish.*` 开关（默认仅 `global_map` / `path` / `status` 开）：

```yaml
publish:
  global_map: true        # latch，仅有订阅者时发送
  target_submap: false    # 调试：裁剪子图
  aligned_scan: false     # 调试：配准后点云
  source_initial_scan: false  # 调试：初值变换点云
  initial_guess: false    # 调试：初值位姿
  path: true              # 轨迹
  status: true            # 状态
```

## 五、运行

```bash
# single 模式（等价于 v1）
roslaunch global_matcher_v2 global_matcher_v2.launch \
  map_mode:=single pcd_path:=/path/to/map.pcd

# chunked 模式（分片目录由 build_chunked_maps_from_keyframes.py 生成）
roslaunch global_matcher_v2 global_matcher_v2.launch \
  map_mode:=chunked map_dir:=/path/to/chunked_map

# 直接替换 localization_v2 链路中的 global_matcher
roslaunch global_matcher_v2 localization_v2_with_gmv2.launch
```

## 六、目录结构

```text
global_matcher_v2/
├── CMakeLists.txt / package.xml
├── config/global_matcher_v2.yaml
├── launch/
│   ├── global_matcher_v2.launch          # 独立运行
│   └── localization_v2_with_gmv2.launch  # 替换 localization_v2 链路中的 GM
├── include/global_matcher_v2/
│   ├── core/
│   │   ├── map_manager.hpp/.cpp   # 地图管理（single/chunked、异步加载、预构建）
│   │   ├── matcher.hpp/.cpp       # 匹配器（small_gicp + fast_gicp_vgicp）
│   │   └── matcher_impl.hpp       # 匹配器模板实现（鲁棒核因子注入）
│   └── ros/
│       └── global_matcher_v2_config.hpp/.cpp
└── src/ros/global_matcher_v2_node.cpp   # 节点（流程与 v1 一致）
```

## 七、已知限制与后续

- `gicp/icp` 的 target 使用活动分片全量（不再按 scale 每帧降采样），配准精度
  取决于 `map.load_voxel_leaf`（默认 0.4m）；如需更细可降低该值并配合分片；
- `fast_gicp_vgicp` 分支的细配准精度受 `vgicp_voxel_resolution`（fine 体素）
  限制，建议 0.5m 起步，按数据实测调整；
- `load_map_command` 在 chunked 模式下为 no-op（分片自动管理），single 模式下
  支持热替换整图。
