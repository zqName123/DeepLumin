# slam_mapping_node 设计文档

**路径**: `DeepLumin/src/localization/slam_mapping`

> 注：不是继续把功能塞进 slam_odometry。slam_odometry 保持"里程计核心"，slam_mapping 负责"建图任务、关键帧、地图保存、可视化、后续回环"。

---

## 1. 目标

slam_mapping_node 需要同时支持两种模式：

### 1.1 单独运行模式

通过 launch 启动后自动开始建图，或通过 `/slam_mapping/start`、`/slam_mapping/stop`、`/slam_mapping/save` 手动控制。

### 1.2 mapping_manager 托管模式

mapping_manager 接收 `/mapping/start`、`/mapping/stop`，再调用 slam_mapping_node 的底层服务启动、结束、保存地图。

### 1.3 首版功能范围

- 使用 slam_odometry 作为核心 LIO/SLAM 里程计
- 按距离/角度保存 keyframe
- 用 keyframe pose 拼接保存 PCD 地图
- 发布轨迹、当前点云、累计地图、关键帧、回环 marker
- 预留回环检测和位姿图优化接口，后续接 FASTLIO2_SAM_LC_ws 的 ICP loop closure + GTSAM pose graph

---

## 2. 总体架构

```
slam_mapping_node
├── RosAdapter
│   ├── 订阅 IMU / LiDAR / CAN / GNSS
│   ├── 发布 odom / path / map / markers / status
│   └── 提供 start / stop / save / reset / status 服务
│
├── MappingRuntime
│   ├── 状态机：IDLE / RUNNING / SAVING / FINISHED / FAILED
│   ├── 调用 slam_odometry::ISlamOdometry
│   ├── 驱动 keyframe、map saver、visualization
│   └── 管理 task_id、output_dir、统计信息
│
├── SlamBackend
│   └── 使用 FasterLIOImpl / ISlamOdometry
│
├── KeyframeManager
│   ├── 按距离/角度/时间保存关键帧
│   └── 保存 cloud_body + pose_odom + pose_map
│
├── MapSaver
│   ├── 按 keyframe pose 拼接地图
│   ├── 保存 PCD
│   ├── 保存 keyframe 点云
│   ├── 保存 keyframe pose
│   └── 保存 trajectory
│
├── LoopClosureBackend
│   ├── 首版空实现
│   └── 后续接 FASTLIO2_SAM_LC 的 ICP + GTSAM
│
└── VisualizationPublisher
    ├── path
    ├── keyframe markers
    ├── loop markers
    ├── local map
    └── accumulated preview map
```

---

## 3. 推荐目录结构

```
DeepLumin/src/localization/slam_mapping/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── slam_mapping.yaml
├── launch/
│   ├── slam_mapping.launch
│   └── slam_mapping_manager_backend.launch
├── include/slam_mapping/
│   ├── common/
│   │   ├── types.hpp
│   │   └── config.hpp
│   ├── core/
│   │   ├── mapping_runtime.hpp
│   │   ├── keyframe_manager.hpp
│   │   ├── map_saver.hpp
│   │   ├── loop_closure_backend.hpp
│   │   └── visualization_data.hpp
│   └── ros/
│       ├── ros_adapter.hpp
│       ├── param_loader.hpp
│       ├── ros_converters.hpp
│       └── visualization_publisher.hpp
└── src/
    ├── slam_mapping_node.cpp
    ├── core/
    │   ├── mapping_runtime.cpp
    │   ├── keyframe_manager.cpp
    │   ├── map_saver.cpp
    │   └── loop_closure_backend.cpp
    └── ros/
        ├── ros_adapter.cpp
        ├── param_loader.cpp
        ├── ros_converters.cpp
        └── visualization_publisher.cpp
```

---

## 4. 新增服务建议

不要让 slam_mapping_node 占用 `/mapping/start`、`/mapping/stop`。这些属于 mapping_manager。

建议 slam_mapping_node 提供：

| 服务名 | 说明 |
|--------|------|
| `/slam_mapping/start` | 启动建图 |
| `/slam_mapping/stop` | 停止建图 |
| `/slam_mapping/save` | 保存地图 |
| `/slam_mapping/reset` | 重置 |
| `/slam_mapping/get_status` | 获取状态 |

可以新增内部服务，放在 `DeepLumin/src/deeplumin_msgs/srv/localization/`：

### 4.1 StartMappingRuntime.srv

```yaml
string task_id
string task_name
string config_path
string output_dir
bool enable_loop_closure
bool enable_visualization
bool reset_before_start
---
bool accepted
string state
string reason
```

### 4.2 StopMappingRuntime.srv

```yaml
string task_id
bool save_map
bool save_keyframes
bool save_keyframe_poses
bool save_trajectory
---
bool accepted
string state
string reason
string map_path
string output_dir
```

### 4.3 SaveMappingRuntime.srv

```yaml
string task_id
string output_dir
string map_name
float64 resolution
bool save_keyframes
bool save_keyframe_poses
bool save_trajectory
---
bool success
string message
string map_path
string keyframe_dir
string keyframe_pose_path
string trajectory_path
```

### 4.4 GetMappingRuntimeStatus.srv

```yaml
---
deeplumin_msgs/MappingStatus status
```

---

## 5. 配置文件

`config/slam_mapping.yaml`：

```yaml
mapping:
  auto_start: false
  task_name: "manual_mapping"
  output_root: "/tmp/deeplumin_mapping"
  publish_tf: true
  process_rate_hz: 5000.0

frames:
  map: "map"
  odom: "odom"
  base_link: "base_link"
  lidar: "lidar_link"
  imu: "imu_link"

sensors:
  imu_topic: "/ouster/imu"
  lidar_topic: "/ouster/points"
  can_topic: "/can_receive_info"
  gnss_topic: "/gnss_chc_data"

slam_backend:
  type: "faster_lio"
  config_namespace: "slam/faster_lio"

keyframe:
  distance_thresh: 1.5
  rotation_thresh_rad: 0.2
  time_thresh_sec: 0.0
  min_points: 50

map_saver:
  default_resolution: 0.2
  save_on_stop: true
  save_keyframes: true
  save_keyframe_poses: true
  save_trajectory: true
  map_filename: "map.pcd"
  trajectory_filename: "trajectory.csv"
  keyframe_pose_filename: "keyframes.csv"
  keyframe_dir_name: "keyframes"

loop_closure:
  enable: false
  search_radius: 15.0
  min_index_gap: 20
  submap_search_num: 25
  submap_resolution: 0.3
  icp_fitness_thresh: 0.3
  pose_graph_enable: false

visualization:
  publish_path: true
  publish_keyframes: true
  publish_local_map: true
  publish_current_scan: true
  publish_accumulated_map_preview: true
  publish_loop_markers: true
  accumulated_map_publish_period: 2.0
  accumulated_map_voxel_leaf: 0.5

outputs:
  odom_topic: "/slam_mapping/odom"
  path_topic: "/slam_mapping/path"
  current_scan_topic: "/slam_mapping/current_scan"
  local_map_topic: "/slam_mapping/local_map"
  map_preview_topic: "/slam_mapping/map_preview"
  keyframe_markers_topic: "/slam_mapping/keyframes"
  loop_markers_topic: "/slam_mapping/loop_markers"
  status_topic: "/slam_mapping/status"
```

---

## 6. 核心数据结构

`include/slam_mapping/common/types.hpp`

### 6.1 RuntimeState

```cpp
namespace slam_mapping {

enum class RuntimeState {
  IDLE,
  READY,
  RUNNING,
  STOPPING,
  SAVING,
  FINISHED,
  FAILED
};
```

### 6.2 MappingKeyframe

```cpp
struct MappingKeyframe {
  int id = -1;
  double timestamp = 0.0;

  localization::Pose pose_odom;
  localization::Pose pose_map;
  
  localization::PointCloudPtr cloud_body;
  std::size_t point_count = 0;
};
```

### 6.3 MappingRuntimeStatus

```cpp
struct MappingRuntimeStatus {
  std::string task_id;
  std::string task_name;
  RuntimeState state = RuntimeState::IDLE;
  std::string message;

  bool running = false;
  bool saving = false;
  bool output_valid = false;
  bool loop_enabled = false;
  bool loop_supported = false;
  
  std::string output_dir;
  std::string map_path;
  std::string keyframe_dir;
  std::string keyframe_pose_path;
  std::string trajectory_path;
  std::string loop_result_path;
  
  std::uint64_t frame_count = 0;
  std::uint64_t keyframe_count = 0;
  std::uint64_t loop_count = 0;
  std::uint64_t accumulated_point_count = 0;
  
  double start_time = 0.0;
  double last_frame_time = 0.0;
  double duration_sec = 0.0;
};
```

### 6.4 RuntimeStartRequest

```cpp
struct RuntimeStartRequest {
  std::string task_id;
  std::string task_name;
  std::string output_dir;
  std::string config_path;
  bool enable_loop_closure = false;
  bool enable_visualization = true;
  bool reset_before_start = true;
};
```

### 6.5 RuntimeStopRequest

```cpp
struct RuntimeStopRequest {
  std::string task_id;
  bool save_map = true;
  bool save_keyframes = true;
  bool save_keyframe_poses = true;
  bool save_trajectory = true;
};
```

### 6.6 SaveRequest

```cpp
struct SaveRequest {
  std::string output_dir;
  std::string map_name = "map.pcd";
  double resolution = 0.2;
  bool save_keyframes = true;
  bool save_keyframe_poses = true;
  bool save_trajectory = true;
};

} // namespace slam_mapping
```

---

## 7. MappingRuntime

`include/slam_mapping/core/mapping_runtime.hpp`

### 7.1 职责

- 持有 ISlamOdometry
- 管理建图状态机
- 接收 IMU/LiDAR/CAN/GNSS
- 每帧调用 slam_->processOnce()
- 获取 odom/current_scan/local_map
- 驱动关键帧保存
- 驱动回环模块
- 驱动保存地图

### 7.2 核心接口

```cpp
class MappingRuntime {
public:
  bool initialize(const MappingConfig& cfg,
                  const localization::SlamConfig& slam_cfg);

  bool start(const RuntimeStartRequest& req);
  bool stop(const RuntimeStopRequest& req);
  bool save(const SaveRequest& req);
  void reset();
  
  void feedImu(const localization::ImuData& imu);
  void feedLidar(const localization::TimestampedPointCloud& cloud);
  void feedCanVelocity(const localization::CanVelocityData& can);
  void feedGnssAttitude(const localization::GnssAttitudeData& gnss);
  
  bool processOnce();
  
  MappingRuntimeStatus status() const;
  std::vector<MappingKeyframe> keyframes() const;
  
  localization::OdomResult currentOdom() const;
  localization::TimestampedPointCloud currentScan() const;
  localization::TimestampedPointCloud localMap() const;

private:
  bool handleFrame(const localization::OdomResult& odom,
                   const localization::TimestampedPointCloud& scan,
                   const localization::TimestampedPointCloud& local_map);

  void updateStatusOnFrame(const localization::OdomResult& odom,
                           const localization::TimestampedPointCloud& scan);
};
```

### 7.3 processOnce() 伪代码

```cpp
if state != RUNNING:
  return false

if !slam_->processOnce():
  return false

odom = slam_->getOdometry()
scan = slam_->getCurrentScan()
local_map = slam_->getLocalMap()

if !odom.valid or scan empty:
  return false

handleFrame(odom, scan, local_map)
return true
```

---

## 8. KeyframeManager

`include/slam_mapping/core/keyframe_manager.hpp`

### 8.1 职责

- 根据距离、角度、时间决定是否插入 keyframe
- 保存 body 系去畸变点云
- 维护 pose_odom 和 pose_map
- 首版 pose_map = pose_odom
- 后续回环优化后更新 pose_map

### 8.2 接口

```cpp
class KeyframeManager {
public:
  void configure(const KeyframeConfig& cfg);
  void reset();

  bool shouldAdd(const localization::OdomResult& odom,
                 const localization::TimestampedPointCloud& scan) const;
  
  bool add(const localization::OdomResult& odom,
           const localization::TimestampedPointCloud& scan);
  
  const std::vector<MappingKeyframe>& keyframes() const;
  std::vector<MappingKeyframe>& mutableKeyframes();
  
  const MappingKeyframe* latest() const;

private:
  double relativeDistance(const localization::Pose& a,
                          const localization::Pose& b) const;

  double relativeRotationRad(const localization::Pose& a,
                             const localization::Pose& b) const;
};
```

### 8.3 关键帧判断

参考 FASTLIO2_SAM_LC：

```cpp
diff_pose.norm() > distance_thresh
abs(rpy.x/y/z) > rotation_thresh_rad
```

首帧强制插入。

---

## 9. MapSaver

`include/slam_mapping/core/map_saver.hpp`

### 9.1 职责

- 用 keyframe 的 pose_map 把 cloud_body 变换到 map frame
- 拼接所有 keyframe 点云
- voxel filter 降采样
- 保存 PCD
- 可选保存 keyframe 点云、keyframe pose、trajectory

### 9.2 接口

```cpp
class MapSaver {
public:
  void configure(const MapSaverConfig& cfg);

  bool saveAll(const SaveRequest& req,
               const std::vector<MappingKeyframe>& keyframes,
               SaveResult* result);
  
  localization::PointCloudPtr buildMap(
      const std::vector<MappingKeyframe>& keyframes,
      double resolution) const;
  
  bool savePcd(const std::string& path,
               const localization::PointCloud& cloud) const;
  
  bool saveKeyframes(const std::string& dir,
                     const std::vector<MappingKeyframe>& keyframes) const;
  
  bool saveKeyframePoses(const std::string& path,
                         const std::vector<MappingKeyframe>& keyframes) const;
  
  bool saveTrajectory(const std::string& path,
                      const std::vector<MappingKeyframe>& keyframes) const;

private:
  void transformCloud(const localization::PointCloud& in,
                      const localization::Pose& pose_map,
                      localization::PointCloud* out) const;
};
```

### 9.3 保存地图核心逻辑

```cpp
for keyframe in keyframes:
  transform cloud_body by pose_map
  append to global_map

voxel filter global_map
pcl::io::savePCDFileBinary(map_path, global_map)
```

这部分对应 FASTLIO2_SAM_LC 的 `saveMapToFile()`。

---

## 10. LoopClosureBackend

`include/slam_mapping/core/loop_closure_backend.hpp`

首版空实现，但接口必须预留好。

### 10.1 职责

- 检测回环
- 构建 submap
- ICP 匹配
- 后续 GTSAM 优化
- 更新 keyframe 的 pose_map
- 输出 loop markers

### 10.2 接口

```cpp
struct LoopClosureResult {
  bool updated = false;
  int loop_count = 0;
  std::vector<std::pair<int, int>> loop_pairs;
};

class LoopClosureBackend {
public:
  void configure(const LoopClosureConfig& cfg);
  void reset();

  LoopClosureResult update(std::vector<MappingKeyframe>* keyframes);
  
  bool enabled() const;
  bool supported() const;

private:
  bool detectCandidate(const std::vector<MappingKeyframe>& keyframes,
                       int* current_idx,
                       int* history_idx);

  bool runIcpVerification(const std::vector<MappingKeyframe>& keyframes,
                          int current_idx,
                          int history_idx);
  
  void optimizePoseGraph(std::vector<MappingKeyframe>* keyframes);
};
```

### 10.3 后续接 FASTLIO2_SAM_LC 时可迁移

- LoopClosureThread
- getSubMaps
- ICP
- ISAM2
- loop_pairs
- loop_history

但要把 `fastlio::PointCloudXYZI` 替换为 `localization::PointCloud`。

---

## 11. VisualizationPublisher

`include/slam_mapping/ros/visualization_publisher.hpp`

发布话题：

| 话题名 | 说明 |
|--------|------|
| `/slam_mapping/odom` | 里程计 |
| `/slam_mapping/path` | 轨迹 |
| `/slam_mapping/current_scan` | 当前扫描 |
| `/slam_mapping/local_map` | 局部地图 |
| `/slam_mapping/map_preview` | 地图预览 |
| `/slam_mapping/keyframes` | 关键帧 |
| `/slam_mapping/loop_markers` | 回环标记 |
| `/slam_mapping/status` | 状态 |

### 11.1 接口

```cpp
class VisualizationPublisher {
public:
  void initialize(ros::NodeHandle& nh,
                  const VisualizationConfig& cfg,
                  const FrameConfig& frames,
                  const OutputTopicConfig& topics);

  void publishOdom(const localization::OdomResult& odom);
  void publishPath(const std::vector<MappingKeyframe>& keyframes);
  void publishCurrentScan(const localization::TimestampedPointCloud& scan,
                          const std::string& frame_id);
  void publishLocalMap(const localization::TimestampedPointCloud& map,
                       const std::string& frame_id);
  void publishMapPreview(const std::vector<MappingKeyframe>& keyframes);
  void publishKeyframeMarkers(const std::vector<MappingKeyframe>& keyframes);
  void publishLoopMarkers(const std::vector<std::pair<int, int>>& loop_pairs,
                          const std::vector<MappingKeyframe>& keyframes);
  void publishStatus(const MappingRuntimeStatus& status);

private:
  nav_msgs::Path buildPathMsg(...);
  visualization_msgs::MarkerArray buildKeyframeMarkers(...);
  visualization_msgs::MarkerArray buildLoopMarkers(...);
};
```

可视化参考 FASTLIO2_SAM_LC：

- local path
- global path
- loop marker
- cloud body
- cloud world
- saved/preview map

---

## 12. RosAdapter

`include/slam_mapping/ros/ros_adapter.hpp`

### 12.1 职责

- ROS 参数加载
- ROS 订阅
- ROS 服务
- 调用 MappingRuntime
- 发布可视化

### 12.2 接口

```cpp
class SlamMappingRosAdapter {
public:
  bool initialize(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spinOnce();

private:
  void onImu(const sensor_msgs::Imu::ConstPtr& msg);
  void onLidar(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void onCan(const deeplumin_msgs::CanReceiveInfo::ConstPtr& msg);
  void onGnss(const deeplumin_msgs::Gpchc::ConstPtr& msg);

  bool handleStart(deeplumin_msgs::StartMappingRuntime::Request& req,
                   deeplumin_msgs::StartMappingRuntime::Response& res);
  
  bool handleStop(deeplumin_msgs::StopMappingRuntime::Request& req,
                  deeplumin_msgs::StopMappingRuntime::Response& res);
  
  bool handleSave(deeplumin_msgs::SaveMappingRuntime::Request& req,
                  deeplumin_msgs::SaveMappingRuntime::Response& res);
  
  bool handleReset(std_srvs::Trigger::Request& req,
                   std_srvs::Trigger::Response& res);
  
  bool handleGetStatus(deeplumin_msgs::GetMappingRuntimeStatus::Request& req,
                       deeplumin_msgs::GetMappingRuntimeStatus::Response& res);
  
  void publishOutputs();
};
```

---

## 13. slam_mapping_node.cpp

入口文件保持很薄：

```cpp
int main(int argc, char** argv) {
  ros::init(argc, argv, "slam_mapping");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  slam_mapping::SlamMappingRosAdapter node;
  if (!node.initialize(nh, pnh)) {
    ROS_ERROR("[slam_mapping] initialize failed");
    return 1;
  }
  
  ros::Rate rate(node.processRateHz());
  while (ros::ok()) {
    ros::spinOnce();
    node.spinOnce();
    rate.sleep();
  }
  return 0;
}
```

---

## 14. mapping_manager 适配方式

mapping_manager 不直接运行算法，只调用 backend。

### 14.1 新增配置

```yaml
backend:
  enabled: true
  start_service: /slam_mapping/start
  stop_service: /slam_mapping/stop
  get_status_service: /slam_mapping/get_status
```

### 14.2 Manager 流程

**`/mapping/start`**:
1. manager 生成 task_id/output_dir
2. 调 `/slam_mapping/start(task_id, output_dir, config_path, enable_loop_closure)`
3. manager 状态 STARTING/RUNNING

**`/mapping/stop`**:
1. manager 调 `/slam_mapping/stop(save_map=true...)`
2. slam_mapping 保存地图
3. manager 从 `/slam_mapping/get_status` 读取 map_path/output_valid
4. manager 状态 FINISHED

### 14.3 优势

- 手动模式：直接调 `/slam_mapping/start`
- 托管模式：调 `/mapping/start`
- 两者复用同一套建图 runtime

---

## 15. 首版实现顺序

建议按这个顺序填代码：

1. 新建包骨架、CMake、package
2. 写 `types.hpp/config.hpp/param_loader`
3. 写 `MappingRuntime`，先只接 `ISlamOdometry`
4. 写 `RosAdapter`，能订阅传感器并跑 `slam_->processOnce()`
5. 写 `KeyframeManager`，按距离/角度保存 keyframe
6. 写 `MapSaver::buildMap/savePcd`
7. 写 `/slam_mapping/start/stop/save/get_status` 服务
8. 写可视化：path、current_scan、local_map、map_preview
9. 写 mapping_manager backend client
10. 预留但暂不实现 loop closure

---

## 16. 首版验收标准

### 16.1 单节点模式

```bash
roslaunch slam_mapping slam_mapping.launch auto_start:=true
rosservice call /slam_mapping/save ...
```

应满足：

- `/slam_mapping/odom` 有输出
- `/slam_mapping/path` 有轨迹
- `/slam_mapping/current_scan` 有点云
- `/slam_mapping/map_preview` 有累计地图
- `/slam_mapping/status` 中 frame_count/keyframe_count 增长
- stop/save 后输出 map.pcd

### 16.2 Manager 模式

```bash
rosservice call /mapping/start ...
rosservice call /mapping/stop ...
```

应满足：

- manager 状态从 STARTING -> RUNNING -> SAVING_RESULTS -> FINISHED
- map_path 有效
- output_valid=true
- 生成 map.pcd/keyframes/keyframes.csv/trajectory.csv

> **注意**：首版不要急着接回环。先把"里程计驱动 + 关键帧 + PCD 保存 + manager 控制"跑通，再接 FASTLIO2_SAM_LC 的 ICP/GTSAM 回环。