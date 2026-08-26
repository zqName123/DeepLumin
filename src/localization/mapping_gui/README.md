# mapping_gui 节点说明文档

## 1. 概述

`mapping_gui` 是 DeepLumin 建图系统的可视化控制台节点，基于 Python tkinter 构建，为用户提供图形化界面来控制和监控建图任务。该节点作为用户交互入口，通过 ROS Service 与后端的 `mapping_manager` 通信，通过 ROS Topic 订阅实时状态和数据。

**核心定位**：
- 用户交互入口（控制面板）
- 任务状态监控（状态面板）
- 环境检测触发（检测面板）
- 结果展示（结果面板）

**不负责**：
- 直接运行建图算法核心
- 直接拼接地图或保存 PCD
- 三维点云渲染（由 RViz 负责）

---

## 2. 目录结构

```
mapping_gui/
├── CMakeLists.txt              # Catkin 构建配置
├── package.xml                 # ROS 包描述
├── requirements.txt            # Python 依赖说明（无第三方 pip 依赖）
├── README.md                   # 本文档
│
├── launch/
│   └── mapping_gui.launch      # 启动文件，可联动启动 mapping_manager 和 slam_mapping_backend
│
└── scripts/
    └── mapping_gui_node.py     # GUI 主节点（Python3 + tkinter）
```

---

## 3. 输入输出详细说明

### 3.1 输入（Subscriptions）

| Topic 名称 | 类型 | 说明 |
|-----------|------|------|
| `/mapping/status` | `deeplumin_msgs/MappingStatus` | 建图任务实时状态（状态机、帧数、关键帧数、结果路径等） |
| `/mapping/record_status` | `deeplumin_msgs/MappingRecordStatus` | 录包任务实时状态（录包状态、bag 文件路径） |
| `/mapping/environment_last_result` | `deeplumin_msgs/MappingEnvironmentStatus` | 最近一次环境检测结果（各项检测项的通过/失败状态） |

**参数化 Topic 名称**（可通过 launch 文件或参数服务器配置）：
- `~status_topic`（默认：`/mapping/status`）
- `~record_status_topic`（默认：`/mapping/record_status`）
- `~environment_topic`（默认：`/mapping/environment_last_result`）

### 3.2 输出（Service Clients）

GUI 作为客户端，调用 `mapping_manager` 提供的服务：

| Service 名称 | 类型 | 说明 |
|-------------|------|------|
| `/mapping/check_environment` | `CheckMappingEnvironment` | 触发环境检测（ROS Master、传感器、磁盘空间等） |
| `/mapping/get_status` | `GetMappingStatus` | 获取当前建图任务完整状态 |
| `/mapping/start` | `StartMapping` | 开始建图任务 |
| `/mapping/stop` | `StopMapping` | 结束建图任务并保存结果 |
| `/mapping/start_bag_recording` | `StartBagRecording` | 开始录制 ROS bag |
| `/mapping/stop_bag_recording` | `StopBagRecording` | 停止录制并落盘 bag 文件 |

**参数化 Service 名称**：
- `~check_environment_service`（默认：`/mapping/check_environment`）
- `~get_status_service`（默认：`/mapping/get_status`）
- `~start_mapping_service`（默认：`/mapping/start`）
- `~stop_mapping_service`（默认：`/mapping/stop`）
- `~start_record_service`（默认：`/mapping/start_bag_recording`）
- `~stop_record_service`（默认：`/mapping/stop_bag_recording`）

### 3.3 可视化数据（由 RViz 订阅，GUI 不直接处理）

| Topic 名称 | 类型 | 说明 |
|-----------|------|------|
| `/mapping/map_points` | `sensor_msgs/PointCloud2` | 实时地图点云 |
| `/mapping/current_scan` | `sensor_msgs/PointCloud2` | 当前帧点云 |
| `/mapping/path` | `nav_msgs/Path` | 建图轨迹 |
| `/mapping/odom` | `nav_msgs/Odometry` | 当前位姿 |
| `/mapping/loop_markers` | `visualization_msgs/MarkerArray` | 回环检测可视化标记 |

> **注意**：GUI 本身不订阅这些可视化 Topic，三维显示由 RViz 负责。GUI 仅通过 `enable_rviz` 选项控制 `mapping_manager` 是否启动 RViz。

---

## 4. 界面布局

GUI 主窗口采用左右分栏布局（1180x820）：

```
+----------------------------------------------------------+
|  DeepLumin 建图控制台                                    |
+---------------------------+------------------------------+
|  左侧：控制面板            |  右侧：状态与信息面板         |
|                           |                              |
|  ┌─ 建图任务 ─────────┐  |  ┌─ 运行状态 ────────────┐  |
|  │ 任务名称            │  |  │ 建图状态 / 任务ID      │  |
|  │ 配置文件路径        │  |  │ 输出目录 / 状态说明    │  |
|  │ 输出根目录          │  |  │ 录包状态 / Bag路径     │  |
|  │ [启用回环检测]      │  |  │ 最近回环信息           │  |
|  │ [启动RViz]          │  |  └───────────────────────┘  |
|  │ [自动录包]          │  |                              |
|  └─────────────────────┘  |  ┌─ 环境检测 / 接口约定 ─┐  |
|                           |  │ 接口说明文本           │  |
|  ┌─ 操作区 ───────────┐  |  │ 环境检测结果列表       │  |
|  │ [检测环境][刷新状态]│  |  └───────────────────────┘  |
|  │ [开始建图][结束建图]│  |                              |
|  └─────────────────────┘  |  ┌─ 最新结果路径 ────────┐  |
|                           |  │ 地图PCD / 轨迹文件     │  |
|  ┌─ Bag 录制 ─────────┐  |  │ 关键帧目录 / 位姿文件  │  |
|  │ Bag名称 / 输出目录  │  |  └───────────────────────┘  |
|  │ 录制话题列表        │  |                              |
|  │ [开始录包][停止录包]│  |  ┌─ 事件日志 ────────────┐  |
|  └─────────────────────┘  |  │ 时间戳 + 事件描述      │  |
|                           |  │ (最多保留300条)        │  |
+---------------------------+  └───────────────────────┘  |
+----------------------------------------------------------+
```

---

## 5. 工作流程

### 5.1 启动流程

```
1. 用户执行：roslaunch mapping_gui mapping_gui.launch
   ├── 可选：启动 slam_mapping_backend（slam_mapping_manager_backend.launch）
   ├── 可选：启动 mapping_manager（mapping_manager.launch）
   └── 启动 mapping_gui_node.py

2. GUI 初始化：
   - 初始化 ROS 节点（mapping_gui）
   - 加载参数（topic/service 名称、默认值）
   - 构建 tkinter 界面
   - 订阅状态 Topic
   - 启动 UI 刷新定时器（500ms 周期）
   - 显示："界面已初始化，正在等待 mapping_manager 接口"
```

### 5.2 环境检测流程

```
用户点击 [检测环境]
    │
    ▼
异步调用 CheckMappingEnvironment Service
    │
    ▼
mapping_manager 执行检测：
    - ROS Master 在线状态
    - IMU / LiDAR 话题可用性
    - 传感器数据频率
    - 时间同步情况
    - 磁盘剩余空间
    - 输出路径可写性
    - 配置文件存在性
    - 回环模块依赖可用性
    │
    ▼
GUI 接收结果，更新环境检测面板
    - 显示 overall_ok（总体是否通过）
    - 逐项显示检测项名称、级别、结果、详情
    - 日志记录检测结果
```

### 5.3 开始建图流程

```
用户配置参数：
    - 任务名称
    - 配置文件路径（slam_mapping 配置）
    - 输出根目录
    - 启用回环检测（checkbox）
    - 启动 RViz（checkbox）
    - 自动开始录包（checkbox）
    │
    ▼
用户点击 [开始建图]
    │
    ▼
异步调用 StartMapping Service
    │
    ▼
mapping_manager 处理：
    - 验证参数和环境
    - 创建任务目录
    - 启动 slam_mapping_node
    - 可选：启动回环检测模块
    - 可选：启动 RViz
    - 可选：自动启动录包
    │
    ▼
GUI 接收响应：
    - accepted=true：记录 task_id，日志显示成功
    - accepted=false：弹窗警告，显示拒绝原因
    │
    ▼
GUI 订阅 /mapping/status，等待状态变为 RUNNING
    - 状态栏更新为"建图中"
    - 显示任务 ID 和输出目录
```

### 5.4 结束建图流程

```
用户点击 [结束建图]
    │
    ▼
GUI 检查当前状态：
    - 若 frame_count == 0 或 keyframe_count == 0：
      弹窗提示"无有效关键帧，是否仅停止不保存？"
    │
    ▼
异步调用 StopMapping Service
    参数：
    - save_map（保存地图 PCD）
    - save_keyframes（保存关键帧点云）
    - save_keyframe_poses（保存关键帧位姿）
    - save_trajectory（保存轨迹文件）
    │
    ▼
mapping_manager 处理：
    - 停止 slam_mapping_node
    - 保存地图、轨迹、关键帧等结果
    - 更新任务状态为 FINISHED 或 FAILED
    │
    ▼
GUI 接收响应并更新：
    - 结果路径面板显示：
      * 地图 PCD 路径
      * 轨迹文件路径
      * 关键帧目录路径
      * 关键帧位姿文件路径
    - 日志记录保存结果
```

### 5.5 录包流程

```
用户配置：
    - Bag 名称
    - Bag 输出目录
    - 录制话题列表（逗号分隔）
    │
    ▼
用户点击 [开始录包]
    │
    ▼
异步调用 StartBagRecording Service
    │
    ▼
mapping_manager 启动 rosbag record
    │
    ▼
GUI 接收响应，更新录包状态为 RUNNING
    显示 bag 文件路径

用户点击 [停止录包]
    │
    ▼
异步调用 StopBagRecording Service
    │
    ▼
mapping_manager 停止录制并落盘
    │
    ▼
GUI 更新录包状态，显示最终 bag 路径
```

### 5.6 状态刷新流程

```
定时器周期：500ms
    │
    ▼
GUI 从订阅的 Topic 读取最新状态：
    - /mapping/status → 建图状态
    - /mapping/record_status → 录包状态
    - /mapping/environment_last_result → 环境检测结果
    │
    ▼
更新界面显示：
    - 状态面板：state、task_id、output_dir、message
    - 录包面板：record_state、bag_path
    - 结果面板：map_path、trajectory_path、keyframe_dir、keyframe_pose_path
    - 回环信息：last_loop_from → last_loop_to（若存在）
    - 环境检测面板：overall_ok、summary、逐项检测结果
    - 日志面板：最近 300 条日志
```

---

## 6. 状态机说明

### 6.1 建图状态（MappingStatus.state）

| 状态值 | 含义 |
|--------|------|
| `IDLE` | 空闲，无运行中任务 |
| `STARTING` | 正在启动建图任务 |
| `RUNNING` | 建图运行中 |
| `STOPPING` | 正在停止建图 |
| `SAVING_RESULTS` | 正在保存结果 |
| `FINISHED` | 任务完成 |
| `FAILED` | 任务失败 |

### 6.2 录包状态（MappingRecordStatus.state）

| 状态值 | 含义 |
|--------|------|
| `REC_IDLE` | 未在录包 |
| `REC_STARTING` | 正在启动录包 |
| `REC_RUNNING` | 录包进行中 |
| `REC_STOPPING` | 正在停止录包 |
| `REC_SAVED` | 录包已保存 |
| `REC_FAILED` | 录包失败 |

---

## 7. 依赖关系

### 7.1 运行依赖

| 依赖 | 说明 |
|------|------|
| ROS Noetic | ROS 运行环境 |
| rospy | ROS Python 客户端库 |
| tkinter | Python 标准库 GUI 工具包 |
| deeplumin_msgs | DeepLumin 自定义消息包 |
| mapping_manager | 后端建图管理节点（提供 Service） |

### 7.2 可选依赖

| 依赖 | 说明 |
|------|------|
| slam_mapping | 建图算法节点（由 mapping_manager 启动） |
| RViz | 三维可视化工具（由 mapping_manager 按需启动） |

---

## 8. 配置参数

### 8.1 Topic 参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `~status_topic` | `/mapping/status` | 建图状态 Topic |
| `~record_status_topic` | `/mapping/record_status` | 录包状态 Topic |
| `~environment_topic` | `/mapping/environment_last_result` | 环境检测结果 Topic |

### 8.2 Service 参数

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `~check_environment_service` | `/mapping/check_environment` | 环境检测服务 |
| `~get_status_service` | `/mapping/get_status` | 获取状态服务 |
| `~start_mapping_service` | `/mapping/start` | 开始建图服务 |
| `~stop_mapping_service` | `/mapping/stop` | 结束建图服务 |
| `~start_record_service` | `/mapping/start_bag_recording` | 开始录包服务 |
| `~stop_record_service` | `/mapping/stop_bag_recording` | 停止录包服务 |

---

## 9. 使用示例

### 9.1 单独启动 GUI（假设后端已运行）

```bash
rosrun mapping_gui mapping_gui_node.py
```

### 9.2 完整启动（GUI + mapping_manager + slam_mapping）

```bash
roslaunch mapping_gui mapping_gui.launch
```

### 9.3 自定义配置启动

```bash
roslaunch mapping_gui mapping_gui.launch \
    slam_mapping_config:=$(find slam_mapping)/config/slam_mapping_parking_rslidar.yaml \
    mapping_manager_config:=$(find mapping_manager)/config/mapping_manager.yaml
```

---

## 10. 注意事项

1. **GUI 不直接处理三维显示**：点云可视化由 RViz 负责，GUI 仅通过 `enable_rviz` 参数控制是否启动 RViz。

2. **Service 调用为异步**：所有按钮操作均在后台线程中执行，避免阻塞 UI 主线程。

3. **状态驱动原则**：GUI 界面状态以 `/mapping/status` Topic 发布的状态为准，不以按钮点击为成功依据。

4. **日志限制**：日志面板最多保留 300 条历史记录，超出后自动丢弃最旧的记录。

5. **关闭行为**：点击窗口关闭按钮时，GUI 会调用 `rospy.signal_shutdown()` 优雅关闭 ROS 节点。

6. **无第三方 pip 依赖**：所有依赖均为 ROS/catkin 环境自带或 Python 标准库。