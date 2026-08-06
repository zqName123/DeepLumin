# mapping_manager 节点说明文档

## 1. 概述

`mapping_manager` 是 DeepLumin 建图系统的中央管理节点，负责建图任务的全生命周期管理。作为后端核心，它向上层 GUI 提供统一的服务接口，向下协调 `slam_mapping` 等后端算法节点，同时管理 bag 录制等辅助功能。

**核心定位**：
- 建图任务生命周期管理（创建、运行、停止、保存）
- 后端算法节点协调（启动、停止、状态同步）
- ROS Bag 录制管理
- 环境检测与前置条件验证
- 状态聚合与发布

**系统角色**：
```
GUI / 其他客户端
    │  Service 调用
    ▼
mapping_manager (本节点)
    │  Service 调用 / Topic 订阅
    ▼
slam_mapping_node (后端建图算法)
```

---

## 2. 目录结构

```
mapping_manager/
├── CMakeLists.txt                    # Catkin 构建配置
├── package.xml                       # ROS 包描述
├── README.md                         # 本文档
│
├── config/
│   └── mapping_manager.yaml          # 默认配置文件
│
├── include/mapping_manager/
│   ├── common/
│   │   ├── config.hpp                # 配置结构体定义
│   │   └── types.hpp                 # 核心数据类型定义（状态枚举、结构体）
│   ├── core/
│   │   └── manager_core.hpp          # 管理核心类声明
│   └── ros/
│       ├── ros_adapter.hpp           # ROS 消息与内部类型的转换接口
│       └── bag_recorder.hpp          # Bag 录制器类声明
│
├── src/
│   ├── mapping_manager_node.cpp      # 主节点入口（ROS 节点封装）
│   ├── core/
│   │   └── manager_core.cpp          # 管理核心实现（状态机、任务管理）
│   └── ros/
│       ├── ros_adapter.cpp           # ROS 消息转换实现
│       └── bag_recorder.cpp          # Bag 录制器实现（rosbag 进程管理）
│
├── launch/
│   └── mapping_manager.launch        # 启动文件
│
└── scripts/
    └── mapping_manager_node.py       # Python 版本节点（备用实现）
```

### 目录/文件含义说明

| 路径 | 说明 |
|------|------|
| `include/mapping_manager/common/` | 公共头文件：配置结构体、数据类型定义，不依赖 ROS |
| `include/mapping_manager/core/` | 核心算法头文件：ManagerCore 类声明，纯业务逻辑 |
| `include/mapping_manager/ros/` | ROS 适配头文件：消息转换、BagRecorder 声明 |
| `src/core/manager_core.cpp` | **核心实现**：状态机管理、任务生命周期、环境检测逻辑 |
| `src/ros/ros_adapter.cpp` | **ROS 适配层**：内部类型与 `deeplumin_msgs` 的互相转换 |
| `src/ros/bag_recorder.cpp` | **Bag 录制实现**：fork/exec rosbag record 进程管理 |
| `src/mapping_manager_node.cpp` | **主节点**：ROS 节点封装，Service/Topic/Timers 的 ROS 侧实现 |

---

## 3. 功能模块

### 3.1 状态机管理（ManagerCore）

管理建图任务的状态流转：

```
IDLE ──→ CHECKING_ENV ──→ READY ──→ STARTING ──→ RUNNING ──→ STOPPING ──→ SAVING_RESULTS ──→ FINISHED
  │          │              │           │            │            │              │
  └──────────┴──────────────┴───────────┴────────────┴────────────┴──────────────┘──→ FAILED
```

| 状态 | 含义 |
|------|------|
| `IDLE` | 空闲，无运行中任务 |
| `CHECKING_ENV` | 正在执行环境检测 |
| `READY` | 环境检测通过，等待开始建图 |
| `STARTING` | 正在请求后端启动建图 |
| `RUNNING` | 建图运行中 |
| `STOPPING` | 正在请求后端停止建图 |
| `SAVING_RESULTS` | 正在保存结果 |
| `FINISHED` | 任务完成 |
| `FAILED` | 任务失败 |

### 3.2 后端协调

当 `backend.enabled=true` 时，mapping_manager 会：
- 通过 Service 调用 `slam_mapping` 的启动/停止接口
- 订阅 `slam_mapping` 的状态 Topic 进行实时同步
- 定期轮询 `slam_mapping` 的 get_status 服务

### 3.3 Bag 录制管理

通过 `BagRecorder` 类管理 `rosbag record` 子进程：
- 启动：fork/exec 启动 rosbag record
- 停止：先发送 SIGINT 优雅关闭，超时后发送 SIGKILL
- 监控：定时检查子进程是否意外退出
- 统计：获取 bag 文件大小

### 3.4 环境检测

在开始建图前验证系统环境：
- ROS Master 在线状态
- 配置文件存在性
- 输出目录可写性
- 后端服务可用性（可选）
- 当前是否有运行中的建图任务

---

## 4. 输入输出详细说明

### 4.1 对外提供的服务（Service Servers）

| Service 名称 | 类型 | 输入 | 输出 | 说明 |
|-------------|------|------|------|------|
| `/mapping/check_environment` | `CheckMappingEnvironment` | `include_runtime_checks`, `config_path` | `overall_ok`, `summary`, `items[]` | 环境检测 |
| `/mapping/get_status` | `GetMappingStatus` | `task_id` | `status`, `record_status`, `environment_status` | 获取完整状态 |
| `/mapping/start` | `StartMapping` | `task_name`, `config_path`, `output_root`, `enable_loop_closure`, `enable_visualization`, `auto_start_recording` | `accepted`, `task_id`, `state`, `reason` | 开始建图 |
| `/mapping/stop` | `StopMapping` | `task_id`, `save_map`, `save_keyframes`, `save_keyframe_poses`, `save_trajectory` | `accepted`, `state`, `reason` | 结束建图 |
| `/mapping/start_bag_recording` | `StartBagRecording` | `task_id`, `bag_name`, `output_dir`, `topics[]` | `accepted`, `state`, `reason`, `bag_path` | 开始录包 |
| `/mapping/stop_bag_recording` | `StopBagRecording` | `task_id`, `save_now` | `accepted`, `state`, `reason`, `bag_path` | 停止录包 |

### 4.2 对外发布的 Topic

| Topic 名称 | 类型 | 发布频率 | 说明 |
|-----------|------|---------|------|
| `/mapping/status` | `MappingStatus` | 5Hz | 建图任务实时状态 |
| `/mapping/record_status` | `MappingRecordStatus` | 5Hz | 录包任务实时状态 |
| `/mapping/environment_last_result` | `MappingEnvironmentStatus` | 按需（latch=true） | 最近一次环境检测结果 |

#### MappingStatus 消息结构

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | string | 任务唯一标识 |
| `task_name` | string | 任务名称 |
| `state` | string | 状态机当前状态 |
| `status_message` | string | 状态说明文本 |
| `ready` | bool | 是否就绪 |
| `running` | bool | 是否运行中 |
| `saving` | bool | 是否正在保存 |
| `loop_enabled` | bool | 是否启用回环检测 |
| `loop_supported` | bool | 后端是否支持回环 |
| `output_dir` | string | 输出目录 |
| `map_path` | string | 地图 PCD 路径 |
| `trajectory_path` | string | 轨迹文件路径 |
| `keyframe_dir` | string | 关键帧目录 |
| `keyframe_pose_path` | string | 关键帧位姿文件路径 |
| `frame_count` | uint32 | 累计帧数 |
| `keyframe_count` | uint32 | 关键帧数 |
| `loop_count` | uint32 | 回环次数 |
| `last_loop_from` | int32 | 最近回环起始帧 |
| `last_loop_to` | int32 | 最近回环目标帧 |
| `last_loop_score` | float64 | 最近回环分数 |
| `duration_sec` | float64 | 运行时长 |

#### MappingRecordStatus 消息结构

| 字段 | 类型 | 说明 |
|------|------|------|
| `task_id` | string | 关联任务 ID |
| `state` | string | 录包状态 |
| `active` | bool | 是否活跃 |
| `bag_path` | string | bag 文件路径 |
| `message` | string | 状态说明 |
| `duration_sec` | float64 | 录制时长 |
| `bytes_written` | uint64 | 已写入字节数 |

### 4.3 对后端的服务调用（Service Clients，可选）

当 `backend.enabled=true` 时：

| Service 名称 | 类型 | 说明 |
|-------------|------|------|
| `/slam_mapping/start` | `StartMappingRuntime` | 请求后端启动建图 |
| `/slam_mapping/stop` | `StopMappingRuntime` | 请求后端停止建图 |
| `/slam_mapping/get_status` | `GetMappingRuntimeStatus` | 获取后端状态 |

### 4.4 对后端的 Topic 订阅（可选）

| Topic 名称 | 类型 | 说明 |
|-----------|------|------|
| `/slam_mapping/status` | `MappingStatus` | 后端实时状态 |

---

## 5. 配置参数

### 5.1 默认配置（mapping_manager.yaml）

```yaml
topics:
  status: /mapping/status                          # 建图状态发布 Topic
  record_status: /mapping/record_status            # 录包状态发布 Topic
  environment_last_result: /mapping/environment_last_result  # 环境检测结果 Topic

services:
  check_environment: /mapping/check_environment    # 环境检测服务
  get_status: /mapping/get_status                  # 获取状态服务
  start_mapping: /mapping/start                    # 开始建图服务
  stop_mapping: /mapping/stop                      # 结束建图服务
  start_bag_recording: /mapping/start_bag_recording  # 开始录包服务
  stop_bag_recording: /mapping/stop_bag_recording    # 停止录包服务

runtime:
  status_hz: 5.0                                   # 状态发布频率
  startup_delay_sec: 1.0                           # 启动延迟
  save_delay_sec: 1.5                              # 保存延迟
  auto_finish_delay_sec: 0.5                       # 自动完成延迟

defaults:
  output_root: /tmp/deeplumin_mapping              # 默认输出根目录
  bag_output_dir: /tmp/deeplumin_mapping/bag       # 默认录包目录
  bag_topics: /localization/points,/ouster/imu,/mapping/status,/mapping/odom,/mapping/map_points,/mapping/loop_markers

backend:
  enabled: true                                    # 是否启用后端协调
  start_service: /slam_mapping/start               # 后端启动服务
  stop_service: /slam_mapping/stop                 # 后端停止服务
  get_status_service: /slam_mapping/get_status     # 后端状态服务
```

---

## 6. 整体工作流程

### 6.1 节点启动流程

```
1. 加载配置（loadConfig）
   - 从参数服务器读取 topics、services、runtime、defaults、backend 配置

2. 初始化 ManagerCore
   - 设置初始状态为 IDLE
   - 初始化状态数据结构

3. 创建 ROS 通信
   - 发布器：status、record_status、environment_last_result
   - 服务：6 个对外服务
   - 定时器：5Hz 状态发布

4. 连接后端（backend.enabled=true 时）
   - 创建后端 ServiceClient
   - 订阅后端状态 Topic

5. 发布初始环境检测结果
   - 发布 latch 的 environment_last_result

6. 启动定时器循环
```

### 6.2 开始建图流程

```
GUI 调用 /mapping/start Service
    │
    ▼
handleStartMapping()
    │
    ├── 1. 调用 ManagerCore::startMapping()
    │   - 状态检查（必须在 IDLE/READY/FINISHED）
    │   - 生成 task_id（task_name + 时间戳）
    │   - 创建输出目录
    │   - 状态设为 STARTING
    │   - 若 auto_start_recording=true，初始化录包状态
    │
    ├── 2. 若 backend.enabled=true
    │   - 检查后端服务是否可用
    │   - 调用 /slam_mapping/start Service
    │   - 传递 task_id、config_path、output_dir 等参数
    │   - 若后端启动失败，标记 FAILED
    │
    ├── 3. 若 auto_start_recording=true
    │   - 启动 BagRecorder
    │   - fork/exec rosbag record
    │
    └── 4. 返回响应（accepted、task_id、state、reason）

定时器持续发布 /mapping/status
    - 状态从 STARTING → RUNNING（由后端状态同步驱动）
```

### 6.3 运行中状态同步流程

```
定时器回调 onTimer() 每 200ms 执行：
    │
    ├── 1. 调用 ManagerCore::update()
    │   - 更新运行时长
    │   - 更新录包时长
    │
    ├── 2. 检查 BagRecorder 状态
    │   - poll() 检查子进程是否退出
    │   - 若意外退出，标记 REC_FAILED
    │   - 更新 bag 文件大小
    │
    ├── 3. 若 backend.enabled=true
    │   - syncBackendStatus() 轮询后端状态
    │   - 或接收后端状态 Topic 回调 onBackendStatus()
    │   - updateFromBackend() 更新内部状态
    │
    └── 4. 发布状态 Topic
        - /mapping/status
        - /mapping/record_status
        - /mapping/environment_last_result
```

### 6.4 结束建图流程

```
GUI 调用 /mapping/stop Service
    │
    ▼
handleStopMapping()
    │
    ├── 1. 调用 ManagerCore::stopMapping()
    │   - 状态检查（必须在 RUNNING/STARTING/READY）
    │   - task_id 匹配检查
    │   - 状态设为 STOPPING
    │
    ├── 2. 若 backend.enabled=true
    │   - 检查后端服务是否可用
    │   - 调用 /slam_mapping/stop Service
    │   - 传递 save_map、save_keyframes 等参数
    │   - 同步后端最终状态（syncBackendStatus）
    │   - 获取结果路径（map_path、trajectory_path 等）
    │
    ├── 3. 停止 BagRecorder（若正在录包）
    │   - 发送 SIGINT
    │   - 等待优雅退出或强制 kill
    │   - 获取最终文件大小
    │
    └── 4. 返回响应

状态流转：STOPPING → SAVING_RESULTS → FINISHED/FAILED
```

### 6.5 环境检测流程

```
GUI 调用 /mapping/check_environment Service
    │
    ▼
handleCheckEnvironment()
    │
    ├── 1. 调用 ManagerCore::checkEnvironment()
    │   - 检查 ROS Master
    │   - 检查输出目录
    │   - 检查配置文件
    │   - 检查是否有运行中的任务
    │
    ├── 2. 若 include_runtime_checks=true 且 backend.enabled=true
    │   - 检查后端服务是否可用
    │
    └── 3. 计算 overall_ok（所有检测项的与）
        - 若全部通过，状态设为 READY
        - 发布环境检测结果（latch topic）
        - 返回检测结果
```

### 6.6 Bag 录制流程

```
开始录包：
    GUI 调用 /mapping/start_bag_recording
        │
        ├── ManagerCore::startBagRecording()
        │   - 检查是否已在录包
        │   - 生成 bag 路径
        │   - 状态设为 REC_RUNNING
        │
        ├── spawnBagRecorder()
        │   - fork/exec rosbag record
        │   - 记录 pid
        │
        └── 返回 bag_path

停止录包：
    GUI 调用 /mapping/stop_bag_recording
        │
        ├── ManagerCore::stopBagRecording()
        │   - 状态设为 REC_SAVED
        │
        ├── BagRecorder::stop()
        │   - 发送 SIGINT
        │   - 等待退出（超时 SIGKILL）
        │
        └── 获取文件大小，更新状态
```

---

## 7. 核心类说明

### 7.1 ManagerCore

纯业务逻辑核心，不依赖 ROS。负责：
- 状态机维护
- 任务生命周期管理
- 环境检测逻辑
- 状态数据聚合

### 7.2 BagRecorder

独立进程管理器，负责：
- `start()`: fork/exec 启动 rosbag record
- `stop()`: 发送 SIGINT/SIGKILL 停止录制
- `poll()`: 检查子进程状态
- `shutdown()`: 节点关闭时的快速清理

### 7.3 RosAdapter

类型转换层，负责：
- 内部类型 ↔ ROS 消息的双向转换
- 配置加载（从 ROS 参数服务器）

---

## 8. 依赖关系

| 依赖 | 说明 |
|------|------|
| ROS Noetic | ROS 运行环境 |
| roscpp | ROS C++ 客户端库 |
| deeplumin_msgs | DeepLumin 自定义消息包 |
| slam_mapping | 后端建图算法节点（可选） |

---

## 9. 使用示例

### 9.1 单独启动

```bash
roslaunch mapping_manager mapping_manager.launch
```

### 9.2 带自定义配置启动

```bash
roslaunch mapping_manager mapping_manager.launch \
    config:=$(find mapping_manager)/config/mapping_manager.yaml
```

### 9.3 禁用后端协调（纯管理模式）

```bash
roslaunch mapping_manager mapping_manager.launch \
    backend/enabled:=false
```

---

## 10. 注意事项

1. **后端服务可用性检查**：所有后端 Service 调用前都会检查服务是否存在，若不存在会返回失败并标记状态为 FAILED。

2. **Bag 录制进程监控**：定时器会检查 rosbag record 子进程是否意外退出（启动 2 秒后才检查，避免误判启动期）。

3. **状态发布**：所有状态 Topic 以 5Hz 固定频率发布，确保 GUI 能实时获取最新状态。

4. **环境检测结果 latch**：`/mapping/environment_last_result` Topic 设置为 latch=true，新订阅者能立即获取最新结果。

5. **任务 ID 生成规则**：`{task_name}_{YYYYMMDD_HHMMSS}`，其中 task_name 中的空格和路径分隔符会被替换为下划线。

6. **输出目录结构**：
   ```
   {output_root}/
   └── {task_id}/
       ├── map.pcd
       ├── trajectory.txt
       ├── keyframes/
       ├── keyframe_poses.txt
       └── bag/
           └── {bag_name}.bag
   ```