# DeepLumin `mapping_manager` 技术设计

## 1. 目标

本文档定义 `DeepLumin/localization` 中可视化建图系统的后端编排器 `mapping_manager` 的职责、状态机、接口、与 GUI 的联动方式，以及如何复用 `FASTLIO2_SAM_LC_ws` 中 `map_builder_node` 的流程思想。

目标不是直接照搬 `map_builder_node.cpp`，而是把它的有效流程抽象成更适合 DeepLumin 的管理层设计。

`mapping_manager` 的核心职责是：

- 面向 GUI 提供统一控制接口
- 在开始建图前做环境检测与状态检测
- 启动、监控、停止建图运行时
- 管理 bag 录制生命周期
- 协调结果保存
- 对外发布统一的任务状态和能力状态

## 2. 为什么需要 `mapping_manager`

当前 `slam_odometry` 更接近算法节点，不适合直接承接：

- GUI 请求
- 任务生命周期管理
- 运行模式切换
- 录包控制
- 结果汇总
- 错误恢复

同时，GUI 也不应该直接：

- 调 `roslaunch`
- 起停 `rosbag record`
- 直接 kill 建图节点
- 直接决定地图保存是否完成

因此需要单独的一层任务编排器，即 `mapping_manager`。

## 3. 对 `mapping_builder` 的复用分析

`FASTLIO2_SAM_LC_ws/src/FASTLIO2_SAM_LC/src/map_builder_node.cpp` 提供了几个很有价值的参考点：

1. 主线程负责：
   - 传感器同步
   - LIO 建图
   - 位姿发布
   - 当前点云发布
   - 关键帧插入

2. 独立回环线程负责：
   - 回环候选搜索
   - 子图构建
   - ICP 验证
   - 图优化
   - 回环约束更新

3. 已有地图保存接口：
   - `/save_map` 服务
   - shutdown 自动保存

4. 已有可视化输出：
   - `/local_cloud`
   - `/body_cloud`
   - `/local_path`
   - `/global_path`
   - `/loop_mark`

5. 已有关键帧历史：
   - `key_poses`
   - `cloud_history`

这说明：

- `mapping_builder` 的运行流非常适合做 DeepLumin 建图 runtime 的参考
- 但它本身不是“任务管理器”
- 它更适合作为 `slam_mapping_node` 的设计参考，而不是 `mapping_manager` 的直接替代

因此建议复用方式是：

### 复用 `mapping_builder` 的“流程思想”

- 主线程建图
- 子线程回环
- 服务式地图保存
- 停机自动保存
- 关键帧历史缓存
- 回环 Marker 发布

### 不直接复用的部分

- GUI 交互逻辑
- 任务状态机
- bag 录制控制
- 环境检测
- 模式管理
- 输出目录编排

## 4. `mapping_manager` 在系统中的位置

建议系统结构如下：

```text
GUI Mapping Console
        |
        | service / topic
        v
mapping_manager
        |
        +-- mapping runtime launcher
        |      +-- slam_mapping_node
        |      +-- optional loop_closure_node
        |      +-- optional rviz bridge
        |
        +-- bag recorder controller
        |
        +-- result validator / exporter coordinator
```

其中：

- GUI 只与 `mapping_manager` 交互
- `mapping_manager` 统一调度后端所有建图相关模块
- `slam_mapping_node` 只负责算法与保存

## 5. 包与模块建议

建议新增包：

```text
DeepLumin/src/localization/mapping_manager/
```

建议内部结构：

```text
mapping_manager/
├── config/
│   └── mapping_manager.yaml
├── include/mapping_manager/
│   ├── common/
│   │   ├── types.hpp
│   │   └── config.hpp
│   ├── core/
│   │   ├── manager_core.hpp
│   │   ├── environment_checker.hpp
│   │   ├── runtime_controller.hpp
│   │   ├── bag_controller.hpp
│   │   └── result_validator.hpp
│   └── ros/
│       └── ros_adapter.hpp
├── launch/
│   └── mapping_manager.launch
└── src/
    ├── mapping_manager_node.cpp
    ├── core/
    └── ros/
```

## 6. 核心职责划分

## 6.1 `mapping_manager_node`

职责：

- ROS 节点入口
- 服务注册
- 状态 topic 发布
- 订阅 runtime 状态
- 驱动 `ManagerCore`

## 6.2 `ManagerCore`

职责：

- 维护建图任务状态机
- 接收来自 GUI 的请求
- 根据当前状态做合法性判断
- 驱动环境检测、运行时控制、录包控制、结果保存流程

## 6.3 `EnvironmentChecker`

职责：

- 检查 ROS Master
- 检查 IMU / LiDAR 数据流
- 检查磁盘空间
- 检查配置文件
- 检查路径可写性
- 检查当前是否已有建图任务或录包任务残留

## 6.4 `RuntimeController`

职责：

- 启动 mapping runtime
- 监控 runtime 是否存活
- 发送开始/停止/保存命令给 `slam_mapping_node`
- 读取 runtime 能力标志

这里建议复用 `mapping_builder` 的建图流程，但不在 manager 内部直接实现算法。

## 6.5 `BagController`

职责：

- 启动 rosbag 录制
- 停止 rosbag 录制
- 汇总 bag 保存路径
- 上报录制状态

## 6.6 `ResultValidator`

职责：

- 检查 PCD 是否生成
- 检查轨迹文件是否生成
- 检查关键帧/关键帧位姿是否生成
- 汇总输出结果
- 生成结果摘要

## 7. 状态机设计

建议 `mapping_manager` 的主状态机如下：

```text
IDLE
  -> CHECKING_ENV
  -> READY
  -> STARTING
  -> RUNNING
  -> STOPPING
  -> SAVING_RESULTS
  -> FINISHED
  -> FAILED
```

### 7.1 状态定义

#### `IDLE`

无活跃建图任务。

#### `CHECKING_ENV`

执行环境检测与状态检测。

#### `READY`

检测通过，允许开始建图。

#### `STARTING`

正在启动建图 runtime，等待进入正常运行态。

#### `RUNNING`

建图进行中。

#### `STOPPING`

已收到结束建图请求，正在停止建图并准备保存。

#### `SAVING_RESULTS`

正在保存 PCD、轨迹、关键帧、位姿、bag 等结果。

#### `FINISHED`

任务成功结束。

#### `FAILED`

任务失败，等待用户重新检测或重新开始。

### 7.2 子状态机：录包

建议单独维护录包状态：

```text
REC_IDLE -> REC_STARTING -> REC_RUNNING -> REC_STOPPING -> REC_SAVED / REC_FAILED
```

### 7.3 状态切换事件

建议核心事件：

- `CheckEnvironment`
- `EnvironmentPassed`
- `EnvironmentFailed`
- `StartMapping`
- `RuntimeStarted`
- `RuntimeStartFailed`
- `StopMapping`
- `RuntimeStopped`
- `SaveStarted`
- `SaveSucceeded`
- `SaveFailed`
- `RuntimeCrashed`
- `StartRecording`
- `StopRecording`

## 8. GUI 与 `mapping_manager` 的对接

## 8.1 联动原则

GUI 不直接控制算法节点，只调用 `mapping_manager` 服务。  
`mapping_manager` 是 GUI 的唯一后端入口。

## 8.2 建议服务接口

建议新增以下服务：

- `CheckMappingEnvironment.srv`
- `GetMappingStatus.srv`
- `StartMapping.srv`
- `StopMapping.srv`
- `StartBagRecording.srv`
- `StopBagRecording.srv`

建议状态 topic：

- `/mapping/status`
- `/mapping/record_status`
- `/mapping/environment_last_result`

## 8.3 GUI 到后端的典型时序

### 环境检测

```text
GUI --CheckMappingEnvironment--> mapping_manager
mapping_manager --run checks--> checker/runtime state
mapping_manager --return result--> GUI
GUI --show table--> user
```

### 开始建图

```text
GUI --StartMapping--> mapping_manager
mapping_manager --validate state--> ManagerCore
mapping_manager --launch runtime--> RuntimeController
mapping_manager --optional start bag--> BagController
runtime --publish RUNNING--> /mapping/status
GUI --subscribe--> update UI
```

### 结束建图

```text
GUI --StopMapping--> mapping_manager
mapping_manager --send stop/save--> slam_mapping_node
slam_mapping_node --save outputs--> disk
mapping_manager --validate outputs--> ResultValidator
mapping_manager --publish FINISHED--> /mapping/status
GUI --show result paths--> user
```

## 9. `mapping_manager` 与建图 runtime 的接口设计

## 9.1 为什么需要 runtime 内部接口

如果只靠 `roslaunch` 起停进程，`mapping_manager` 只能做粗粒度控制，无法做到：

- 优雅停止
- 保存进度感知
- 能力发现
- 获取关键帧/回环支持情况

因此建议 `slam_mapping_node` 对外提供明确接口。

## 9.2 建议 runtime 服务

建议 `slam_mapping_node` 提供：

- `StartSession.srv`
- `StopAndSave.srv`
- `GetRuntimeStatus.srv`

如果运行时由 `mapping_manager` 启动后自动进入工作状态，那么 `StartSession.srv` 可以简化为“初始化本次任务参数”。

## 9.3 建议 runtime 状态 topic

- `/mapping/runtime_status`
- `/mapping/runtime_capabilities`

其中 `runtime_capabilities` 建议包含：

- `loop_supported`
- `keyframe_export_supported`
- `trajectory_export_supported`
- `bag_record_supported`

这能让 `mapping_manager` 和 GUI 都按能力做自适应。

## 10. 复用 `mapping_builder` 的流程建议

## 10.1 建图主流程复用

参考 `map_builder_node`，建议 `slam_mapping_node` 内部采用类似结构：

### 主线程

- IMU / LiDAR 数据同步
- 调用 LIO core 建图
- 输出当前点云与位姿
- 插入关键帧
- 发布局部路径/全局路径

### 子线程

- 回环候选搜索
- 子图构建
- ICP/匹配验证
- 图优化
- 更新全局关键帧位姿
- 发布回环 Marker

这部分设计思想可以直接复用。

## 10.2 保存流程复用

参考 `map_builder_node`：

- 提供 `/save_map` 服务
- 退出时自动保存
- 根据关键帧历史重建最终全局地图

在 DeepLumin 中建议扩展为：

- 保存 `global_map.pcd`
- 保存 `trajectory.tum`
- 若支持则保存 `keyframes/`
- 若支持则保存 `keyframe_poses.txt`
- 若支持则保存回环约束文件

## 10.3 可视化流程复用

参考 `map_builder_node` 已有的：

- 点云发布
- 局部路径发布
- 全局路径发布
- `loop_mark` Marker 发布

在 DeepLumin 中建议统一命名为 `/mapping/*` 命名空间，便于 GUI 订阅和隔离。

## 11. 主题与服务建议

## 11.1 GUI 与 `mapping_manager`

### 服务

| 服务 | 用途 |
|------|------|
| `CheckMappingEnvironment` | 环境与状态检测 |
| `GetMappingStatus` | 查询当前建图状态 |
| `StartMapping` | 创建并启动建图任务 |
| `StopMapping` | 停止建图并保存 |
| `StartBagRecording` | 启动录包 |
| `StopBagRecording` | 停止录包并保存 |

### Topic

| Topic | 用途 |
|------|------|
| `/mapping/status` | 主状态机状态 |
| `/mapping/record_status` | 录包状态 |
| `/mapping/environment_last_result` | 最近一次检测结果 |
| `/mapping/events` | 事件流/日志 |

## 11.2 `mapping_manager` 与 runtime

### 服务

| 服务 | 用途 |
|------|------|
| `/mapping/runtime/start_session` | 初始化当前建图任务 |
| `/mapping/runtime/stop_and_save` | 停止并保存 |
| `/mapping/runtime/get_status` | 查询 runtime 状态 |

### Topic

| Topic | 用途 |
|------|------|
| `/mapping/runtime_status` | runtime 状态 |
| `/mapping/runtime_capabilities` | 能力标志 |
| `/mapping/map_points` | 实时地图 |
| `/mapping/current_scan` | 当前扫描 |
| `/mapping/path` | 当前轨迹 |
| `/mapping/keyframes` | 关键帧点位或点云 |
| `/mapping/loop_markers` | 回环可视化 |

## 12. 结果输出管理

`mapping_manager` 应统一管理输出目录。

建议输出结构：

```text
<output_root>/<task_id>/
  map/
    global_map.pcd
    trajectory.tum
    keyframe_poses.txt
    keyframes/
    loop_edges.json
  bag/
    mapping.bag
  config/
    task_config.yaml
  meta/
    task_metadata.json
  logs/
```

## 13. 能力协商设计

由于当前后端能力并不完全确定，`mapping_manager` 需要做一层能力协商。

建议在 runtime 启动后读取能力标志：

- 是否支持关键帧导出
- 是否支持关键帧位姿导出
- 是否支持回环显示
- 是否支持轨迹导出

`mapping_manager` 将这些能力透传给 GUI，GUI 再决定：

- 显示哪些结果项
- 启用哪些功能按钮
- 是否提示“当前版本不支持”

## 14. 失败场景处理

建议 `mapping_manager` 至少处理以下异常：

1. 环境检测失败
2. runtime 启动失败
3. runtime 启动后无有效传感器流
4. runtime 中途崩溃
5. 保存结果失败
6. bag 录制失败
7. bag 停止失败
8. 重复启动请求
9. 运行中再次开始请求
10. 空闲时结束请求

处理原则：

- 保持状态机一致
- 明确给 GUI 返回失败原因
- 尽量保留中间结果和日志

## 15. 与现有定位系统的关系

第一版建议将建图模式与定位模式严格分离。

即：

- `mapping_manager` 负责建图模式
- `localization_manager` 负责定位模式

理由：

- 建图是任务式工作流
- 定位是持续在线工作流
- 二者状态机和资源占用特征不同

后续如果需要统一到更上层系统 manager，可以再做模式编排融合。

## 16. 第一版实现建议

建议先做最小闭环：

1. `mapping_manager` 服务与状态 topic
2. `EnvironmentChecker`
3. `RuntimeController`
4. `BagController`
5. `ResultValidator`
6. 与 `slam_mapping_node` 的最小保存接口

优先保证：

- GUI 能控制
- 建图能运行
- 结束能保存
- bag 能录制
- 状态一致

## 17. 结论

`mapping_manager` 的核心价值不是做算法，而是把 GUI、建图 runtime、录包、保存和结果校验组织成一条稳定的任务链路。

对于当前 DeepLumin 项目，最合理的路线是：

1. 复用 `mapping_builder` 的流程思想，设计 `slam_mapping_node`
2. 新增 `mapping_manager` 承担任务编排
3. 让 GUI 只对接 `mapping_manager`
4. 用统一状态与能力消息把前后端联动起来

这样可以做到：

- GUI 简洁
- 后端职责清晰
- 算法可替换
- 功能可逐步增强

## 18. 后续建议

这份文档之后，下一步最值得继续细化的是：

1. `mapping_manager` 的服务与消息定义
2. `mapping_manager` 状态消息字段设计
3. `slam_mapping_node` 内部保存流程设计
4. `mapping_runtime.launch` 结构设计
