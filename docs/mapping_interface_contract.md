# DeepLumin 建图 GUI / Manager / Runtime 接口契约

## 1. 目的

本文档用于固定三者之间的接口边界：

- `mapping_gui`
- `mapping_manager`
- `slam_mapping_node`

这里的原则是：

- GUI 只做控制和状态展示
- GUI 不做点云三维显示
- RViz 由后端 runtime 启动
- `mapping_manager` 是 GUI 唯一后端入口
- `slam_mapping_node` 是建图 runtime 核心

## 2. 角色分工

## 2.1 `mapping_gui`

负责：

- 检测环境
- 查询状态
- 开始建图
- 结束建图
- 开始录包
- 停止录包
- 显示任务状态和结果路径

不负责：

- 启动 RViz
- 拼接点云地图
- 保存 PCD
- 关键帧管理
- 回环计算

## 2.2 `mapping_manager`

负责：

- 接收 GUI 请求
- 做环境检测和状态检测
- 启停 `slam_mapping_node` 及相关 runtime
- 启停 rosbag record
- 请求 runtime 停止和保存
- 汇总状态与结果

## 2.3 `slam_mapping_node`

负责：

- 执行建图算法
- 发布实时地图和状态
- 可选做关键帧和回环
- 保存地图、轨迹、关键帧等结果
- 可选启动 RViz

## 3. 控制面接口：GUI -> `mapping_manager`

## 3.1 服务

### `/mapping/check_environment`

类型：`deeplumin_msgs/CheckMappingEnvironment`

用途：

- 检查传感器
- 检查磁盘
- 检查路径
- 检查当前是否已有建图任务在运行

### `/mapping/get_status`

类型：`deeplumin_msgs/GetMappingStatus`

用途：

- 获取当前建图状态
- 获取录包状态
- 获取最近一次环境检测结果

### `/mapping/start`

类型：`deeplumin_msgs/StartMapping`

用途：

- 创建新的建图任务
- 启动 runtime
- 可选自动开启录包
- 可选要求 runtime 启动 RViz

### `/mapping/stop`

类型：`deeplumin_msgs/StopMapping`

用途：

- 结束建图
- 请求保存地图与相关结果

### `/mapping/start_bag_recording`

类型：`deeplumin_msgs/StartBagRecording`

用途：

- 启动 bag 录制

### `/mapping/stop_bag_recording`

类型：`deeplumin_msgs/StopBagRecording`

用途：

- 停止 bag 录制并保存

## 4. 状态面接口：`mapping_manager` -> GUI

## 4.1 Topic

### `/mapping/status`

类型：`deeplumin_msgs/MappingStatus`

字段用途：

- 当前任务状态
- 当前任务路径
- 关键帧、回环、轨迹等导出路径
- 基本统计计数
- 能力标志

### `/mapping/record_status`

类型：`deeplumin_msgs/MappingRecordStatus`

字段用途：

- 录包状态
- bag 路径
- 写入字节数
- 当前录制时长

### `/mapping/environment_last_result`

类型：`deeplumin_msgs/MappingEnvironmentStatus`

字段用途：

- 最近一次环境检测结果
- 检测项列表

## 5. `mapping_manager` -> `slam_mapping_node`

建议后续实现以下 runtime 接口。

## 5.1 服务

### `/mapping/runtime/start_session`

用途：

- 初始化当前建图任务
- 传入任务 ID、输出目录、配置、功能开关

### `/mapping/runtime/stop_and_save`

用途：

- 请求停止当前建图任务
- 请求保存地图与结果

### `/mapping/runtime/get_status`

用途：

- 查询 runtime 当前状态和能力

## 5.2 Topic

### `/mapping/runtime_status`

用途：

- 发布 runtime 当前状态

### `/mapping/runtime_capabilities`

用途：

- 发布能力标志：
  - 是否支持关键帧导出
  - 是否支持关键帧位姿导出
  - 是否支持回环显示
  - 是否支持轨迹导出
  - 是否支持自动保存

## 6. 可视化面接口：`slam_mapping_node` / runtime -> RViz

注意：这些接口不由 GUI 消费，而由 RViz 消费。

建议 runtime 发布：

- `/mapping/map_points`
- `/mapping/current_scan`
- `/mapping/path`
- `/mapping/path_global`
- `/mapping/keyframe_markers`
- `/mapping/loop_markers`

GUI 只显示“RViz 由 runtime 启动”的状态，不承担这些数据展示。

## 7. 关键约束

1. GUI 只调 `mapping_manager`，不直接调 `slam_mapping_node`
2. GUI 不直接启动 `rviz`
3. `mapping_manager` 决定是否要求 runtime 启动 RViz
4. 是否支持关键帧保存、回环展示，必须由 runtime 能力标志决定
5. GUI 不应预设“关键帧和回环一定存在”

## 8. 当前已落地的接口定义文件

在 `deeplumin_msgs` 中已新增：

### Msg

- `MappingEnvironmentItem.msg`
- `MappingEnvironmentStatus.msg`
- `MappingStatus.msg`
- `MappingRecordStatus.msg`

### Srv

- `CheckMappingEnvironment.srv`
- `GetMappingStatus.srv`
- `StartMapping.srv`
- `StopMapping.srv`
- `StartBagRecording.srv`
- `StopBagRecording.srv`

## 9. 当前 GUI 骨架

当前 `mapping_gui` 已按上述契约做了第一版最小实现：

- 服务调用按钮
- 状态订阅面板
- 环境检测结果显示
- 结果路径显示
- 事件日志显示

当前 GUI 不包含：

- 嵌入式 RViz
- 点云三维渲染
- 回环图形化展示

这些由 runtime 启动的 RViz 负责。
