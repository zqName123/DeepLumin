# DeepLumin 可视化建图 GUI 功能设计

## 1. 目标

本文档定义面向用户的可视化建图界面设计，重点回答三个问题：

1. GUI 要提供哪些功能
2. GUI 如何与后端建图系统对接
3. GUI 与后端建图算法、`mapping_manager` 之间如何联动

这里的 GUI 不只是“显示 RViz”，而是一个完整的建图工作台，负责：

- 环境检测
- 状态检测
- 开始建图
- 结束建图
- 录制 bag
- 保存结果
- 展示实时地图、轨迹、回环效果

## 2. 总体定位

GUI 在系统中的定位是：

- 用户交互入口
- 任务控制面板
- 实时状态展示面板
- 可视化结果承载体

GUI 不负责：

- 直接运行建图算法核心
- 直接拼接地图或保存 PCD
- 直接调用 `roslaunch`/`rosbag record` 命令

这些能力统一由后端 `mapping_manager` 负责。

## 3. 系统分层

建议 GUI 所处的系统结构如下：

```text
用户
  |
  v
GUI Mapping Console
  |
  | service / topic
  v
mapping_manager
  |
  +-- mapping runtime
  |     +-- slam_mapping_node
  |     +-- optional loop_closure_node
  |     +-- optional rviz backend / viz bridge
  |
  +-- bag recorder controller
  |
  +-- result exporter
```

对应关系：

- GUI 发起控制请求给 `mapping_manager`
- `mapping_manager` 统一启动、停止和协调各后端节点
- `slam_mapping_node` 持续发布地图、轨迹、状态、可选回环信息
- GUI 订阅这些状态与结果并展示

## 4. GUI 主页面结构

建议 GUI 主页面拆成六个区域。

### 4.1 顶部状态栏

显示全局状态：

- 当前工作模式：`IDLE / READY / RUNNING / SAVING / FAILED`
- 当前任务 ID
- 当前地图任务名称
- 运行时长
- 录包状态
- 输出目录

### 4.2 左侧控制栏

主要按钮建议：

- `检测环境`
- `检测状态`
- `开始建图`
- `结束建图`
- `开始录包`
- `停止录包并保存`
- `打开输出目录`
- `导出结果摘要`

建议补充输入框：

- 地图任务名
- 输出根目录
- 配置文件选择
- 是否启用回环
- 是否启用可视化
- 是否自动录包

### 4.3 环境检测面板

显示开始建图前的检测项和结果：

- ROS Master 在线状态
- IMU 话题可用性
- LiDAR 话题可用性
- 时间同步情况
- 磁盘剩余空间
- 输出路径可写性
- bag 路径可写性
- 配置文件存在性
- 回环模块依赖可用性

每项建议显示：

- `PASS`
- `WARN`
- `FAIL`
- 原因说明

### 4.4 运行状态面板

显示当前建图运行态：

- 建图状态机当前状态
- 当前帧号
- 当前关键帧数
- 当前累计点数
- 当前位姿
- 当前地图保存状态
- 当前回环状态
- 当前录包状态

### 4.5 中央三维可视化面板

这是 GUI 的核心区域，建议显示：

- 实时地图点云
- 当前扫描点云
- 当前轨迹
- 当前位姿坐标轴
- 关键帧位置
- 回环边

实现方式建议二选一：

1. 嵌入 RViz
2. 自研 3D 渲染面板

第一版更推荐嵌入 RViz，原因：

- 成本低
- 稳定
- 与 ROS Topic 对接天然兼容
- Marker/Path/PointCloud2 都可直接显示

### 4.6 底部日志与事件面板

显示：

- 操作日志
- 检测日志
- 状态切换事件
- 回环检测事件
- 保存结果日志
- 录包日志

建议支持：

- 按级别过滤：`INFO / WARN / ERROR`
- 导出日志

## 5. GUI 功能需求详细设计

## 5.1 环境检测功能

### 功能目标

在用户点击“开始建图”前，先验证当前系统是否具备运行建图任务的基础条件。

### 输入

GUI 调用：

- `CheckMappingEnvironment.srv`

### 输出

后端返回结构化检测结果，GUI 展示成表格。

### 建议检测项

1. `ros_master_ok`
2. `imu_topic_ok`
3. `lidar_topic_ok`
4. `imu_rate_ok`
5. `lidar_rate_ok`
6. `time_sync_ok`
7. `disk_space_ok`
8. `output_dir_ok`
9. `bag_dir_ok`
10. `config_ok`
11. `loop_module_ok`
12. `mapping_runtime_not_running`

### GUI 行为

- 若全部 `PASS`：界面置为 `READY`
- 若有 `WARN`：允许开始，但弹出确认
- 若有 `FAIL`：禁止开始建图

## 5.2 状态检测功能

### 功能目标

用于查看系统当前是否已有建图任务、录包任务或异常残留进程。

### 输入

GUI 调用：

- `GetMappingStatus.srv`

### 输出

建议返回：

- 当前建图状态
- 当前任务是否存在
- 录包是否进行中
- 上次任务输出目录
- 最近一次失败原因

### GUI 行为

- 刷新状态面板
- 刷新按钮可用性

## 5.3 开始建图功能

### 功能目标

用户点击“开始建图”后，GUI 发起任务创建请求，后端自动完成建图系统启动。

### GUI 输入项

建议用户可配置：

- 地图任务名
- 输出目录
- 使用的 mapping config
- 是否启用回环
- 是否启用界面可视化
- 是否自动开始录包

### 输入接口

GUI 调用：

- `StartMapping.srv`

### GUI 成功后的联动

- 顶部状态栏切换到 `STARTING`
- 当订阅到 `/mapping/status` 进入 `RUNNING` 后切换到运行态
- 中央地图面板开始显示实时地图
- 轨迹面板开始刷新
- 若启用自动录包，录包状态切换为 `REC_RUNNING`

## 5.4 结束建图功能

### 功能目标

用户点击“结束建图”后，系统优雅结束算法运行并自动保存结果。

### 输入接口

GUI 调用：

- `StopMapping.srv`

### GUI 联动

- 状态切换为 `STOPPING`
- 后端进入 `SAVING_RESULTS`
- GUI 显示保存进度和输出路径
- 成功后切换为 `FINISHED`
- 若失败，切换为 `FAILED`

## 5.5 开始录包功能

### 功能目标

在建图过程中录制指定 topic，便于后期复现。

### 输入接口

GUI 调用：

- `StartBagRecording.srv`

### GUI 联动

- 录包状态切换为 `REC_STARTING` → `REC_RUNNING`
- 在日志面板显示 bag 路径

## 5.6 停止录包并保存功能

### 功能目标

停止录制并将 bag 文件落盘。

### 输入接口

GUI 调用：

- `StopBagRecording.srv`

### GUI 联动

- 录包状态切换为 `REC_STOPPING`
- 保存成功后显示 bag 文件路径

## 5.7 实时地图显示功能

### 目标

在 GUI 中实时可视化当前正在构建的点云地图。

### 推荐订阅主题

- `/mapping/map_points`
- `/mapping/current_scan`
- `/mapping/path`
- `/mapping/odom`

### 显示建议

- 地图点云：白色或灰色
- 当前扫描：绿色
- 当前轨迹：蓝色
- 当前位姿：红色坐标轴

### GUI 与后端联动

- `mapping_manager` 启动成功后，确保这些 topic 开始持续发布
- GUI 订阅后自动刷新显示

## 5.8 回环检测效果显示功能

### 目标

向用户直观展示回环候选、回环成功和回环约束。

### 注意事项

当前 DeepLumin 代码并没有现成完整的在线回环显示链路，因此 GUI 设计必须允许该功能“可选启用”。

### 推荐订阅主题

第一版建议至少预留：

- `/mapping/loop_markers`
- `/mapping/loop_status`
- `/mapping/loop_events`

### 展示方式

- 回环边：绿色线段
- 当前回环候选：黄色提示
- 回环失败：橙色日志
- 回环成功：绿色日志 + 可视化边

### GUI 行为建议

- 若后端声明 `loop_supported=false`，GUI 显示“当前版本未启用回环展示”
- 若 `loop_supported=true`，显示回环面板和统计信息

## 5.9 保存结果展示功能

GUI 在任务结束后应展示：

- 地图 PCD 路径
- bag 文件路径
- 轨迹文件路径
- 关键帧目录路径
- 关键帧位姿文件路径
- 回环结果文件路径

但其中后 3 项应根据后端能力动态显示。

GUI 不应写死要求这些文件必定存在。

## 6. GUI 与后端的联动设计

## 6.1 联动原则

建议遵循以下原则：

1. GUI 只发请求，不直接控进程
2. 所有任务状态以 `mapping_manager` 发布状态为准
3. GUI 不缓存“推断状态”，只显示后端真实状态
4. 可视化数据一律通过 ROS topic 获取
5. 控制面和数据面分离

## 6.2 控制面接口

建议 GUI 使用 service 调后端。

### 服务建议

- `CheckMappingEnvironment.srv`
- `GetMappingStatus.srv`
- `StartMapping.srv`
- `StopMapping.srv`
- `StartBagRecording.srv`
- `StopBagRecording.srv`

### GUI 调用时机

- 点击按钮时调用
- 页面初始化时调用 `GetMappingStatus`
- 定时刷新时只依赖状态 topic，不轮询所有服务

## 6.3 状态面接口

建议 `mapping_manager` 周期发布：

- `/mapping/status`
- `/mapping/record_status`
- `/mapping/environment_last_result`

GUI 订阅这些 topic，用于刷新：

- 顶部状态栏
- 按钮可用性
- 运行状态面板
- 日志面板

## 6.4 数据面接口

建议后端发布以下数据给 GUI：

- `/mapping/map_points`
- `/mapping/current_scan`
- `/mapping/path`
- `/mapping/odom`
- `/mapping/keyframes`
- `/mapping/loop_markers`

GUI 不参与这些数据生成，只负责显示。

## 7. GUI 按钮与后端接口映射

| GUI 按钮 | 后端接口 | 预期后端行为 | GUI 后续行为 |
|----------|----------|--------------|--------------|
| 检测环境 | `CheckMappingEnvironment.srv` | 检查传感器、配置、磁盘、目录、运行状态 | 显示检测表 |
| 检测状态 | `GetMappingStatus.srv` | 返回当前任务状态 | 刷新状态栏 |
| 开始建图 | `StartMapping.srv` | 创建任务、启动 runtime、可选自动录包 | 切换到 `STARTING` |
| 结束建图 | `StopMapping.srv` | 停止建图、保存结果 | 切换到 `STOPPING`/`SAVING_RESULTS` |
| 开始录包 | `StartBagRecording.srv` | 启动 rosbag 录制 | 录包状态显示 `RUNNING` |
| 停止录包并保存 | `StopBagRecording.srv` | 停止 rosbag 并落盘 | 显示 bag 路径 |
| 打开输出目录 | 无 ROS 接口或本地文件浏览 | 打开目录 | 显示结果 |

## 8. GUI 状态驱动规则

建议 GUI 不是“按钮一按就认为成功”，而是按后端状态驱动。

示例：

### 开始建图

1. 用户点击 `开始建图`
2. GUI 调用 `StartMapping.srv`
3. 若服务返回 `accepted=true`
   - GUI 进入等待态
4. GUI 订阅到 `/mapping/status.state=RUNNING`
   - 才真正显示“建图中”

### 结束建图

1. 用户点击 `结束建图`
2. GUI 调用 `StopMapping.srv`
3. 状态经过：
   - `STOPPING`
   - `SAVING_RESULTS`
   - `FINISHED`
4. GUI 最终显示结果路径

这样能避免前端和后端状态不一致。

## 9. GUI 需要感知的后端能力标志

建议后端在状态消息中显式告诉 GUI：

- `loop_supported`
- `keyframe_export_supported`
- `trajectory_export_supported`
- `bag_record_supported`
- `embedded_viz_supported`

原因：

- GUI 不需要猜后端支持什么
- 可以根据能力动态显示或隐藏功能区

例如：

- 若 `loop_supported=false`
  则回环面板置灰
- 若 `keyframe_export_supported=false`
  则结果页不显示关键帧保存项

## 10. GUI 错误处理设计

建议 GUI 对以下情况给出明确反馈：

1. 环境检测失败
2. 开始建图请求被拒绝
3. 建图运行时崩溃
4. 保存地图失败
5. 录包启动失败
6. 录包停止失败
7. 可视化数据中断

反馈方式：

- 状态栏红色告警
- 日志面板错误消息
- 弹窗提示

## 11. 第一版推荐实现策略

第一版 GUI 功能建议按优先级做：

### P0 必做

- 环境检测
- 状态检测
- 开始建图
- 结束建图
- 实时地图显示
- 录包开始/停止
- 结果路径展示

### P1 推荐

- 关键帧显示
- 保存结果摘要
- 运行时统计

### P2 增强

- 回环检测效果显示
- 优化前后轨迹对比
- 地图结果预览

## 12. 与后端建图算法的对接总结

GUI 到后端建图算法的路径建议固定为：

```text
GUI
  -> service 调用 mapping_manager
  -> mapping_manager 启动/控制 slam_mapping_node
  -> slam_mapping_node 发布地图/轨迹/回环数据
  -> GUI 订阅并可视化
```

这个设计的优点是：

- GUI 和算法核心解耦
- GUI 不需要关心具体线程、回环、保存细节
- 后端可替换算法而 GUI 接口保持稳定

## 13. 结论

GUI 功能设计的核心不是把所有后端能力都塞进界面，而是建立一套稳定的控制面和显示面。

建议最终 GUI 设计遵循以下原则：

1. 控制统一走 `mapping_manager`
2. 可视化统一走 ROS 数据 topic
3. 后端能力通过能力标志显式暴露给 GUI
4. 界面功能区按“必需/推荐/可选”分层展示

对于当前 DeepLumin 项目，第一版最合理的 GUI 落地方向是：

- 一个面向操作员的建图工作台
- 嵌入 RViz 或使用 RViz 作为实时可视化后端
- 服务调用控制建图和录包
- 状态 topic 驱动界面状态

## 14. 后续可继续细化

下一步最值得继续细化的是：

1. GUI 页面原型图和布局规范
2. GUI 所需消息和服务精确定义
3. RViz 嵌入方案或自研 3D 面板方案
4. GUI 能力标志字段设计
