# DeepLumin 新手入门指南

> 本文面向第一次参与井工矿无人驾驶项目开发的工程师，从零开始讲解系统架构、核心机制和编码方法。不需要你有 ROS 经验，但需要有基本的 C++ 和 Linux 知识。

---

## 目录

1. [系统整体架构](#1-系统整体架构)
2. [ROS 核心机制（5分钟速成）](#2-ros-核心机制5分钟速成)
3. [消息包怎么用](#3-消息包怎么用)
4. [开发环境准备](#4-开发环境准备)
5. [第一个节点：Hello ROS](#5-第一个节点hello-ros)
6. [如何添加一个新模块](#6-如何添加一个新模块)
7. [编码规范与最佳实践](#7-编码规范与最佳实践)
8. [编译与调试](#8-编译与调试)
9. [常见问题 FAQ](#9-常见问题-faq)

---

## 1. 系统整体架构

### 1.1 一句话概括

DeepLumin 是一套让无人驾驶车辆在**井下巷道**自主行驶的软件系统。它接收传感器数据（激光雷达、摄像头、IMU），理解周围环境（障碍物在哪），知道自己在哪里（定位），规划怎么走（路径），控制车辆动起来（转向、油门、刹车），同时与地面监控中心保持通信。

### 1.2 模块关系图（简化版）

```
传感器数据流：

  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐
  │   Sensing   │────▶│  Perception │────▶│  Prediction │
  │  传感器驱动  │     │  环境感知    │     │  轨迹预测    │
  └─────────────┘     └─────────────┘     └──────┬──────┘
                                                  │
  ┌─────────────┐     ┌─────────────┐     ┌─────┴─────┐
  │   GNSS/IMU  │────▶│ Localization│     │ Planning  │
  │  定位传感器  │     │   定位融合   │     │  路径规划  │
  └─────────────┘     └──────┬──────┘     └─────┬─────┘
                             │                    │
                             └────────┬───────────┘
                                      ▼
                             ┌─────────────────┐
                             │     Control     │
                             │    运动控制      │
                             └────────┬────────┘
                                      ▼
                             ┌─────────────────┐
                             │ Vehicle Interface│
                             │   车辆CAN接口    │
                             └─────────────────┘
```

**数据流动方向**：从下到上（传感器 → 感知 → 规划 → 控制 → 车辆）

### 1.3 各模块一句话说明

| 模块 | 做什么 | 类比 |
|------|--------|------|
| **Sensing** | 驱动硬件传感器，把原始数据转成 ROS 消息 | 人的眼睛、耳朵 |
| **Perception** | 分析传感器数据，找出障碍物在哪 | 人看到前方有车 |
| **Prediction** | 猜测障碍物接下来会怎么动 | 人预判前车要刹车 |
| **Localization** | 确定自己在地图上的精确位置 | 人知道自己在哪条街 |
| **Planning** | 计算从A到B的最佳路径，同时避开障碍物 | 人导航去目的地 |
| **Control** | 把路径转成方向盘角度和油门大小 | 人手握方向盘、脚踩油门 |
| **Vehicle Interface** | 把控制指令翻译成车辆能懂的CAN信号 | 人的神经传到手脚肌肉 |
| **System** | 监控系统健康、记录数据、与地面通信 | 人的大脑皮层监控身体状态 |
| **Simulation** | 在电脑里模拟井下环境和车辆 | 人在梦里练车 |

### 1.4 为什么这样分层？

想象你要开一家餐厅：
- **Sensing** = 采购员（买食材）
- **Perception** = 洗菜切菜（处理食材）
- **Prediction** = 预判客人点啥菜
- **Localization** = 知道自己在厨房哪个位置
- **Planning** = 厨师长安排做菜顺序
- **Control** = 厨师炒菜的具体动作
- **Vehicle Interface** = 服务员把菜端给客人
- **System** = 老板监控营业额、存录像、接电话订餐

**关键原则**：每层只做一件事，层与层之间通过**标准消息**通信。换厨师（换算法）不需要换服务员（车辆接口）。

---

## 2. ROS 核心机制（5分钟速成）

### 2.1 ROS 是什么？

ROS（Robot Operating System）不是真正的操作系统，而是一套**让多个程序互相通信的框架**。类比：微信群里大家发消息，ROS 就是微信群。

### 2.2 五个核心概念

#### 概念 1：节点（Node）

**节点** = 一个独立运行的程序。

```bash
# 每个 rosrun 启动的就是一个节点
rosrun perception pointcloud_filter    # 启动一个"点云滤波"节点
rosrun localization pose_fusion_node   # 启动一个"位姿融合"节点
```

每个节点只做一件事。节点之间互相不认识，只通过**话题**发消息。

#### 概念 2：话题（Topic）

**话题** = 微信群聊的名称。节点在话题上**发布（Publish）**或**订阅（Subscribe）**消息。

```
话题：/points_raw（原始点云数据）

  lidar_driver 节点 ──发布──▶ /points_raw
                                    │
                                    ▼
                        pointcloud_filter 节点 ──订阅── /points_raw
                        ground_segmentation 节点 ──订阅── /points_raw
```

**特点**：
- 一个话题可以有多个发布者（少见）和多个订阅者（常见）
- 发布者和订阅者互相不知道对方存在（松耦合）
- 话题名用 `/` 开头，如 `/obstacles`、`/localization/fused_pose`

#### 概念 3：消息（Message）

**消息** = 微信群里发的消息内容。ROS 自带很多标准消息，我们也可以定义自己的消息。

```cpp
// 标准消息：激光雷达点云
sensor_msgs/PointCloud2 points;

// 自定义消息：障碍物列表（定义在 deeplumin_msgs 包中）
deeplumin_msgs/ObstacleArray obstacles;
```

消息的定义写在 `.msg` 文件中，编译后自动生成 C++ 类。

#### 概念 4：服务（Service）

**服务** = 一对一的问答。节点 A **请求（Request）**，节点 B **响应（Response）**。

```
节点 A: "请告诉我当前系统状态"
            │──请求──▶
            │         │  system_monitor 节点
            │◀──响应──│  "状态正常，定位精度 0.1m"
```

服务适合**偶尔调用**的操作（如重置、查询状态），不适合高频数据流。

#### 概念 5：参数（Parameter）

**参数** = 节点的配置项，存在 ROS 的参数服务器上。

```bash
# 查看参数
rosparam list
rosparam get /planning/max_speed

# 设置参数
rosparam set /planning/max_speed 3.0
```

参数的好处：修改配置不需要重新编译代码。

### 2.3 TF 坐标变换

ROS 中不同传感器安装在车辆不同位置，需要知道它们的相对位置关系。TF 就是干这个的。

```
map（全局地图坐标系）
  └── odom（里程计坐标系）
        └── base_link（车体中心）
              ├── lidar_link（激光雷达安装点）
              ├── camera_link（摄像头安装点）
              └── imu_link（IMU安装点）
```

**为什么需要 TF？**

激光雷达检测到障碍物在 `lidar_link` 坐标系下坐标是 `(2, 0, 0)`，但规划模块需要知道它在 `base_link` 坐标系下的位置。TF 自动帮你做坐标转换。

```cpp
// C++ 代码中查询 TF 变换
tf2_ros::Buffer tf_buffer;
geometry_msgs::TransformStamped transform = 
    tf_buffer.lookupTransform("base_link", "lidar_link", ros::Time(0));
```

---

## 3. 消息包怎么用

### 3.1 消息包结构

所有自定义消息统一放在 `deeplumin_msgs` 包中，按功能域分目录：

```
deeplumin_msgs/msg/
├── perception/
│   ├── Obstacle.msg        # 单个障碍物
│   ├── ObstacleArray.msg   # 障碍物数组
│   └── FilteredPointCloud.msg
├── localization/
│   ├── SlamPose.msg
│   ├── FusedOdometry.msg
│   ├── LocalizationStatus.msg
│   └── SlamHealth.msg
├── planning/
│   ├── PathState.msg
│   └── SpeedProfile.msg
├── control/
│   ├── ControlCommand.msg
│   └── ControlStatus.msg
└── system/
    ├── SystemStatus.msg
    └── VehicleStatus.msg
```

### 3.2 在代码中使用消息

**Step 1：包含头文件**

```cpp
#include <deeplumin_msgs/ObstacleArray.h>
#include <deeplumin_msgs/ControlCommand.h>
```

**Step 2：发布消息**

```cpp
// 创建发布者
ros::Publisher pub = nh.advertise<deeplumin_msgs::ObstacleArray>("/obstacles", 10);

// 构造消息
deeplumin_msgs::ObstacleArray msg;
msg.header.stamp = ros::Time::now();
msg.header.frame_id = "base_link";

// 添加障碍物
deeplumin_msgs::Obstacle obs;
obs.id = 1;
obs.position.x = 5.0;
obs.position.y = 2.0;
obs.position.z = 0.0;
obs.confidence = 0.95;
msg.obstacles.push_back(obs);

// 发布
pub.publish(msg);
```

**Step 3：订阅消息**

```cpp
// 回调函数
void obstacleCallback(const deeplumin_msgs::ObstacleArray::ConstPtr& msg)
{
    ROS_INFO("收到 %zu 个障碍物", msg->obstacles.size());
    for (const auto& obs : msg->obstacles) {
        ROS_INFO("  障碍物 %u: 位置(%.2f, %.2f, %.2f), 置信度 %.2f",
                 obs.id, obs.position.x, obs.position.y, obs.position.z, obs.confidence);
    }
}

// 创建订阅者
ros::Subscriber sub = nh.subscribe("/obstacles", 10, obstacleCallback);
```

**关键规则**：
- 消息头文件路径：`deeplumin_msgs/消息名.h`
- C++ 类名：`deeplumin_msgs::消息名`
- 智能指针类型：`deeplumin_msgs::消息名::ConstPtr`
- **所有消息必须包含 `std_msgs/Header header`**（时间戳 + 坐标系 + 序列号）

### 3.3 添加新消息

如果你需要定义新的消息类型：

**Step 1：创建 .msg 文件**

```bash
# 在对应功能域下创建
touch src/deeplumin_msgs/msg/perception/MyNewMessage.msg
```

**Step 2：编写消息定义**

```
std_msgs/Header header
uint32 object_id
geometry_msgs/Point position
float32 confidence
```

**Step 3：更新 CMakeLists.txt**

```cmake
# 在 deeplumin_msgs/CMakeLists.txt 中
add_message_files(
  DIRECTORY msg/perception
  FILES Obstacle.msg ObstacleArray.msg MyNewMessage.msg  # 新增
)
```

**Step 4：重新编译**

```bash
cd /home/ubuntu/G1_workspace/DeepLumin
catkin_make --pkg deeplumin_msgs
```

**Step 5：在代码中使用**

```cpp
#include <deeplumin_msgs/MyNewMessage.h>
```

---

## 4. 开发环境准备

### 4.1 目录结构速览

```
DeepLumin/
├── src/                      # 所有源代码
│   ├── deeplumin_msgs/       # 消息定义（只改这里加新消息）
│   ├── sensing/              # 传感器驱动
│   ├── perception/           # 感知
│   ├── prediction/           # 预测
│   ├── localization/         # 定位
│   ├── planning/             # 规划
│   ├── control/              # 控制
│   ├── vehicle_interface/    # 车辆接口
│   ├── system/               # 系统
│   └── simulation/           # 仿真
├── docs/                     # 文档（你在看的就是这里）
├── build/                    # 编译中间文件（自动生成）
├── devel/                    # 编译结果（自动生成）
└── README.md                 # 项目总览
```

### 4.2 首次编译

```bash
# 1. 进入工作空间
cd /home/ubuntu/G1_workspace/DeepLumin

# 2. 初始化（如果是全新环境）
catkin_init_workspace  # 如果 .catkin_workspace 不存在

# 3. 编译（第一次会比较慢）
catkin_make -DCMAKE_BUILD_TYPE=Release

# 4. 加载环境
source devel/setup.bash
```

### 4.3 每次新开终端必做

```bash
# 加载 ROS 环境
source /opt/ros/noetic/setup.bash

# 加载项目环境
cd /home/ubuntu/G1_workspace/DeepLumin
source devel/setup.bash

# 验证环境
echo $ROS_PACKAGE_PATH
# 应该包含 /home/ubuntu/G1_workspace/DeepLumin/src
```

**建议**：把上面两行 source 加到 `~/.bashrc` 末尾，这样每次开终端自动加载。

---

## 5. 第一个节点：Hello ROS

### 5.1 目标

创建一个最简单的 ROS 节点，每隔 1 秒打印 "Hello from DeepLumin"。

### 5.2 创建节点文件

```bash
# 在 perception 包下创建（作为示例）
touch src/perception/src/hello_node.cpp
```

### 5.3 编写代码

```cpp
#include <ros/ros.h>

int main(int argc, char** argv)
{
    // 1. 初始化 ROS
    ros::init(argc, argv, "hello_node");
    
    // 2. 创建节点句柄（与 ROS 系统通信的入口）
    ros::NodeHandle nh;
    
    // 3. 创建循环频率对象（1Hz = 每秒1次）
    ros::Rate loop_rate(1.0);
    
    // 4. 主循环
    while (ros::ok()) {  // ros::ok() 在 ROS 正常工作时返回 true
        ROS_INFO("Hello from DeepLumin!");
        
        // 处理回调（如果有订阅者的话）
        ros::spinOnce();
        
        // 按频率休眠
        loop_rate.sleep();
    }
    
    return 0;
}
```

### 5.4 修改 CMakeLists.txt

在 `src/perception/CMakeLists.txt` 中添加：

```cmake
add_executable(hello_node src/hello_node.cpp)
add_dependencies(hello_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(hello_node ${catkin_LIBRARIES})
```

### 5.5 编译并运行

```bash
# 编译
cd /home/ubuntu/G1_workspace/DeepLumin
catkin_make

# 加载环境
source devel/setup.bash

# 运行（需要 ROS Master 在运行）
roscore &                    # 启动 ROS Master（后台运行）
rosrun perception hello_node # 运行我们的节点
```

**预期输出**：
```
[ INFO] [1234567890.123456789]: Hello from DeepLumin!
[ INFO] [1234567890.123456789]: Hello from DeepLumin!
[ INFO] [1234567890.123456789]: Hello from DeepLumin!
...
```

### 5.6 代码解释

| 代码 | 作用 |
|------|------|
| `ros::init()` | 初始化 ROS，告诉系统这个节点的名字 |
| `ros::NodeHandle` | 创建节点句柄，用于创建发布者/订阅者/参数等 |
| `ros::Rate` | 控制循环频率，避免 CPU 跑满 |
| `ros::ok()` | 检查 ROS 是否正常运行（按 Ctrl+C 会返回 false） |
| `ros::spinOnce()` | 处理一次回调队列（订阅的消息在这里触发回调） |
| `ROS_INFO()` | 打印信息日志，带时间戳 |

---

## 6. 如何添加一个新模块

### 6.1 场景：我要加一个"水位检测"模块

假设井下需要检测巷道积水深度，需要新增 `water_detection` 包。

### 6.2 步骤详解

**Step 1：创建目录结构**

```bash
cd /home/ubuntu/G1_workspace/DeepLumin/src
mkdir -p water_detection/{src,include/water_detection,launch,config,docs}
```

**Step 2：创建 package.xml**

```xml
<?xml version="1.0"?>
<package format="2">
  <name>water_detection</name>
  <version>1.0.0</version>
  <description>水位检测模块 - 检测井下巷道积水深度</description>
  <maintainer email="developer@example.com">Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>deeplumin_msgs</build_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>deeplumin_msgs</exec_depend>

  <export>
    <build_type>catkin</build_type>
  </export>
</package>
```

**Step 3：创建 CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(water_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  deeplumin_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(water_level_detector src/water_level_detector.cpp)
add_dependencies(water_level_detector ${catkin_EXPORTED_TARGETS})
target_link_libraries(water_level_detector ${catkin_LIBRARIES})

install(TARGETS water_level_detector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

**Step 4：编写节点代码**

```cpp
// src/water_detection/src/water_level_detector.cpp
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Range.h>

class WaterLevelDetector
{
public:
    WaterLevelDetector(ros::NodeHandle& nh)
    {
        // 订阅超声波测距传感器
        sub_range_ = nh.subscribe("/ultrasonic/range", 10, 
                                  &WaterLevelDetector::rangeCallback, this);
        
        // 发布水位深度
        pub_water_level_ = nh.advertise<std_msgs::Float32>("/water_level", 10);
        
        // 从参数服务器读取安装高度
        nh.param<double>("sensor_mount_height", mount_height_, 1.5);
    }

private:
    void rangeCallback(const sensor_msgs::Range::ConstPtr& msg)
    {
        // 水位 = 安装高度 - 测距值
        double water_level = mount_height_ - msg->range;
        
        std_msgs::Float32 result;
        result.data = water_level;
        pub_water_level_.publish(result);
        
        ROS_INFO("当前水位: %.2f m", water_level);
    }

    ros::Subscriber sub_range_;
    ros::Publisher pub_water_level_;
    double mount_height_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "water_level_detector");
    ros::NodeHandle nh("~");  // ~ 表示私有命名空间
    
    WaterLevelDetector detector(nh);
    
    ros::spin();  // 阻塞等待回调
    return 0;
}
```

**Step 5：添加消息（如果需要）**

如果需要新的消息类型（如 `WaterLevelStatus.msg`）：

```bash
# 1. 创建消息文件
touch src/deeplumin_msgs/msg/system/WaterLevelStatus.msg

# 2. 编写内容
# std_msgs/Header header
# float32 water_level
# bool is_safe
# string warning_message

# 3. 更新 deeplumin_msgs/CMakeLists.txt
# 4. 重新编译消息包
```

**Step 6：编译**

```bash
cd /home/ubuntu/G1_workspace/DeepLumin
catkin_make
source devel/setup.bash
```

**Step 7：运行测试**

```bash
roscore &
rosrun water_detection water_level_detector
```

### 6.3  checklist（添加新模块必查）

- [ ] 目录结构完整（src/ include/ launch/ config/ docs/）
- [ ] package.xml 填写正确（name/version/description/依赖）
- [ ] CMakeLists.txt 正确（find_package/add_executable/target_link_libraries）
- [ ] 如果需要新消息，更新 deeplumin_msgs 并重新编译
- [ ] 代码中包含必要的头文件
- [ ] 发布的话题名符合项目规范
- [ ] 节点代码放在 `src/` 目录下
- [ ] 头文件放在 `include/包名/` 目录下

---

## 7. 编码规范与最佳实践

### 7.1 命名规范

| 类型 | 规范 | 示例 |
|------|------|------|
| **类名** | 大驼峰 | `class ObstacleDetector` |
| **函数名** | 小写+下划线 | `void process_pointcloud()` |
| **变量名** | 小写+下划线 | `double max_speed;` |
| **成员变量** | 下划线后缀 | `ros::Publisher pub_obstacles_;` |
| **常量** | 全大写+下划线 | `const double PI = 3.14159;` |
| **话题名** | 小写+下划线 | `/localization/fused_pose` |
| **包名** | 小写+下划线 | `pointcloud_preprocessing` |

### 7.2 节点编写模板

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

class MyNode
{
public:
    MyNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        // 1. 加载参数
        pnh.param<double>("publish_rate", publish_rate_, 10.0);
        pnh.param<std::string>("output_topic", output_topic_, "/output");
        
        // 2. 创建发布者
        pub_ = nh.advertise<std_msgs::String>(output_topic_, 10);
        
        // 3. 创建订阅者
        sub_ = nh.subscribe("/input", 10, &MyNode::inputCallback, this);
        
        // 4. 创建定时器（替代 while 循环）
        timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_), 
                                &MyNode::timerCallback, this);
    }

private:
    void inputCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("收到: %s", msg->data.c_str());
        latest_input_ = msg->data;
    }
    
    void timerCallback(const ros::TimerEvent& event)
    {
        std_msgs::String msg;
        msg.data = "处理结果: " + latest_input_;
        pub_.publish(msg);
    }

    // 成员变量
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Timer timer_;
    double publish_rate_;
    std::string output_topic_;
    std::string latest_input_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");  // 私有命名空间，参数从 /my_node/param 读取
    
    MyNode node(nh, pnh);
    
    ros::spin();  // 阻塞等待回调和定时器
    return 0;
}
```

### 7.3 参数加载规范

```yaml
# config/my_node_params.yaml
my_node:
  publish_rate: 10.0
  output_topic: "/my_output"
  max_value: 100.0
  enabled: true
```

```cpp
// C++ 中读取参数
ros::NodeHandle pnh("~");
pnh.param<double>("publish_rate", publish_rate_, 10.0);
pnh.param<std::string>("output_topic", output_topic_, "/output");
pnh.param<bool>("enabled", enabled_, true);
```

**规则**：
- 所有可调参数必须外部化到 YAML 配置文件
- 参数名用小写+下划线
- 提供合理的默认值
- 布尔参数用 `true`/`false`（不是 `True`/`False`）

### 7.4 日志规范

```cpp
ROS_DEBUG("调试信息，仅在 debug 模式显示");
ROS_INFO("普通信息，默认显示");
ROS_WARN("警告，黄色显示");
ROS_ERROR("错误，红色显示");
ROS_FATAL("致命错误，程序即将退出");
```

**规则**：
- `ROS_DEBUG` 用于开发调试，发布版本自动屏蔽
- `ROS_INFO` 用于关键状态变化（如"开始运行"、"检测到障碍物"）
- `ROS_WARN` 用于异常情况但不影响功能（如"参数未设置，使用默认值"）
- `ROS_ERROR` 用于影响功能的错误（如"传感器数据异常"）
- 不要每个循环都打印 `ROS_INFO`，会刷屏

### 7.5 异常处理

```cpp
try {
    // 可能出错的代码
    auto result = some_risky_operation();
} catch (const std::exception& e) {
    ROS_ERROR("操作失败: %s", e.what());
    // 降级处理：发布空结果或保持上一次有效结果
}
```

---

## 8. 编译与调试

### 8.1 常用编译命令

```bash
# 编译整个工作空间
catkin_make

# 只编译指定包（更快）
catkin_make --pkg deeplumin_msgs
catkin_make --pkg perception

# Release 模式（运行更快）
catkin_make -DCMAKE_BUILD_TYPE=Release

# Debug 模式（带调试符号，用于 gdb）
catkin_make -DCMAKE_BUILD_TYPE=Debug

# 强制重新编译
catkin_make --force-cmake
```

### 8.2 调试方法

**方法 1：ROS 日志**

```bash
# 查看所有节点的日志
roscore &
rosrun rqt_console rqt_console

# 过滤特定级别的日志
rosrun rqt_logger_level rqt_logger_level
```

**方法 2：rostopic 调试**

```bash
# 列出所有话题
rostopic list

# 查看话题内容
rostopic echo /obstacles

# 查看话题发布频率
rostopic hz /obstacles

# 查看话题带宽
rostopic bw /obstacles

# 手动发布测试消息
rostopic pub /test std_msgs/String "data: 'hello'"
```

**方法 3：rviz 可视化**

```bash
rosrun rviz rviz
```

- 添加 PointCloud2 显示点云
- 添加 MarkerArray 显示障碍物框
- 添加 Path 显示规划路径
- 添加 Odometry 显示车辆轨迹

**方法 4：GDB 调试**

```bash
# 编译 Debug 版本
catkin_make -DCMAKE_BUILD_TYPE=Debug

# 用 gdb 启动节点
rosrun --prefix 'gdb -ex run --args' perception pointcloud_filter

# 常用 gdb 命令
(gdb) break main          # 在 main 函数设断点
(gdb) continue            # 继续运行
(gdb) next                # 单步执行（不进入函数）
(gdb) step                # 单步执行（进入函数）
(gdb) print variable      # 打印变量值
(gdb) backtrace           # 查看调用栈
(gdb) quit                # 退出
```

**方法 5：rosbag 录播**

```bash
# 录制所有话题
rosbag record -a -o session.bag

# 录制指定话题
rosbag record /points_raw /image_raw /imu/data -o session.bag

# 回放
rosbag play session.bag

# 查看 bag 文件信息
rosbag info session.bag
```

### 8.3 性能分析

```bash
# 查看节点 CPU/内存占用
rosnode info /node_name

# 系统级性能监控
htop

# ROS 话题延迟分析
rostopic delay /obstacles
```

---

## 9. 常见问题 FAQ

### Q1: catkin_make 报错 "package not found"

**原因**：ROS 找不到你的包。

**解决**：
```bash
# 1. 确保在工作空间根目录
cd /home/ubuntu/G1_workspace/DeepLumin

# 2. 重新加载环境
source devel/setup.bash

# 3. 检查包路径
echo $ROS_PACKAGE_PATH
# 应该包含 /home/ubuntu/G1_workspace/DeepLumin/src

# 4. 如果还不行，检查 package.xml 中的 <name> 是否正确
```

### Q2: 编译报错 "undefined reference to ..."

**原因**：链接时找不到库。

**解决**：
```cmake
# 在 CMakeLists.txt 中确保添加了 target_link_libraries
target_link_libraries(my_node ${catkin_LIBRARIES})

# 如果用了 PCL，还要加
target_link_libraries(my_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
```

### Q3: 节点运行时订阅不到消息

**原因 1**：话题名拼写错误。

```bash
# 检查实际的话题名
rostopic list | grep obstacles
```

**原因 2**：消息类型不匹配。

```bash
# 检查话题的消息类型
rostopic info /obstacles
```

**原因 3**：节点启动顺序问题（发布者在订阅者之后启动）。ROS 话题是持久化的，但 `advertise` 需要一点时间建立连接。可以在 `advertise` 后加延迟：

```cpp
ros::Publisher pub = nh.advertise<...>("...", 10);
ros::Duration(0.5).sleep();  // 等待连接建立
```

### Q4: 修改了 .msg 文件但代码中不生效

**原因**：消息头文件没有重新生成。

**解决**：
```bash
# 1. 先编译消息包
catkin_make --pkg deeplumin_msgs

# 2. 再编译整个项目
catkin_make

# 3. 重新加载环境
source devel/setup.bash
```

### Q5: TF 查询报错 "Lookup would require extrapolation into the future"

**原因**：查询的时间戳比最新 TF 还新。

**解决**：
```cpp
// 方案1：用 ros::Time(0) 获取最新 TF
tf_buffer.lookupTransform("base_link", "lidar_link", ros::Time(0));

// 方案2：增加等待时间
tf_buffer.lookupTransform("base_link", "lidar_link", ros::Time::now(), ros::Duration(0.1));
```

### Q6: 节点 CPU 占用 100%

**原因**：while 循环中没有睡眠。

**解决**：
```cpp
// 错误 ❌
while (ros::ok()) {
    // 干活
}

// 正确 ✅
ros::Rate loop_rate(10);  // 10Hz
while (ros::ok()) {
    // 干活
    ros::spinOnce();
    loop_rate.sleep();    // 自动计算并休眠
}
```

### Q7: 如何同时运行多个节点？

**方法 1：多个终端**

```bash
# 终端1
roscore

# 终端2
rosrun perception pointcloud_filter

# 终端3
rosrun localization pose_fusion_node
```

**方法 2：Launch 文件（推荐）**

```xml
<!-- launch/my_system.launch -->
<launch>
  <!-- 启动 ROS Master（如果没有在运行） -->
  <master auto="start"/>
  
  <!-- 启动多个节点 -->
  <node pkg="perception" type="pointcloud_filter" name="pointcloud_filter" />
  <node pkg="localization" type="pose_fusion_node" name="pose_fusion_node" />
  <node pkg="planning" type="local_planner" name="local_planner" />
  <node pkg="control" type="motion_controller" name="motion_controller" />
</launch>
```

```bash
# 一行命令启动所有节点
roslaunch my_pkg my_system.launch
```

### Q8: 我想在仿真里测试，怎么做？

```bash
# 1. 启动 Gazebo 仿真环境
roslaunch simulation mine_environment.launch

# 2. 启动仿真车辆
roslaunch simulation vehicle_spawn.launch

# 3. 启动算法模块
roslaunch perception perception.launch
roslaunch localization localization.launch
roslaunch planning planning.launch
roslaunch control control.launch

# 4. 在 rviz 中查看
rosrun rviz rviz
```

---

## 附录：推荐阅读

1. [ROS Wiki - 官方教程](http://wiki.ros.org/ROS/Tutorials)（英文，但非常详细）
2. [README.md](../README.md) - 项目总览和模块说明
3. `src/模块名/docs/requirements.md` - 各模块的详细需求
4. `src/模块名/docs/design.md` - 各模块的详细设计
5. `src/deeplumin_msgs/msg/` - 消息定义参考

---

> **最后的话**：不要试图一次性理解整个系统。从一个模块开始，读它的需求文档和设计文档，跑通它的节点，理解它的输入输出，然后再看上下游模块。ROS 的松耦合设计让你可以独立理解每个模块。
