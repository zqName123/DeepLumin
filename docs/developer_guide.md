# DeepLumin 开发指南

> 本文面向参与 DeepLumin 项目开发的工程师，介绍项目架构设计原则、子包组织方式、统一依赖管理、CUDA/TensorRT 集成以及扩展开发流程。

---

## 目录

1. [项目架构总览](#1-项目架构总览)
2. [子包组织规范](#2-子包组织规范)
3. [统一依赖管理（deeplumin_cmake）](#3-统一依赖管理deeplumin_cmake)
4. [CUDA / TensorRT 集成指南](#4-cuda--tensorrt-集成指南)
5. [消息定义规范](#5-消息定义规范)
6. [添加新子包完整流程](#6-添加新子包完整流程)
7. [编译与 CI](#7-编译与-ci)
8. [代码审查 checklist](#8-代码审查-checklist)

---

## 1. 项目架构总览

### 1.1 设计哲学

DeepLumin 采用 **"功能域聚合、子包独立"** 的组织方式：

- **功能域（Domain）**：按无人驾驶技术栈划分的大模块，如 `perception/`、`localization/`
- **子包（Package）**：功能域下的独立 ROS 包，每个子包有独立的 `package.xml` 和 `CMakeLists.txt`
- **节点（Node）**：子包内的可执行程序，通过 ROS 话题/服务通信

```
src/                                    # CATKIN 工作空间源码根目录
├── deeplumin_cmake/                    # 统一 CMake 依赖管理
├── deeplumin_msgs/                     # 统一消息定义
├── sensing/                            # 功能域：传感器驱动
│   ├── lidar_driver/                   # 子包：激光雷达驱动
│   ├── camera_driver/                  # 子包：摄像头驱动
│   ├── imu_driver/                     # 子包：IMU驱动
│   ├── gnss_driver/                    # 子包：GNSS驱动
│   └── sensor_sync/                    # 子包：传感器时间同步
├── perception/                         # 功能域：环境感知
│   ├── pointcloud_preprocessing/       # 子包：点云预处理
│   ├── lidar_obstacle_detection/       # 子包：激光障碍物检测
│   └── camera_obstacle_detection/      # 子包：视觉障碍物检测
├── prediction/                         # 功能域：轨迹预测
│   ├── obstacle_predictor/             # 子包：轨迹预测
│   └── map_based_predictor/            # 子包：地图约束预测
├── localization/                       # 功能域：定位
│   ├── slam_odometry/                  # 子包：SLAM里程计
│   ├── dr_odometry/                    # 子包：航位推算
│   ├── pose_fusion/                    # 子包：多源位姿融合
│   ├── global_matcher/                 # 子包：Fast ICP全局匹配
│   ├── localization_manager/           # 子包：定位管理器
│   └── relocalization/                 # 子包：重定位
├── planning/                           # 功能域：路径规划
│   ├── global_planner/                 # 子包：全局规划
│   ├── local_planner/                  # 子包：局部规划
│   └── path_smoother/                  # 子包：路径平滑
├── control/                            # 功能域：运动控制
│   ├── motion_controller/              # 子包：运动控制器
│   ├── steering_control/               # 子包：转向控制
│   └── speed_control/                  # 子包：速度控制
├── vehicle_interface/                  # 功能域：车辆接口
│   ├── vehicle_interface_node/         # 子包：CAN协议转换
│   └── vehicle_state_receiver/         # 子包：状态接收
├── system/                             # 功能域：系统服务
│   ├── system_monitor/                 # 子包：系统监控
│   ├── data_logger/                    # 子包：数据记录
│   ├── communication_gateway/          # 子包：通信网关
│   └── parameter_server/               # 子包：参数服务
└── simulation/                         # 功能域：仿真
    ├── mine_world/                     # 子包：井下环境
    ├── vehicle_model/                  # 子包：车辆模型
    └── sensor_simulator/               # 子包：传感器仿真
```

### 1.2 为什么采用子包模式？

| 对比维度 | 单大包模式（旧） | 子包模式（新） |
|---------|---------------|--------------|
| 编译粒度 | 改一行代码要编译整个模块 | 只编译改动的子包 |
| 依赖管理 | 模块内所有节点共享同一套依赖 | 每个子包只引入自己需要的依赖 |
| 复用性 | 难以单独复用某个功能 | 子包可独立移植到其他项目 |
| 团队协作 | 多人改同一模块易冲突 | 每人负责独立子包，冲突少 |
| CI/CD | 全模块测试耗时 | 只测试变更子包及其依赖 |

### 1.3 子包命名规范

```
功能域/子包名/

规则：
1. 子包名使用小写+下划线
2. 子包名应体现核心功能，避免缩写
3. 同一功能域内的子包名前缀尽量一致（如 localization 域内都用 *_odometry, *_fusion, *_matcher）
4. 节点可执行文件名 = 子包名 + "_node"（如 lidar_driver_node）
```

---

## 2. 子包组织规范

### 2.1 标准目录结构

每个子包**必须**包含以下目录和文件：

```
子包名/
├── package.xml              # ROS 包描述（名称、版本、依赖）
├── CMakeLists.txt           # CMake 构建配置
├── src/                     # 源文件（.cpp）
│   └── 子包名_node.cpp      # 主节点入口
├── include/子包名/           # 头文件（.h / .hpp）
│   └── 子包名.hpp
├── launch/                  # 启动文件（.launch）
│   └── 子包名.launch
├── config/                  # 参数配置文件（.yaml）
│   └── 子包名_params.yaml
└── docs/                    # 子包专属文档（可选）
    ├── requirements.md
    └── design.md
```

### 2.2 package.xml 模板

```xml
<?xml version="1.0"?>
<package format="2">
  <name>子包名</name>
  <version>1.0.0</version>
  <description>一句话描述子包功能</description>
  <maintainer email="developer@example.com">Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>catkin</buildtool_depend>

  <!-- 核心 ROS 依赖 -->
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>

  <!-- 可选：传感器消息 -->
  <build_depend>sensor_msgs</build_depend>

  <!-- 可选：PCL / OpenCV / Eigen -->
  <build_depend>pcl_ros</build_depend>
  <build_depend>cv_bridge</build_depend>

  <!-- 必须：消息包 -->
  <build_depend>deeplumin_msgs</build_depend>

  <!-- 必须：统一依赖管理 -->
  <build_depend>deeplumin_cmake</build_depend>

  <!-- exec_depend 与 build_depend 保持一致 -->
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>pcl_ros</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>deeplumin_msgs</exec_depend>
  <exec_depend>deeplumin_cmake</exec_depend>

  <export>
    <build_type>catkin</build_type>
  </export>
</package>
```

### 2.3 CMakeLists.txt 模板

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(子包名)

# ===== 步骤1：引入统一依赖管理 =====
find_package(deeplumin_cmake REQUIRED)

# ===== 步骤2：查找 ROS 依赖 =====
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  deeplumin_msgs
)

# ===== 步骤3：按需查找第三方库（使用 deeplumin_cmake 的版本变量） =====
# 需要 PCL 时：
find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED)

# 需要 OpenCV 时：
find_package(OpenCV ${DEEPLUMIN_OPENCV_VERSION} EXACT REQUIRED)

# 需要 Eigen3 时：
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)

# ===== 步骤4：catkin 包配置 =====
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs deeplumin_msgs
)

# ===== 步骤5：头文件搜索路径 =====
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}           # 如果用了 PCL
  ${OpenCV_INCLUDE_DIRS}        # 如果用了 OpenCV
  ${EIGEN3_INCLUDE_DIR}         # 如果用了 Eigen3
)

# ===== 步骤6：编译节点 =====
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}              # 如果用了 PCL
  ${OpenCV_LIBRARIES}           # 如果用了 OpenCV
)

# ===== 步骤7：安装规则 =====
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

---

## 3. 统一依赖管理（deeplumin_cmake）

### 3.1 设计目标

- **版本一致性**：所有子包引用同一套第三方库版本，避免版本冲突
- **按需引入**：子包只引入自己需要的库，不强制安装所有依赖
- **GPU 可选**：CUDA 和 TensorRT 为可选依赖，无 GPU 环境时自动跳过相关模块

### 3.2 版本号定义

所有版本号集中定义在 [deeplumin_cmake/cmake/deeplumin_dependencies.cmake](file:///home/ubuntu/G1_workspace/DeepLumin/src/deeplumin_cmake/cmake/deeplumin_dependencies.cmake)：

| 库 | 版本变量 | 当前值 | 说明 |
|----|---------|--------|------|
| PCL | `DEEPLUMIN_PCL_VERSION` | 1.10 | 点云处理库 |
| OpenCV | `DEEPLUMIN_OPENCV_VERSION` | 4.2 | 计算机视觉库 |
| Eigen3 | `DEEPLUMIN_EIGEN_VERSION` | 3.3 | 矩阵运算库 |
| CUDA | `DEEPLUMIN_CUDA_VERSION` | 11.4 | GPU 并行计算（可选） |
| TensorRT | `DEEPLUMIN_TENSORRT_VERSION` | 8.2 | 推理加速引擎（可选） |

### 3.3 子包使用方式

**Step 1**：在 `CMakeLists.txt` 顶部引入

```cmake
find_package(deeplumin_cmake REQUIRED)
```

**Step 2**：使用版本变量进行 `find_package`

```cmake
# 需要 PCL 的子包
find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED)

# 需要 OpenCV 的子包
find_package(OpenCV ${DEEPLUMIN_OPENCV_VERSION} EXACT REQUIRED)

# 需要 Eigen3 的子包
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)
```

**Step 3**：在 `package.xml` 中声明依赖

```xml
<build_depend>deeplumin_cmake</build_depend>
<exec_depend>deeplumin_cmake</exec_depend>
```

### 3.4 修改全局版本号

当需要升级某个库的版本时，**只需修改一处**：

```cmake
# src/deeplumin_cmake/cmake/deeplumin_dependencies.cmake
set(DEEPLUMIN_PCL_VERSION "1.12" CACHE STRING "PCL version")  # 从 1.10 升级到 1.12
```

然后重新编译整个工作空间：

```bash
cd /home/ubuntu/G1_workspace/DeepLumin
catkin_make clean
catkin_make -DCMAKE_BUILD_TYPE=Release
```

所有子包会自动使用新版本的 PCL。

---

## 4. CUDA / TensorRT 集成指南

### 4.1 设计原则

- **可选依赖**：CUDA 和 TensorRT 不是强制依赖。没有 GPU 的机器上，项目仍能正常编译和运行（CPU 模式）。
- **条件编译**：使用 CMake 的 `if(DEEPLUMIN_CUDA_FOUND)` 条件判断，自动跳过 GPU 代码。
- **隔离封装**：GPU 加速代码封装在独立的类/函数中，主逻辑不直接调用 CUDA API。

### 4.2 在子包中集成 CUDA

**Step 1：检查 CUDA 可用性**

```cmake
# CMakeLists.txt
deeplumin_check_cuda()  # 如果 CUDA 不可用，打印警告并继续

if(DEEPLUMIN_CUDA_FOUND)
  # 定义一个宏，让 C++ 代码知道 CUDA 可用
  add_definitions(-DUSE_CUDA)
  
  # 添加 CUDA 源文件
  cuda_add_executable(my_node src/my_node.cpp src/cuda_kernel.cu)
else()
  # 无 CUDA 时只编译 CPU 代码
  add_executable(my_node src/my_node.cpp)
endif()
```

**Step 2：C++ 代码中条件编译**

```cpp
// include/my_package/cuda_wrapper.hpp
#pragma once

#ifdef USE_CUDA
  #include <cuda_runtime.h>
  #define CUDA_CHECK(call) \
      do { cudaError_t err = call; if(err != cudaSuccess) \
          ROS_ERROR("CUDA error: %s", cudaGetErrorString(err)); } while(0)
#else
  // 无 CUDA 时的空实现
  #define CUDA_CHECK(call) ((void)0)
#endif

class CudaAccelerator
{
public:
    void process(const float* input, float* output, int size);
    
private:
#ifdef USE_CUDA
    float* d_input_ = nullptr;
    float* d_output_ = nullptr;
#endif
};
```

```cpp
// src/cuda_wrapper.cpp
#include "my_package/cuda_wrapper.hpp"

void CudaAccelerator::process(const float* input, float* output, int size)
{
#ifdef USE_CUDA
    // GPU 实现
    CUDA_CHECK(cudaMemcpy(d_input_, input, size * sizeof(float), cudaMemcpyHostToDevice));
    // ... 启动 kernel ...
    CUDA_CHECK(cudaMemcpy(output, d_output_, size * sizeof(float), cudaMemcpyDeviceToHost));
#else
    // CPU 兜底实现
    for (int i = 0; i < size; ++i) {
        output[i] = input[i] * 2.0f;  // 示例操作
    }
    ROS_WARN_ONCE("Running CPU fallback, consider enabling CUDA for better performance");
#endif
}
```

### 4.3 在子包中集成 TensorRT

**TensorRT 用于深度学习推理加速**（如 YOLO 目标检测、点云分割）。

```cmake
# CMakeLists.txt
deeplumin_check_tensorrt()

if(DEEPLUMIN_TENSORRT_FOUND AND DEEPLUMIN_CUDA_FOUND)
  add_definitions(-DUSE_TENSORRT)
  
  include_directories(${DEEPLUMIN_CUDA_INCLUDE_DIRS})
  
  add_executable(detector_node src/detector_node.cpp src/tensorrt_engine.cpp)
  target_link_libraries(detector_node
    ${catkin_LIBRARIES}
    ${DEEPLUMIN_CUDA_LIBRARIES}
    ${DEEPLUMIN_TENSORRT_LIBRARY}
    nvinfer_plugin
    nvonnxparser
  )
else()
  add_executable(detector_node src/detector_node.cpp src/cpu_detector.cpp)
  target_link_libraries(detector_node ${catkin_LIBRARIES})
endif()
```

### 4.4 推荐的 GPU 加速子包

| 子包 | 加速内容 | 预期收益 |
|------|---------|---------|
| `lidar_obstacle_detection` | 点云聚类 CUDA 加速 | 10~20x |
| `camera_obstacle_detection` | YOLO/RT-DETR TensorRT 推理 | 5~10x |
| `global_matcher` | ICP 并行最近邻搜索 | 3~5x |
| `local_planner` | 轨迹采样批量计算 | 2~3x |

---

## 5. 消息定义规范

### 5.1 消息组织原则

所有自定义消息**必须**放在 `deeplumin_msgs` 包中，按功能域分目录：

```
deeplumin_msgs/msg/
├── perception/       # 感知相关消息
├── localization/     # 定位相关消息
├── planning/         # 规划相关消息
├── control/          # 控制相关消息
└── system/           # 系统状态消息
```

**禁止**在业务子包中定义 `.msg` 文件。

### 5.2 消息命名规范

```
功能域/消息名.msg

规则：
1. 消息名使用 PascalCase（大驼峰）
2. 名词优先，不使用动词
3. 数组类消息用复数形式或 Array 后缀
   正确：Obstacle.msg, ObstacleArray.msg
   错误：obstacle.msg, GetObstacle.msg
4. 状态类消息用 Status 后缀
   正确：LocalizationStatus.msg, SystemStatus.msg
```

### 5.3 消息字段规范

每条消息**必须**包含 `std_msgs/Header`：

```
std_msgs/Header header
  uint32 seq      # 序列号（ROS 自动填充）
  time stamp      # 时间戳（用于时间同步）
  string frame_id # 坐标系 ID（如 "base_link", "lidar_link"）
```

**字段类型优先级**：

| 数据类型 | 使用场景 | 示例 |
|---------|---------|------|
| `geometry_msgs/Point` | 3D 位置 | 障碍物位置 |
| `geometry_msgs/Quaternion` | 姿态 | 车辆朝向 |
| `geometry_msgs/Pose` | 位姿（位置+姿态） | 定位结果 |
| `geometry_msgs/Twist` | 速度/角速度 | 车辆速度 |
| `float32` / `float64` | 标量 | 置信度、速度值 |
| `uint8` + 常量 | 枚举状态 | 控制模式、失效等级 |
| `string` | 描述性信息 | 失效原因 |

**枚举定义示例**：

```
uint8 mode
uint8 MODE_MANUAL = 0
uint8 MODE_AUTO = 1
uint8 MODE_EMERGENCY_STOP = 2
```

### 5.4 新增消息流程

1. 在 `deeplumin_msgs/msg/功能域/` 下创建 `.msg` 文件
2. 更新 `deeplumin_msgs/CMakeLists.txt` 中的 `add_message_files`
3. 重新编译 `deeplumin_msgs` 包：`catkin_make --pkg deeplumin_msgs`
4. 在业务子包中 `#include <deeplumin_msgs/消息名.h>`

---

## 6. 添加新子包完整流程

### 6.1 场景：新增 "巷道宽度检测" 功能

**Step 1：确定功能域**

巷道宽度检测属于**感知**功能域，应放在 `perception/` 下。

**Step 2：创建子包目录**

```bash
cd /home/ubuntu/G1_workspace/DeepLumin/src
mkdir -p perception/tunnel_width_detector/{src,include/tunnel_width_detector,launch,config}
```

**Step 3：编写 package.xml**

```xml
<?xml version="1.0"?>
<package format="2">
  <name>tunnel_width_detector</name>
  <version>1.0.0</version>
  <description>巷道宽度检测 - 基于点云分析检测井下巷道有效宽度</description>
  <maintainer email="developer@example.com">Developer</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>pcl_ros</build_depend>
  <build_depend>deeplumin_msgs</build_depend>
  <build_depend>deeplumin_cmake</build_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>pcl_ros</exec_depend>
  <exec_depend>deeplumin_msgs</exec_depend>
  <exec_depend>deeplumin_cmake</exec_depend>

  <export>
    <build_type>catkin</build_type>
  </export>
</package>
```

**Step 4：编写 CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(tunnel_width_detector)

find_package(deeplumin_cmake REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
  deeplumin_msgs
)

find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros deeplumin_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

**Step 5：编写节点代码**

```cpp
// src/tunnel_width_detector_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class TunnelWidthDetector
{
public:
    TunnelWidthDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    {
        pnh.param<double>("detection_range", detection_range_, 30.0);
        pnh.param<double>("min_width", min_width_, 2.5);
        
        sub_points_ = nh.subscribe("/points_no_ground", 10,
                                   &TunnelWidthDetector::pointsCallback, this);
        pub_width_ = nh.advertise<std_msgs::Float32>("/tunnel_width", 10);
    }

private:
    void pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // 计算巷道宽度（简化示例：找左右墙壁的最大距离）
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        
        for (const auto& pt : cloud->points) {
            if (std::abs(pt.x) < detection_range_) {
                min_y = std::min(min_y, (double)pt.y);
                max_y = std::max(max_y, (double)pt.y);
            }
        }
        
        double width = max_y - min_y;
        
        std_msgs::Float32 result;
        result.data = width;
        pub_width_.publish(result);
        
        if (width < min_width_) {
            ROS_WARN("巷道宽度过窄: %.2f m (最小要求 %.2f m)", width, min_width_);
        }
    }

    ros::Subscriber sub_points_;
    ros::Publisher pub_width_;
    double detection_range_;
    double min_width_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tunnel_width_detector");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    TunnelWidthDetector detector(nh, pnh);
    ros::spin();
    
    return 0;
}
```

**Step 6：添加参数配置文件**

```yaml
# config/tunnel_width_detector_params.yaml
tunnel_width_detector:
  detection_range: 30.0    # 检测范围（m）
  min_width: 2.5           # 最小允许宽度（m）
  publish_rate: 10.0       # 发布频率（Hz）
```

**Step 7：添加启动文件**

```xml
<!-- launch/tunnel_width_detector.launch -->
<launch>
  <node pkg="tunnel_width_detector" type="tunnel_width_detector_node"
        name="tunnel_width_detector" output="screen">
    <rosparam file="$(find tunnel_width_detector)/config/tunnel_width_detector_params.yaml"
              command="load" ns="tunnel_width_detector" />
  </node>
</launch>
```

**Step 8：编译测试**

```bash
cd /home/ubuntu/G1_workspace/DeepLumin
catkin_make --pkg tunnel_width_detector
source devel/setup.bash
roslaunch tunnel_width_detector tunnel_width_detector.launch
```

---

## 7. 编译与 CI

### 7.1 本地编译流程

```bash
# 1. 加载环境
source /opt/ros/noetic/setup.bash
cd /home/ubuntu/G1_workspace/DeepLumin

# 2. 编译整个工作空间（首次）
catkin_make -DCMAKE_BUILD_TYPE=Release

# 3. 只编译变更子包（日常开发）
catkin_make --pkg 子包名

# 4. 强制重新编译（CMakeLists.txt 修改后）
catkin_make --force-cmake
```

### 7.2 推荐 CI 流程

```yaml
# .github/workflows/ci.yml（示例）
name: CI

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-20.04
    steps:
      - uses: actions/checkout@v2
      
      - name: Setup ROS
        uses: ros-tooling/setup-ros@v0.3
        with:
          required-ros-distributions: noetic
      
      - name: Install dependencies
        run: |
          sudo apt-get update
          sudo apt-get install -y libpcl-dev libopencv-dev libeigen3-dev
          rosdep install --from-paths src --ignore-src -y
      
      - name: Build
        run: |
          source /opt/ros/noetic/setup.bash
          catkin_make -DCMAKE_BUILD_TYPE=Release
      
      - name: Test
        run: |
          source devel/setup.bash
          catkin_make run_tests
          catkin_test_results
```

---

## 8. 代码审查 checklist

提交 PR 前，请确认以下事项：

### 8.1 结构规范

- [ ] 子包目录结构符合标准（src/ include/ launch/ config/）
- [ ] package.xml 包含正确的 name、version、description
- [ ] package.xml 声明了 deeplumin_cmake 和 deeplumin_msgs 依赖
- [ ] CMakeLists.txt 顶部有 `find_package(deeplumin_cmake REQUIRED)`
- [ ] CMakeLists.txt 使用版本变量（`${DEEPLUMIN_PCL_VERSION}` 等）
- [ ] 有 install 规则（install TARGETS / DIRECTORY）

### 8.2 代码规范

- [ ] 类名使用 PascalCase，函数/变量使用 snake_case
- [ ] 成员变量以 `_` 结尾
- [ ] 所有参数从 YAML/参数服务器加载，硬编码值有注释说明
- [ ] 使用 `ros::NodeHandle pnh("~")` 读取私有参数
- [ ] 日志级别使用恰当（DEBUG/INFO/WARN/ERROR）
- [ ] 回调函数中不阻塞（不 sleep、不做重计算）

### 8.3 消息与接口

- [ ] 新消息定义在 deeplumin_msgs 中，不在业务子包中
- [ ] 消息包含 `std_msgs/Header header`
- [ ] 话题名使用小写+下划线，以 `/` 开头
- [ ] 发布频率符合设计文档要求

### 8.4 可扩展性

- [ ] 算法核心逻辑封装为类，不直接写在 main 中
- [ ] 关键参数外部化到 YAML 配置文件
- [ ] 有合理的默认值，缺失参数时不崩溃
- [ ] 考虑 GPU 加速时，提供 CPU fallback 实现
