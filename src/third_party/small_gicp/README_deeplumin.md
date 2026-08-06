# small_gicp（DeepLumin catkin 包装）

上游仓库：<https://github.com/koide3/small_gicp>

本目录是 DeepLumin 工作区使用的 vendor 版本，基于上游 master 拉取：

- 版本：1.0.1
- commit：`8a2d3734f699c042db74ae61d295b0b928163526`（shallow clone，depth=1）

## 相对上游的改动

1. `package.xml`：`<export><build_type>` 由 `cmake` 改为 `catkin`，`<buildtool_depend>` 改为 `catkin`。
2. `CMakeLists.txt`：
   - 增加 ROS1/catkin 检测（`find_package(catkin QUIET)`）；
   - catkin 环境下调用 `catkin_package(INCLUDE_DIRS include LIBRARIES small_gicp)`；
   - catkin 环境下 install 走 catkin 路径（库到 `CATKIN_PACKAGE_LIB_DESTINATION`，头文件到 `CATKIN_GLOBAL_INCLUDE_DESTINATION`）；
   - 非 ROS 环境保持上游原始 cmake 安装行为。
3. 其余源码、头文件与上游一致。

## 消费方式（ROS 1 / catkin_make）

依赖包中：

```cmake
find_package(catkin REQUIRED COMPONENTS ... small_gicp)
target_link_libraries(your_target ${catkin_LIBRARIES} ...)
```

```xml
<depend>small_gicp</depend>
```

```cpp
#include <small_gicp/registration/registration_helper.hpp>  // preprocess_points / align 便捷函数
#include <small_gicp/registration/registration.hpp>          // Registration 模板
#include <small_gicp/ros/ros1.hpp>                            // PointCloud2 <-> small_gicp::PointCloud
```

## 注意

- 本工作区 PCL 固定为 1.10（Noetic）。small_gicp 的 PCL 接口
  （`small_gicp/pcl/pcl_registration.hpp` 与 `pcl::PointCovariance` traits）要求 PCL >= 1.11，
  集成时建议使用原生接口（`std::vector<Eigen::Vector4f>` 或 `small_gicp::PointCloud`）。
- helper 库 `libsmall_gicp.so` 提供 `preprocess_points` / `align` / `create_gaussian_voxelmap` 等便捷函数；
  核心算法本身是 header-only。
