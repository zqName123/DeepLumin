# slam_mapping_v2 自包含说明

本包已把复用的 slam_mapping 内容整体移植到包内，**不再依赖 slam_mapping /
slam_odometry 的外部 include 与库**。二进制中不链接 `libslam_mapping_core.so` /
`libslam_odometry_core.so`。

## 移植清单

`include/slam_mapping/`（命名空间不变，便于与上游 diff/同步）

| 文件 | 来源 |
|------|------|
| common/config.hpp、common/types.hpp | slam_mapping |
| core/keyframe_manager.hpp / .cpp | slam_mapping |
| core/loop_closure_backend.hpp / .cpp、core/i_loop_closure_backend.hpp | slam_mapping |
| core/loop_closure_fastlio2_backend.hpp / .cpp | slam_mapping |
| core/gnss_optimizer.hpp / .cpp | slam_mapping |
| core/map_saver.hpp / .cpp | slam_mapping |
| ros/param_loader.hpp / .cpp、ros/visualization_publisher.hpp / .cpp | slam_mapping |
| ros/ros_converters.hpp / .cpp | slam_mapping |

`include/localization/`（slam_mapping 依赖的最小公共类型/转换，自包含）

| 文件 | 来源 |
|------|------|
| common/config.hpp、common/types.hpp | slam_odometry（原样） |
| ros/ros_converters.hpp + src/vendored/localization_ros_converters.cpp | 最小化移植（loadSlamConfig + 基础转换） |

`src/vendored/types_impl.cpp`：`slam_mapping::toString`（原在 mapping_runtime.cpp）。

未移植：`mapping_runtime.hpp/.cpp`、`ros_adapter.hpp/.cpp`（内部直连 ISlamOdometry
的旧运行时；v2 用订阅驱动替代）。

## 外部依赖（仅系统/ROS 库）

roscpp、deeplumin_msgs、PCL、GTSAM、tf2_ros、pcl_ros 等 —— 与 slam_mapping /
slam_odometry 包本身无关。
