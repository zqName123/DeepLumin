# deeplumin_cmake

DeepLumin 统一 CMake 依赖管理包。集中管理 PCL、OpenCV、Eigen3、CUDA、TensorRT 等第三方库的版本号，确保全工作空间版本一致。

---

## 设计目标

| 目标 | 说明 |
|------|------|
| **版本一致性** | 所有子包引用同一套库版本，避免链接冲突 |
| **集中管理** | 升级版本只需改一处，全工作空间自动生效 |
| **按需引入** | 子包只引入自己需要的库，不强制安装所有依赖 |
| **GPU 可选** | CUDA / TensorRT 为可选依赖，无 GPU 环境自动跳过 |

---

## 使用方式

### Step 1：在子包 CMakeLists.txt 中引入

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_package)

# 必须：先引入 deeplumin_cmake
find_package(deeplumin_cmake REQUIRED)

# 然后使用版本变量进行 find_package
find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED)
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} EXACT REQUIRED)
```

### Step 2：在 package.xml 中声明依赖

```xml
<build_depend>deeplumin_cmake</build_depend>
<exec_depend>deeplumin_cmake</exec_depend>
```

### Step 3：使用统一变量

```cmake
# 头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}          # PCL 已自动找到
  ${EIGEN3_INCLUDE_DIR}        # Eigen3 已自动找到
)

# 链接库
target_link_libraries(my_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
```

---

## 版本号定义

所有版本号定义在 [cmake/deeplumin_dependencies.cmake](cmake/deeplumin_dependencies.cmake) 中：

| 库 | CMake 变量 | 当前值 | 类型 |
|----|-----------|--------|------|
| PCL | `DEEPLUMIN_PCL_VERSION` | 1.10 | 强制 |
| OpenCV | `DEEPLUMIN_OPENCV_VERSION` | 4.2 | 强制 |
| Eigen3 | `DEEPLUMIN_EIGEN_VERSION` | 3.3 | 强制 |
| CUDA | `DEEPLUMIN_CUDA_VERSION` | 11.4 | 可选 |
| TensorRT | `DEEPLUMIN_TENSORRT_VERSION` | 8.2 | 可选 |

**修改版本号**：

```cmake
# cmake/deeplumin_dependencies.cmake
set(DEEPLUMIN_PCL_VERSION "1.12" CACHE STRING "PCL version")
```

修改后重新编译整个工作空间即可。

---

## CUDA / TensorRT 可选集成

### 检查 CUDA 是否可用

```cmake
# CMakeLists.txt
deeplumin_check_cuda()

if(DEEPLUMIN_CUDA_FOUND)
  cuda_add_executable(my_gpu_node src/my_node.cpp src/kernel.cu)
  target_link_libraries(my_gpu_node ${catkin_LIBRARIES} ${DEEPLUMIN_CUDA_LIBRARIES})
else()
  add_executable(my_gpu_node src/my_node.cpp)
  target_link_libraries(my_gpu_node ${catkin_LIBRARIES})
endif()
```

### C++ 代码中条件编译

```cpp
#ifdef USE_CUDA
  #include <cuda_runtime.h>
  // GPU 实现
#else
  // CPU fallback
#endif
```

---

## 常见问题

### Q1：找不到 Eigen3 精确版本

**现象**：
```
Could not find a configuration file for package "Eigen3" that exactly
matches requested version "3.3"
```

**原因**：系统中安装的 Eigen3 版本与 `DEEPLUMIN_EIGEN_VERSION` 不匹配。

**解决**：
```bash
# 查看系统实际安装的 Eigen3 版本
cat /usr/share/eigen3/cmake/Eigen3Config.cmake | grep VERSION

# 修改 deeplumin_dependencies.cmake 中的版本号与之匹配
```

或去掉 `EXACT`（不推荐，会丧失版本一致性保障）：
```cmake
find_package(Eigen3 ${DEEPLUMIN_EIGEN_VERSION} REQUIRED)
```

### Q2：子包中找不到 DEEPLUMIN_PCL_VERSION 变量

**原因**：子包 CMakeLists.txt 中 `find_package(deeplumin_cmake REQUIRED)` 写在了 `find_package(PCL ...)` 之后，或根本没写。

**解决**：确保 `find_package(deeplumin_cmake REQUIRED)` 在**所有**使用版本变量的 `find_package` 之前。

```cmake
# 正确顺序 ✅
find_package(deeplumin_cmake REQUIRED)
find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED)

# 错误顺序 ❌
find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED)  # 变量未定义！
find_package(deeplumin_cmake REQUIRED)
```

### Q3：catkin_make 时 deeplumin_cmake 没有先编译

**原因**：子包的 `package.xml` 中缺少 `deeplumin_cmake` 的 `build_depend`。

**解决**：确保所有子包的 `package.xml` 都包含：

```xml
<build_depend>deeplumin_cmake</build_depend>
<exec_depend>deeplumin_cmake</exec_depend>
```

---

## 目录结构

```
deeplumin_cmake/
├── cmake/
│   └── deeplumin_dependencies.cmake    # 核心：版本定义 + CUDA/TensorRT 查找
├── package.xml                          # ROS 包描述
├── CMakeLists.txt                       # CMake 配置（导出 extras）
└── README.md                            # 本文件
```

---

## 相关文档

- [docs/developer_guide.md](../../docs/developer_guide.md) - 项目开发指南
- [docs/newcomer_guide.md](../../docs/newcomer_guide.md) - 新手入门
