# DeepLumin 统一依赖版本管理
# 所有业务包应在 CMakeLists.txt 顶部引入：
#   find_package(deeplumin_cmake REQUIRED)
# 然后使用下方变量进行 find_package：
#   find_package(PCL ${DEEPLUMIN_PCL_VERSION} EXACT REQUIRED)

# ==================== 核心库版本 ====================
set(DEEPLUMIN_PCL_VERSION "1.10" CACHE STRING "PCL version")
set(DEEPLUMIN_OPENCV_VERSION "4.2" CACHE STRING "OpenCV version")
set(DEEPLUMIN_EIGEN_VERSION "3.3.7" CACHE STRING "Eigen3 version")

# ==================== CUDA / TensorRT 版本 ====================
# 仅当系统安装了 CUDA/TensorRT 时才启用
# 各子包按需引用，不强制要求所有包都安装
set(DEEPLUMIN_CUDA_VERSION "12.1" CACHE STRING "CUDA version")
set(DEEPLUMIN_TENSORRT_VERSION "8.2" CACHE STRING "TensorRT version")

# CUDA 可选查找（QUIET 模式，找不到不报错）
find_package(CUDA ${DEEPLUMIN_CUDA_VERSION} QUIET)
if(CUDA_FOUND)
  message(STATUS "[deeplumin_cmake] CUDA ${CUDA_VERSION} found")
  set(DEEPLUMIN_CUDA_FOUND TRUE CACHE BOOL "" FORCE)
  set(DEEPLUMIN_CUDA_INCLUDE_DIRS ${CUDA_INCLUDE_DIRS} CACHE PATH "" FORCE)
  set(DEEPLUMIN_CUDA_LIBRARIES ${CUDA_LIBRARIES} CACHE STRING "" FORCE)
else()
  message(STATUS "[deeplumin_cmake] CUDA not found, GPU-accelerated packages will skip")
  set(DEEPLUMIN_CUDA_FOUND FALSE CACHE BOOL "" FORCE)
endif()

# TensorRT 可选查找
find_library(TENSORRT_LIBRARY nvinfer
  HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64
)
if(TENSORRT_LIBRARY)
  message(STATUS "[deeplumin_cmake] TensorRT found: ${TENSORRT_LIBRARY}")
  set(DEEPLUMIN_TENSORRT_FOUND TRUE CACHE BOOL "" FORCE)
  set(DEEPLUMIN_TENSORRT_LIBRARY ${TENSORRT_LIBRARY} CACHE PATH "" FORCE)
else()
  message(STATUS "[deeplumin_cmake] TensorRT not found, inference packages will skip")
  set(DEEPLUMIN_TENSORRT_FOUND FALSE CACHE BOOL "" FORCE)
endif()

# ==================== 便捷宏 ====================
# 检查 CUDA 是否可用，如果不可用则给出警告并跳过当前包中的 GPU 代码
macro(deeplumin_check_cuda)
  if(NOT DEEPLUMIN_CUDA_FOUND)
    message(WARNING "[${PROJECT_NAME}] CUDA not available, skipping GPU-accelerated targets")
  endif()
endmacro()

# 检查 TensorRT 是否可用
macro(deeplumin_check_tensorrt)
  if(NOT DEEPLUMIN_TENSORRT_FOUND)
    message(WARNING "[${PROJECT_NAME}] TensorRT not available, skipping inference targets")
  endif()
endmacro()
