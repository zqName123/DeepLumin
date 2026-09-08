# syslog 日志库

一个轻量级、线程安全、支持多模块分文件的 C++17 日志库。

## 特性

- **多模块分文件**：每个 module 写入独立的 `<module>.log`，便于按子系统检索
- **线程安全**：每个模块拥有独立互斥锁，多线程并发写入安全
- **日志级别过滤**：运行时可动态调整最低输出级别（INFO / WARNING / ERROR / FATAL）
- **自动滚动**：单文件超过配置上限时自动重命名归档并新建文件
- **高精度时间戳**：毫秒级，格式 `YYYY-MM-DD HH:MM:SS.mmm`
- **自动附加调用位置**：文件名、行号、函数名
- **FATAL 自动终止**：记录 FATAL 日志后调用 `std::abort()`
- **错误同步输出**：ERROR 级别日志同时写入文件与 `stderr`
- **零外部依赖**：仅依赖标准库与 pthread

## 目录结构

```
common/syslog/
├── CMakeLists.txt                 # 独立 CMake 工程定义
├── build.sh                       # 编译 & 安装脚本
├── cmake/
│   └── syslogConfig.cmake.in      # find_package(syslog) 配置模板
├── include/syslog/
│   ├── log_severity.h             # 日志级别枚举
│   └── logger.h                   # LoggerManager 与日志宏
└── src/
    └── logger.cpp                 # 实现
```

## 编译与安装

### 快速安装（默认装到 `temp/lib`）

```bash
cd common/syslog
./build.sh
```

安装产物布局：

```
<install_prefix>/
├── include/syslog/{logger.h, log_severity.h}
└── lib/
    ├── libsyslog.a
    └── cmake/syslog/              # 供 find_package(syslog) 使用
        ├── syslogConfig.cmake
        ├── syslogConfigVersion.cmake
        └── syslogTargets.cmake
```

### 指定安装位置

`build.sh` 支持三种方式指定安装前缀（优先级从高到低）：

```bash
# 1) 命令行参数
./build.sh /opt/syslog

# 2) 环境变量
SYSLOG_INSTALL_PREFIX=/opt/syslog ./build.sh

# 3) 不传任何参数 → 默认 <temp>/lib
./build.sh
```

脚本可重复执行，每次都会清理 `build/` 重新配置。

## API 速查

### 日志级别

```cpp
enum class LogSeverity : int {
  INFO = 0,     // 常规信息
  WARNING = 1,  // 警告
  ERROR = 2,    // 错误，同时输出到 stderr
  FATAL = 3,    // 致命，记录后调用 std::abort()
};
```

辅助函数：

| 函数 | 说明 |
| --- | --- |
| `SeverityName(LogSeverity)` | 返回 `"INFO"/"WARN"/"ERROR"/"FATAL"` |
| `SeverityFromString(const std::string&)` | 字符串转枚举，支持 `"INFO"/"info"/"0"` 等形式 |

### 日志宏

| 宏 | 说明 |
| --- | --- |
| `SYS_LOG_INIT(log_dir)` | 简化初始化，仅指定日志目录，其余用默认值 |
| `SYS_LOG_INIT_FULL(log_dir, severity, max_mb, module)` | 完整初始化 |
| `SYS_LOG(severity) << ...` | 向**默认模块**写入日志 |
| `SYS_LOG_TO(module, severity) << ...` | 向**指定模块**写入日志 |
| `SYS_LOG_SET_MODULE(module)` | 修改默认模块名 |

> 所有 `SYS_LOG*` 宏内部已做级别过滤：低于 `MinSeverity` 时写入到空流，无开销。

### LoggerManager 接口

```cpp
class LoggerManager {
public:
  static LoggerManager& Instance();          // 单例入口

  void Init(const std::string& log_dir,
            LogSeverity min_severity = LogSeverity::INFO,
            size_t max_file_size_mb = 50,
            const std::string& default_module = "main");
  void Shutdown();                           // 关闭所有文件流

  // 运行时配置
  void SetMinSeverity(LogSeverity s);
  void SetDefaultModule(const std::string& m);
  void SetLogDir(const std::string& dir);

  // 查询
  LogSeverity    MinSeverity() const;
  const std::string& DefaultModule() const;
  const std::string& LogDir() const;
  size_t         MaxFileSizeMb() const;
  bool           ShouldLog(LogSeverity s) const;
};
```

## 使用示例

### 最小示例

```cpp
#include "syslog/logger.h"

int main() {
  SYS_LOG_INIT_FULL("./logs",                    // 日志目录
                    syslog::LogSeverity::INFO,    // 最低级别
                    50,                           // 单文件上限 50MB
                    "main");                      // 默认模块名

  SYS_LOG(syslog::LogSeverity::INFO) << "服务启动";
  SYS_LOG(syslog::LogSeverity::WARNING) << "配置项缺失，使用默认值";

  // 不同模块 → 不同文件 (perception.log / planning.log)
  SYS_LOG_TO("perception", syslog::LogSeverity::INFO) << "检测到 3 个目标";
  SYS_LOG_TO("planning",   syslog::LogSeverity::ERROR) << "路径规划失败";

  // 运行时调整级别：低于 WARNING 的日志将被丢弃
  syslog::LoggerManager::Instance().SetMinSeverity(syslog::LogSeverity::WARNING);

  syslog::LoggerManager::Instance().Shutdown();
  return 0;
}
```

编译（假设已 `./build.sh` 安装到默认位置）：

```bash
g++ -std=c++17 main.cpp \
    -I<install_prefix>/include \
    -L<install_prefix>/lib -lsyslog -pthread
```

### 在 CMake 项目中集成

```cmake
# 方式一：通过安装前缀查找
set(SYSLOG_INSTALL_PREFIX "/path/to/install" CACHE PATH "")
list(APPEND CMAKE_PREFIX_PATH "${SYSLOG_INSTALL_PREFIX}")
find_package(syslog REQUIRED)

add_executable(my_app main.cpp)
target_link_libraries(my_app PRIVATE syslog::syslog)
```

```cmake
# 方式二：直接指定 cmake 配置目录
find_package(syslog REQUIRED CONFIG
             PATHS "/path/to/install/lib/cmake/syslog")
```

`syslog::syslog` 会自动传递头文件路径与 pthread 依赖，无需额外 `target_link_libraries(Threads)`。

## 日志文件格式

每条日志一行，格式：

```
[2026-08-11 13:48:46.256] [INFO] [TID:139654255249280] [main.cpp:39] [main] 服务启动
└────── 时间戳 ────────┘ └─级别─┘ └──── 线程ID ────┘ └─ 文件:行号 ─┘ └函数┘ └ 消息 ┘
```

- **时间戳**：本地时间，毫秒精度
- **线程ID**：`std::this_thread::get_id()` 的字符串形式
- **文件:行号**：取自 `__FILE__` / `__LINE__`，文件名仅保留 basename
- **函数**：取自 `__func__`

### 文件滚动

当某模块文件大小达到 `max_file_size_mb` 时，原文件被重命名为
`<module>.log.<时间戳>` 并新建 `<module>.log` 继续写入。

## 在 ROS1 节点中使用

本仓库 `temp/src/syslog_test/` 提供了完整的 ROS1 测试包示例。

### 1. 先安装 syslog 库

```bash
cd common/syslog && ./build.sh
```

### 2. 编译 ROS 测试节点

```bash
source /opt/ros/noetic/setup.bash
cd temp                 # catkin 工作空间根目录
catkin_make
source devel/setup.bash
```

测试包的 [CMakeLists.txt](../../src/syslog_test/CMakeLists.txt) 通过
`SYSLOG_INSTALL_PREFIX`（默认指向 `temp/lib`）定位已安装的 syslog，
若安装到其他位置用 `catkin_make -DSYSLOG_INSTALL_PREFIX=/your/path` 覆盖。

### 3. 运行测试节点

```bash
roscore &
# 综合测试：覆盖各级别、多模块、多线程、运行时级别切换
roslaunch syslog_test syslog_test.launch

# 多模块演示：perception + planning 并发写入各自日志文件
roslaunch syslog_test syslog_multi.launch
```

运行后日志默认写入 `/tmp/syslog_test/` 与 `/tmp/syslog_multi/`，
可通过 launch 文件的 `log_dir` 参数修改。

## 常见问题

**Q: 为什么 `SYS_LOG(INFO)` 没有输出？**
A: 检查 `MinSeverity()`，级别低于它会被过滤。可用
   `LoggerManager::Instance().SetMinSeverity(LogSeverity::INFO)` 调低。

**Q: FATAL 日志后程序为什么直接退出？**
A: 这是设计行为。FATAL 记录后调用 `std::abort()`，确保致命错误不被忽略。
   若不希望退出，使用 `ERROR` 级别。

**Q: 多个可执行文件能否共享同一个 LoggerManager？**
A: `LoggerManager` 是进程内单例，仅在单个进程内有效。多进程需各自 Init，
   日志目录可以相同（各进程写入各自的模块文件），或分开存放避免混淆。

**Q: 如何禁用日志？**
A: 将 `MinSeverity` 设为高于 `FATAL` 的整数值，例如：
   `SetMinSeverity(static_cast<LogSeverity>(4))`，所有日志将被过滤。
