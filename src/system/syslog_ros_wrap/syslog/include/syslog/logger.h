#ifndef SYSLOG_LOGGER_H_
#define SYSLOG_LOGGER_H_

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

#include "syslog/log_severity.h"

namespace syslog {

class LoggerManager;

class LogMessage {
 public:
  LogMessage(const char* file, int line, const char* function,
             const std::string& module, LogSeverity severity, bool print_to_terminal = false);
  ~LogMessage();

  LogMessage(const LogMessage&) = delete;
  LogMessage& operator=(const LogMessage&) = delete;

  std::ostream& stream() { return stream_; }

  static std::ostream& NullStream();

  void SetPrintToTerminal(bool enabled) { print_to_terminal_ = enabled; }
  bool PrintToTerminal() const { return print_to_terminal_; }

 private:
  std::ostringstream stream_;
  std::string module_;
  LogSeverity severity_;
  bool print_to_terminal_{false};
};

class LoggerManager {
 public:
  static LoggerManager& Instance();

  void Init(const std::string& log_dir,
            LogSeverity min_severity = LogSeverity::INFO,
            size_t max_file_size_mb = 50,
            const std::string& default_module = "main");

  void Shutdown();

  bool ShouldLog(LogSeverity severity) const;

  LogSeverity MinSeverity() const { return min_severity_; }
  const std::string& DefaultModule() const { return default_module_; }
  const std::string& LogDir() const { return log_dir_; }
  size_t MaxFileSizeMb() const { return max_file_size_mb_; }

  void SetMinSeverity(LogSeverity severity) { min_severity_ = severity; }
  void SetDefaultModule(const std::string& module) { default_module_ = module; }
  void SetLogDir(const std::string& dir) { log_dir_ = dir; }

 private:
  friend class LogMessage;

  LoggerManager() = default;
  ~LoggerManager();

  LoggerManager(const LoggerManager&) = delete;
  LoggerManager& operator=(const LoggerManager&) = delete;

  void WriteLog(const std::string& module, LogSeverity severity,
                const std::string& message);

  std::string log_dir_;
  LogSeverity min_severity_{LogSeverity::INFO};
  size_t max_file_size_mb_{50};
  std::string default_module_{"main"};
  bool initialized_{false};

  std::mutex mutex_;
  std::atomic<bool> shutting_down_{false};
  std::unordered_map<std::string, std::unique_ptr<std::mutex>> file_mutexes_;
  std::unordered_map<std::string, std::ofstream> log_files_;
  std::unordered_map<std::string, size_t> file_sizes_;
};

#define SYS_LOG(severity, print_to_terminal)                                                      \
  ((static_cast<int>(severity) >=                                             \
    static_cast<int>(::syslog::LoggerManager::Instance().MinSeverity()))     \
       ? ::syslog::LogMessage(                                               \
             __FILE__, __LINE__, __func__,                                   \
             ::syslog::LoggerManager::Instance().DefaultModule(),            \
             severity, print_to_terminal)                                    \
             .stream()                                                       \
       : ::syslog::LogMessage::NullStream())

#define SYS_LOG_TO(module, severity, print_to_terminal)                                         \
  ((static_cast<int>(severity) >=                                             \
    static_cast<int>(::syslog::LoggerManager::Instance().MinSeverity()))     \
       ? ::syslog::LogMessage(                                               \
             __FILE__, __LINE__, __func__,                                   \
             module,                                                         \
             severity, print_to_terminal)                                    \
             .stream()                                                       \
       : ::syslog::LogMessage::NullStream())

#define SYS_LOG_SET_MODULE(module) \
  ::syslog::LoggerManager::Instance().SetDefaultModule(module)

#define SYS_LOG_INIT(log_dir) \
  ::syslog::LoggerManager::Instance().Init(log_dir)

#define SYS_LOG_INIT_FULL(log_dir, severity, max_mb, module) \
  ::syslog::LoggerManager::Instance().Init(log_dir, severity, max_mb, module)

}  // namespace syslog

#endif  // SYSLOG_LOGGER_H_