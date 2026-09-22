#include "syslog/logger.h"

#include <algorithm>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace syslog {

namespace {

std::string GetTimestamp() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) %
            1000;

  std::tm tm_info;
  localtime_r(&time_t_now, &tm_info);

  char buf[64];
  std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d.%03ld",
                tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
                static_cast<long>(ms.count()));
  return std::string(buf);
}

std::string GetThreadId() {
  std::ostringstream oss;
  oss << std::this_thread::get_id();
  return oss.str();
}

std::string ExtractBasename(const char* path) {
  const char* slash = std::strrchr(path, '/');
  if (slash) {
    return std::string(slash + 1);
  }
  return std::string(path);
}

void EnsureDirectory(const std::string& path) {
  mkdir(path.c_str(), 0755);
}

}  // namespace

LoggerManager& LoggerManager::Instance() {
  static LoggerManager instance;
  return instance;
}

void LoggerManager::Init(const std::string& log_dir, LogSeverity min_severity,
                         size_t max_file_size_mb,
                         const std::string& default_module) {
  shutting_down_.store(false);
  std::lock_guard<std::mutex> lock(mutex_);
  log_dir_ = log_dir;
  min_severity_ = min_severity;
  max_file_size_mb_ = max_file_size_mb;
  default_module_ = default_module;
  EnsureDirectory(log_dir_);
  initialized_ = true;
}

void LoggerManager::Shutdown() {
  shutting_down_.store(true);
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& pair : log_files_) {
    if (pair.second.is_open()) {
      pair.second.flush();
      pair.second.close();
    }
  }
  log_files_.clear();
  file_sizes_.clear();
  initialized_ = false;
}

LoggerManager::~LoggerManager() {
  Shutdown();
}

bool LoggerManager::ShouldLog(LogSeverity severity) const {
  return static_cast<int>(severity) >= static_cast<int>(min_severity_);
}

void LoggerManager::WriteLog(const std::string& module, LogSeverity severity,
                             const std::string& message) {
  std::mutex* file_mutex_ptr = nullptr;
  std::ofstream* file_ptr = nullptr;
  size_t* file_size_ptr = nullptr;
  std::string log_dir;
  size_t max_file_size;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (shutting_down_.load() || !initialized_ || log_dir_.empty()) {
      return;
    }

    log_dir = log_dir_;
    max_file_size = max_file_size_mb_ * 1024 * 1024;

    auto& file_mutex = file_mutexes_[module];
    if (!file_mutex) {
      file_mutex = std::make_unique<std::mutex>();
    }
    file_mutex_ptr = file_mutex.get();

    auto it = log_files_.find(module);
    if (it == log_files_.end() || !it->second.is_open()) {
      std::string path = log_dir_ + "/" + module + ".log";
      log_files_[module].open(path, std::ios::out | std::ios::app);
      file_sizes_[module] = 0;
    }
    file_ptr = &log_files_[module];
    file_size_ptr = &file_sizes_[module];
  }

  if (shutting_down_.load()) {
    return;
  }

  std::lock_guard<std::mutex> file_lock(*file_mutex_ptr);

  if (!file_ptr->is_open()) {
    std::cerr << "[SYSLOG CANNOT OPEN FILE] " << log_dir << "/" << module
              << ".log" << std::endl;
    return;
  }

  if (*file_size_ptr >= max_file_size) {
    file_ptr->flush();
    file_ptr->close();
    std::string old_path = log_dir + "/" + module + ".log";
    std::string rotated_path = old_path + "." + GetTimestamp();
    rename(old_path.c_str(), rotated_path.c_str());
    file_ptr->open(old_path, std::ios::out | std::ios::app);
    *file_size_ptr = 0;
  }

  std::string timestamp = GetTimestamp();
  std::string tid = GetThreadId();
  const char* sev_name = SeverityName(severity);

  std::string line = "[" + timestamp + "] [" + sev_name + "] [TID:" + tid +
                     "] " + message + "\n";

  *file_ptr << line;
  file_ptr->flush();
  *file_size_ptr += line.size();

  if (severity >= LogSeverity::FATAL) {
    std::cerr << "[FATAL] " << message << std::endl;
    std::abort();
  }
}

LogMessage::LogMessage(const char* file, int line, const char* function,
                       const std::string& module, LogSeverity severity, bool print_to_terminal)
    : module_(module), severity_(severity), print_to_terminal_(print_to_terminal) {
  std::string basename = ExtractBasename(file);
  stream_ << "[" << basename << ":" << line << "] [" << function << "] ";
}

LogMessage::~LogMessage() {
  std::string message = stream_.str();
  LoggerManager::Instance().WriteLog(module_, severity_, message);

  if (severity_ >= LogSeverity::ERROR) {
    std::cerr << stream_.str() << std::endl;
  }
  else if (print_to_terminal_) {
    std::cerr << stream_.str() << std::endl;
  }
}

namespace {
class NullStreamBuf : public std::streambuf {
 protected:
  int overflow(int c) override { return c; }
};
}  // namespace

std::ostream& LogMessage::NullStream() {
  static NullStreamBuf null_buf;
  static std::ostream null_stream(&null_buf);
  return null_stream;
}

}  // namespace syslog