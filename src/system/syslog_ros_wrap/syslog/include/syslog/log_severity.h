#ifndef SYSLOG_LOG_SEVERITY_H_
#define SYSLOG_LOG_SEVERITY_H_

#include <string>

namespace syslog {

enum class LogSeverity : int {
  INFO = 0,
  WARNING = 1,
  ERROR = 2,
  FATAL = 3,
};

inline const char* SeverityName(LogSeverity s) {
  switch (s) {
    case LogSeverity::INFO:    return "INFO";
    case LogSeverity::WARNING: return "WARN";
    case LogSeverity::ERROR:   return "ERROR";
    case LogSeverity::FATAL:   return "FATAL";
  }
  return "UNKNOWN";
}

inline LogSeverity SeverityFromString(const std::string& s) {
  if (s == "INFO" || s == "info" || s == "0") return LogSeverity::INFO;
  if (s == "WARNING" || s == "WARN" || s == "warning" || s == "warn" || s == "1")
    return LogSeverity::WARNING;
  if (s == "ERROR" || s == "error" || s == "2") return LogSeverity::ERROR;
  if (s == "FATAL" || s == "fatal" || s == "3") return LogSeverity::FATAL;
  return LogSeverity::INFO;
}

}  // namespace syslog

#endif  // SYSLOG_LOG_SEVERITY_H_