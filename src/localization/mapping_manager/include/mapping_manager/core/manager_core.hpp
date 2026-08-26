#pragma once

#include "mapping_manager/common/config.hpp"
#include "mapping_manager/common/types.hpp"

#include <mutex>
#include <string>
#include <vector>

namespace mapping_manager {

class ManagerCore {
 public:
  ManagerCore() = default;

  bool initialize(const ManagerConfig& config);

  EnvironmentStatusData checkEnvironment(bool include_runtime_checks,
                                         const std::string& config_path,
                                         double now_sec);
  StartMappingResponseData startMapping(const StartMappingRequestData& req, double now_sec);
  StopMappingResponseData stopMapping(const StopMappingRequestData& req, double now_sec);
  StartBagRecordingResponseData startBagRecording(const StartBagRecordingRequestData& req,
                                                  double now_sec);
  StopBagRecordingResponseData stopBagRecording(const StopBagRecordingRequestData& req,
                                                double now_sec);
  GetStatusResponseData getStatus() const;

  void update(double now_sec);
  void updateFromBackend(const MappingStatusData& backend, double now_sec);
  void markFailed(const std::string& reason, double now_sec);
  void failBagRecording(const std::string& reason, double now_sec);
  void setRecordBytes(unsigned long long bytes, double now_sec);

 private:
  EnvironmentItem makeEnvItem(const std::string& name,
                              bool ok,
                              const std::string& level,
                              const std::string& detail) const;
  void resetStatus(double now_sec);
  void resetRecordStatus(double now_sec);
  bool isTaskActive() const;
  std::string makeTaskId(const std::string& task_name, double now_sec) const;

  ManagerConfig config_;
  bool initialized_ = false;
  mutable std::mutex mutex_;

  MappingStatusData status_;
  RecordStatusData record_status_;
  EnvironmentStatusData environment_status_;

  double task_start_sec_ = 0.0;
  double record_start_sec_ = 0.0;
  double runtime_started_deadline_sec_ = -1.0;
  double save_started_deadline_sec_ = -1.0;
  double finish_deadline_sec_ = -1.0;
};

}  // namespace mapping_manager
