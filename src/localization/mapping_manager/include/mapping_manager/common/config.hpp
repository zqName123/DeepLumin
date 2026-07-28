#pragma once

#include <string>

namespace mapping_manager {

struct TopicConfig {
  std::string status = "/mapping/status";
  std::string record_status = "/mapping/record_status";
  std::string environment_last_result = "/mapping/environment_last_result";
};

struct ServiceConfig {
  std::string check_environment = "/mapping/check_environment";
  std::string get_status = "/mapping/get_status";
  std::string start_mapping = "/mapping/start";
  std::string stop_mapping = "/mapping/stop";
  std::string start_bag_recording = "/mapping/start_bag_recording";
  std::string stop_bag_recording = "/mapping/stop_bag_recording";
};

struct RuntimeConfig {
  double status_hz = 5.0;
  double startup_delay_sec = 1.0;
  double save_delay_sec = 1.5;
  double auto_finish_delay_sec = 0.5;
};

struct DefaultConfig {
  std::string output_root = "/tmp/deeplumin_mapping";
  std::string bag_output_dir = "/tmp/deeplumin_mapping/bag";
};

struct BackendConfig {
  bool enabled = true;
  std::string start_service = "/slam_mapping/start";
  std::string stop_service = "/slam_mapping/stop";
  std::string get_status_service = "/slam_mapping/get_status";
  std::string status_topic = "/slam_mapping/status";
};

struct ManagerConfig {
  TopicConfig topics;
  ServiceConfig services;
  RuntimeConfig runtime;
  DefaultConfig defaults;
  BackendConfig backend;
};

}  // namespace mapping_manager
