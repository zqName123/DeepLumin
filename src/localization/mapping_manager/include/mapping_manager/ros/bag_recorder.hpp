#pragma once

#include <sys/types.h>

#include <mutex>
#include <string>
#include <vector>

namespace mapping_manager {

// Spawns and supervises a `rosbag record` child process.
class BagRecorder {
 public:
  BagRecorder() = default;
  ~BagRecorder();

  BagRecorder(const BagRecorder&) = delete;
  BagRecorder& operator=(const BagRecorder&) = delete;

  // Starts `rosbag record -O <bag_path> <topics...>`. Returns false if a child is
  // already running, arguments are empty, or the process cannot be spawned.
  bool start(const std::string& bag_path, const std::vector<std::string>& topics);

  // Sends SIGINT and waits up to timeout_sec for a graceful shutdown; escalates to
  // SIGKILL on timeout. Returns true when the child has exited (or was not running).
  bool stop(double timeout_sec = 5.0);

  // Reaps an already-exited child without sending signals. Returns true if running.
  bool poll();

  // Short-timeout stop used on node shutdown.
  void shutdown();

  bool running() const { return pid_ > 0; }
  bool stopping() const { return stopping_; }
  pid_t pid() const { return pid_; }

 private:
  pid_t pid_ = -1;
  bool stopping_ = false;
  std::string bag_path_;
  mutable std::mutex mutex_;
};

}  // namespace mapping_manager
