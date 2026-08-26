#include "mapping_manager/ros/bag_recorder.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace {

double steadyNowSec() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

std::string findExecutable(const std::string& name) {
  if (name.find('/') != std::string::npos) {
    return access(name.c_str(), X_OK) == 0 ? name : std::string();
  }
  const char* path_env = getenv("PATH");
  if (path_env == nullptr) return std::string();
  const std::string path(path_env);
  std::size_t start = 0;
  while (start <= path.size()) {
    const std::size_t end = path.find(':', start);
    const std::string dir =
        path.substr(start, end == std::string::npos ? std::string::npos : end - start);
    const std::string candidate = (dir.empty() ? std::string(".") : dir) + "/" + name;
    if (access(candidate.c_str(), X_OK) == 0) return candidate;
    if (end == std::string::npos) break;
    start = end + 1;
  }
  return std::string();
}

}  // namespace

namespace mapping_manager {

BagRecorder::~BagRecorder() { shutdown(); }

bool BagRecorder::start(const std::string& bag_path, const std::vector<std::string>& topics) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pid_ > 0) return false;
  if (bag_path.empty() || topics.empty()) return false;

  const std::string rosbag_bin = findExecutable("rosbag");
  if (rosbag_bin.empty()) {
    std::fprintf(stderr, "[bag_recorder] cannot find 'rosbag' executable in PATH\n");
    return false;
  }

  // Build argv in the parent before fork() so the child only calls async-signal-safe
  // functions between fork and exec.
  std::vector<std::string> args;
  args.reserve(4 + topics.size());
  args.push_back(rosbag_bin);
  args.push_back("record");
  args.push_back("-O");
  args.push_back(bag_path);
  for (const std::string& topic : topics) args.push_back(topic);

  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (std::string& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
  argv.push_back(nullptr);

  const pid_t pid = fork();
  if (pid < 0) {
    std::fprintf(stderr, "[bag_recorder] fork failed: %s\n", std::strerror(errno));
    return false;
  }
  if (pid == 0) {
    execvp(argv[0], argv.data());
    _exit(127);
  }

  pid_ = pid;
  bag_path_ = bag_path;
  stopping_ = false;
  std::fprintf(stderr, "[bag_recorder] started pid=%d bag=%s topics=%zu\n",
               static_cast<int>(pid), bag_path.c_str(), topics.size());
  return true;
}

bool BagRecorder::poll() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pid_ <= 0) return false;
  int status = 0;
  const pid_t r = waitpid(pid_, &status, WNOHANG);
  if (r == pid_) {
    std::fprintf(stderr, "[bag_recorder] process exited pid=%d bag=%s\n",
                 static_cast<int>(pid_), bag_path_.c_str());
    pid_ = -1;
    return false;
  }
  if (r < 0) {
    std::fprintf(stderr, "[bag_recorder] waitpid failed: %s\n", std::strerror(errno));
    pid_ = -1;
    return false;
  }
  return true;
}

bool BagRecorder::stop(double timeout_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (pid_ <= 0) return true;
  stopping_ = true;

  if (kill(pid_, SIGINT) != 0 && errno != ESRCH) {
    std::fprintf(stderr, "[bag_recorder] kill(SIGINT) failed pid=%d: %s\n",
                 static_cast<int>(pid_), std::strerror(errno));
  } else {
    std::fprintf(stderr, "[bag_recorder] sent SIGINT pid=%d\n", static_cast<int>(pid_));
  }

  const double deadline = steadyNowSec() + timeout_sec;
  int status = 0;
  while (steadyNowSec() < deadline) {
    const pid_t r = waitpid(pid_, &status, WNOHANG);
    if (r == pid_) {
      pid_ = -1;
      stopping_ = false;
      return true;
    }
    if (r < 0) {
      pid_ = -1;
      stopping_ = false;
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  std::fprintf(stderr, "[bag_recorder] stop timeout pid=%d, sending SIGKILL\n",
               static_cast<int>(pid_));
  kill(pid_, SIGKILL);
  waitpid(pid_, &status, 0);
  pid_ = -1;
  stopping_ = false;
  return false;
}

void BagRecorder::shutdown() { stop(2.0); }

}  // namespace mapping_manager
