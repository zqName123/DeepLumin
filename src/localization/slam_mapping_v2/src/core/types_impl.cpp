#include "slam_mapping_v2/common/types.hpp"

namespace slam_mapping_v2 {

std::string toString(RuntimeState state) {
  switch (state) {
    case RuntimeState::IDLE: return "IDLE";
    case RuntimeState::READY: return "READY";
    case RuntimeState::RUNNING: return "RUNNING";
    case RuntimeState::STOPPING: return "STOPPING";
    case RuntimeState::SAVING: return "SAVING";
    case RuntimeState::FINISHED: return "FINISHED";
    case RuntimeState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

}  // namespace slam_mapping_v2
