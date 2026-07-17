#include "pose_fusion/common/types.hpp"

namespace pose_fusion {

std::string toString(ObservationSource source) {
  switch (source) {
    case ObservationSource::DR: return "DR";
    case ObservationSource::SLAM: return "SLAM";
    case ObservationSource::GLOBAL_MATCHER: return "GLOBAL_MATCHER";
    case ObservationSource::RELOCALIZATION: return "RELOCALIZATION";
    case ObservationSource::GNSS: return "GNSS";
    case ObservationSource::MANUAL: return "MANUAL";
    default: return "UNKNOWN";
  }
}

std::string toString(FusionStateName state) {
  switch (state) {
    case FusionStateName::UNINITIALIZED: return "UNINITIALIZED";
    case FusionStateName::DR_ONLY: return "DR_ONLY";
    case FusionStateName::LOCAL_FUSION: return "LOCAL_FUSION";
    case FusionStateName::MAP_CONSTRAINED: return "MAP_CONSTRAINED";
    case FusionStateName::DEGRADED: return "DEGRADED";
    case FusionStateName::FAILURE: return "FAILURE";
    default: return "UNKNOWN";
  }
}

}  // namespace pose_fusion
