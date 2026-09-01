#pragma once

#include <memory>

namespace faster_lio {

struct ImuSample {
    double timestamp = 0.0;
    struct {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    } angular_velocity;
    struct {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    } linear_acceleration;
};

using ImuSamplePtr = std::shared_ptr<const ImuSample>;

}  // namespace faster_lio
