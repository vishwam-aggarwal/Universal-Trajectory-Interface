#pragma once

#include "TrajectoryLimits.h"

class ITrajectoryProfile {
public:
    virtual ~ITrajectoryProfile() = default;

    // targetDuration = 0  -> minimum time given limits
    // targetDuration > 0  -> time-dilate to exactly this duration (multi-axis sync)
    virtual bool plan(float q0, float qf, const TrajectoryLimits& limits,
                      float targetDuration = 0.0f) = 0;

    // Pure query, no side effects. Clamped for t < 0 or t > duration.
    // Returns true while still in motion, false once settled at qf.
    virtual bool evaluate(float t, float& pos, float& vel, float& accel) const = 0;

    virtual float getDuration() const = 0;
};
