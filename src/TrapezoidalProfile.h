#pragma once

#include "ITrajectoryProfile.h"

class TrapezoidalProfile : public ITrajectoryProfile {
public:
    bool plan(float q0, float qf, const TrajectoryLimits& limits,
              float targetDuration) override;

    bool evaluate(float t, float& pos, float& vel, float& accel) const override;

    float getDuration() const override;

private:
    float _q0    = 0.0f;
    float _qf    = 0.0f;
    float _dir   = 1.0f;   // +1 or -1
    float _vPeak = 0.0f;   // actual peak velocity (< vMax for triangular profile)
    float _aMax  = 0.0f;
    float _t1    = 0.0f;   // end of acceleration phase
    float _t2    = 0.0f;   // end of cruise phase
    float _t3    = 0.0f;   // end of deceleration phase (== total duration)
    bool  _valid = false;
};
