#pragma once

#include "ITrajectoryProfile.h"

class TrapezoidalProfile : public ITrajectoryProfile {
public:
    // Without this, declaring the 4-arg plan() override below hides ALL
    // base-class overloads of the name "plan" for any call made through a
    // TrapezoidalProfile-typed expression (C++ member lookup stops at the
    // first class scope where the name appears at all, before overload
    // resolution runs) -- including ITrajectoryProfile's 3-arg
    // targetDuration-omitting convenience overload, even though nothing
    // about that overload actually conflicts with this override. Pulls it
    // back into scope so `TrapezoidalProfile p; p.plan(q0, qf, limits);`
    // (no explicit targetDuration) works directly, not just through a
    // base-class reference/pointer.
    using ITrajectoryProfile::plan;

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
