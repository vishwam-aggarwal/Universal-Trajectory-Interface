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

    // Returns false (rather than throwing/asserting, per this library's
    // no-exceptions hot-path constraint) in two cases where the plan is
    // still left in a usable, safe state:
    //  - limits.vMax <= 0 or limits.aMax <= 0: the profile is left invalid
    //    and evaluate() reports the axis parked at q0, never moving.
    //  - targetDuration > 0 but shorter than this move's own minimum-time
    //    duration (i.e. a "dilation" request that would actually have to
    //    speed the move up): the profile falls back to the minimum-time
    //    plan instead of exceeding vMax or producing a nonsensical
    //    duration, and getDuration() reports that clamped duration, not
    //    the one requested.
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
