#pragma once

#include "ITrajectoryProfile.h"

// Jerk-limited (7-segment "S-curve") profile -- a faithful port of the
// MATLAB reference in Resources/MATLAB/S_Curve_Gen/, exactly as
// TrapezoidalProfile is a port of Resources/MATLAB/Trap_Traj_Gen/. The
// planning math (branch selection, segment times, boundary conditions) is
// transcribed, not re-derived from textbook equations.
//
// The seven phases, in MATLAB's t1..t7 parametrisation (note t1..t7 are
// absolute *times*, not durations):
//
//   [0,  t1)  jerk = +J    ramp acceleration up
//   [t1, t2)  jerk =  0    constant acceleration
//   [t2, t3)  jerk = -J    ramp acceleration down to zero
//   [t3, t4)  jerk =  0    cruise at constant velocity
//   [t4, t5)  jerk = -J    ramp deceleration up
//   [t5, t6)  jerk =  0    constant deceleration
//   [t6, t7)  jerk = +J    ramp deceleration down to zero
//   [t7, inf) settled at qf
//
// Degenerate moves collapse phases by making them zero-length (t2==t1 and
// t4==t3 in the triangular-acceleration cases, for instance); no branch in
// evaluate() is special-cased for them, which is precisely how the MATLAB
// behaves.
class SCurveProfile : public ITrajectoryProfile {
public:
    // Same rationale as TrapezoidalProfile's: declaring the 4-arg override
    // below would otherwise hide ITrajectoryProfile's 3-arg convenience
    // overload for any call made through an SCurveProfile-typed expression,
    // since C++ member lookup stops at the first class scope where the name
    // appears at all, before overload resolution runs.
    using ITrajectoryProfile::plan;

    // Returns false (never throws -- no exceptions on any path in this
    // library) in two cases, both of which leave the object in a safe,
    // well-defined state:
    //  - limits.vMax, limits.aMax or limits.jMax is not strictly positive
    //    (NaN included, since NaN fails every comparison): the profile is
    //    left invalid and evaluate() reports the axis parked at q0, never
    //    moving. Note jMax == 0 -- documented in TrajectoryLimits as
    //    "trapezoidal, ignored" -- is rejected here rather than silently
    //    degrading to a trapezoidal profile: picking the profile type is
    //    the caller's job, and quietly emitting a different profile shape
    //    than the class name promises would be worse than a clear false.
    //  - targetDuration > 0 but shorter than this move's own minimum-time
    //    duration (a "dilation" that would actually have to speed the move
    //    up): the profile falls back to the minimum-time plan rather than
    //    violating the limits, and getDuration() reports that clamped
    //    duration, not the one requested. Matches TrapezoidalProfile.
    bool plan(float q0, float qf, const TrajectoryLimits& limits,
              float targetDuration) override;

    bool evaluate(float t, float& pos, float& vel, float& accel) const override;

    float getDuration() const override;

private:
    // Port of SCurve_calculateTimeSegments.m. Writes the seven absolute
    // segment times into tOut[0..6] (MATLAB's t1..t7). Caller guarantees
    // jMax/aMax/vMax are strictly positive and displacement >= 0.
    static void computeSegments(float displacement, float vMax, float aMax,
                                float jMax, float tOut[7]);

    // Port of SCurve_calculateInitialConditions.m: chains the seven
    // boundary states so evaluate() only ever needs one cubic.
    void computeInitialConditions();

    float _q0       = 0.0f;
    float _qf       = 0.0f;
    float _polarity = 0.0f;  // sign(qf - q0); 0 for a zero-distance move
    float _jerk     = 0.0f;  // unsigned jerk actually used (post-dilation)

    float _t[7] = {0,0,0,0,0,0,0};  // MATLAB t1..t7
    float _a[7] = {0,0,0,0,0,0,0};  // acceleration at each t[i]
    float _v[7] = {0,0,0,0,0,0,0};  // velocity     at each t[i]
    float _s[7] = {0,0,0,0,0,0,0};  // position     at each t[i]

    bool _valid = false;
};
