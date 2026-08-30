#include "JerkPercentProfile.h"
#include <math.h>   // not <cmath>: avr-gcc ships no C++ standard library

// Script.m's floor on the trapezoid's acceleration time, kept verbatim:
//     T_A = max(1e-6, SEGMENTS.t1);
// It stops the jerk derivation dividing by zero on a move whose trapezoidal
// accel phase has zero duration (a zero-distance move, where t_a =
// sqrt(0/A) = 0). Note it is an absolute figure in seconds, so it is
// unit-dependent -- but it only ever engages on moves far too short to
// execute anyway, and it bounds the derived jerk rather than letting it run
// to infinity.
static const float kMinAccelTime = 1e-6f;

TrajectoryLimits JerkPercentProfile::deriveLimits(float q0, float qf,
                                                  const TrajectoryLimits& nominal,
                                                  float jerkPercent) {
    // Invalid input -> all-zero limits, which every profile rejects.
    // plan() validates before calling this, so this is a guard for direct
    // callers rather than a path plan() relies on.
    if (!(nominal.vMax > 0.0f) || !(nominal.aMax > 0.0f) ||
        !(jerkPercent > 0.0f)  || !(jerkPercent <= 100.0f)) {
        return TrajectoryLimits(0.0f, 0.0f, 0.0f);
    }

    const float A = nominal.aMax;   // NOMINAL acceleration, not a ceiling
    const float V = nominal.vMax;
    const float d = fabsf(qf - q0);

    // Port of Trap_calculateTimeSegments.m, but only its t1 (== t_a, the
    // acceleration-phase duration) -- that is all Script.m consumes:
    //     [SEGMENTS, polarity] = Trap_calculateTimeSegments(...)
    //     T_A = max(1e-6, SEGMENTS.t1);
    // Recomputed here rather than routing through TrapezoidalProfile so
    // this stays a dependency-free static that allocates nothing and plans
    // nothing.
    const float s_v = (V * V) / A;
    const float t_a = (d >= s_v) ? (V / A) : sqrtf(d / A);

    const float T_A = (t_a > kMinAccelTime) ? t_a : kMinAccelTime;

    // Script.m:
    //     accLimit  = 2*ACCLimit/(2 - (jerkPercent/100))
    //     jerkLimit = 2*accLimit/(T_A*(jerkPercent/100))
    const float p  = jerkPercent / 100.0f;
    const float A2 = 2.0f * A / (2.0f - p);
    const float J  = 2.0f * A2 / (T_A * p);

    return TrajectoryLimits(V, A2, J);
}

bool JerkPercentProfile::plan(float q0, float qf, const TrajectoryLimits& limits,
                              float targetDuration) {
    // jerkPercent must be in (0, 100]. 0 divides by zero in the derivation
    // and means "trapezoid" anyway; above 100 makes A' = 2A/(2-p) blow up
    // (and turn negative past p = 2). Script.m does not validate -- the
    // sibling helper calculate_jerk_and_acceleration.m does, with exactly
    // this 0..100 range -- and this library's convention is to report
    // rather than trust, so it is enforced here. Written as !(x > 0) so NaN
    // is rejected too.
    if (!(_jerkPercent > 0.0f) || !(_jerkPercent <= 100.0f) ||
        !(limits.vMax > 0.0f)  || !(limits.aMax > 0.0f)) {
        _derived = TrajectoryLimits(0.0f, 0.0f, 0.0f);
        // Drive the inner profile into its own invalid state so a
        // subsequent evaluate() is inert and parked at q0 rather than
        // replaying whatever move was planned before this failed call.
        _inner.plan(q0, qf, TrajectoryLimits(0.0f, 0.0f, 0.0f), 0.0f);
        return false;
    }

    _derived = deriveLimits(q0, qf, limits, _jerkPercent);

    // The derivation divides by T_A, which the kMinAccelTime floor keeps
    // away from zero, so J is finite for any sane input. Guard anyway: an
    // absurd nominal aMax could still overflow it, and a non-finite jerk
    // would sail past SCurveProfile's own !(jMax > 0) check (infinity is
    // greater than zero) and poison every segment time.
    if (!(_derived.jMax > 0.0f) || !isfinite(_derived.jMax) ||
        !(_derived.aMax > 0.0f) || !isfinite(_derived.aMax)) {
        _derived = TrajectoryLimits(0.0f, 0.0f, 0.0f);
        _inner.plan(q0, qf, TrajectoryLimits(0.0f, 0.0f, 0.0f), 0.0f);
        return false;
    }

    // Run the unmodified 7-segment engine on the derived limits. Time
    // dilation, if requested, is handled inside SCurveProfile by uniform
    // limit scaling -- which is shape-preserving, so the jerk-percentage
    // character of the profile survives being stretched.
    return _inner.plan(q0, qf, _derived, targetDuration);
}
