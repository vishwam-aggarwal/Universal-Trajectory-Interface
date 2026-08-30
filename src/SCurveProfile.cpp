#include "SCurveProfile.h"
#include <math.h>   // not <cmath>: avr-gcc ships no C++ standard library

// Port of SCurve_calculateMotion.m -- the one kinematic step every phase
// uses. Kept as a file-local helper so the transcription stays visible:
//     a = a0 + J*dt
//     v = v0 + a0*dt + J*dt^2/2
//     s = s0 + v0*dt + a0*dt^2/2 + J*dt^3/6
static inline void scurveMotion(float a0, float v0, float s0, float t0,
                                float jerk, float T,
                                float& a, float& v, float& s) {
    const float dt = T - t0;
    a = a0 + jerk * dt;
    v = v0 + a0 * dt + jerk * dt * dt * 0.5f;
    s = s0 + v0 * dt + a0 * dt * dt * 0.5f + jerk * dt * dt * dt / 6.0f;
}

// Port of SCurve_calculateTimeSegments.m, branch for branch.
void SCurveProfile::computeSegments(float displacement, float vMax, float aMax,
                                    float jMax, float tOut[7]) {
    const float v_a = (aMax * aMax) / jMax;
    const float s_a = 2.0f * (aMax * aMax * aMax) / (jMax * jMax);

    float s_v;
    if (vMax * jMax < aMax * aMax) {
        s_v = 2.0f * vMax * sqrtf(vMax / jMax);
    } else {
        s_v = vMax * ((vMax / aMax) + (aMax / jMax));
    }

    // The MATLAB four-way if/elseif chain has no else -- the four
    // conditions are exhaustive over the two independent comparisons, so
    // one always fires. Initialised anyway so no path can leave these
    // indeterminate.
    float t_j = 0.0f, t_a = 0.0f, t_v = 0.0f;

    if ((vMax < v_a) && (displacement >= s_a)) {                    // A
        t_j = sqrtf(vMax / jMax);
        t_a = t_j;
        t_v = displacement / vMax;
    } else if ((vMax >= v_a) && (displacement < s_a)) {             // B
        t_j = cbrtf(displacement / (2.0f * jMax));
        t_a = t_j;
        t_v = 2.0f * t_j;
    } else if ((vMax < v_a) && (displacement < s_a)) {              // C
        if (displacement >= s_v) {                                  // C.1
            t_j = sqrtf(vMax / jMax);
            t_a = t_j;
            t_v = displacement / vMax;
        } else {                                                    // C.2
            t_j = cbrtf(displacement / (2.0f * jMax));
            t_a = t_j;
            t_v = 2.0f * t_j;
        }
    } else if ((vMax >= v_a) && (displacement >= s_a)) {            // D
        if (displacement >= s_v) {                                  // D.1
            t_j = aMax / jMax;
            t_a = vMax / aMax;
            t_v = displacement / vMax;
        } else {                                                    // D.2
            t_j = aMax / jMax;
            t_a = 0.5f * (sqrtf((4.0f * displacement * (jMax * jMax)
                                 + (aMax * aMax * aMax))
                                / (aMax * (jMax * jMax)))
                          - (aMax / jMax));
            t_v = t_a + t_j;
        }
    }

    tOut[0] = t_j;              // t1
    tOut[1] = t_a;              // t2
    tOut[2] = t_j + t_a;        // t3
    tOut[3] = t_v;              // t4
    tOut[4] = t_j + t_v;        // t5
    tOut[5] = t_v + t_a;        // t6
    tOut[6] = t_v + t_a + t_j;  // t7
}

// The signed jerk applied during each of the seven phases, in order,
// multiplied by polarity at use. Mirrors the jerk argument passed at each
// step of SCurve_calculateInitialConditions.m / SCurve_CalculateTraj.m.
static const float kJerkSign[7] = { 1.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f, 1.0f };

// Port of SCurve_calculateInitialConditions.m: walk the seven phases once
// at plan time and cache the state at every boundary, so evaluate() is a
// single cubic rather than a re-walk.
void SCurveProfile::computeInitialConditions() {
    float a = 0.0f, v = 0.0f, s = _q0, t0 = 0.0f;

    for (int i = 0; i < 7; ++i) {
        const float jerk = kJerkSign[i] * _jerk * _polarity;
        scurveMotion(a, v, s, t0, jerk, _t[i], a, v, s);
        _a[i] = a;
        _v[i] = v;
        _s[i] = s;
        t0 = _t[i];
    }
}

bool SCurveProfile::plan(float q0, float qf, const TrajectoryLimits& limits,
                         float targetDuration) {
    _q0    = q0;
    _qf    = qf;
    _valid = false;

    const float displacement = fabsf(qf - q0);
    _polarity = (qf > q0) ? 1.0f : ((qf < q0) ? -1.0f : 0.0f);  // MATLAB sign()

    // All three limits must be strictly positive. Written as !(x > 0) so
    // NaN -- which fails every comparison -- is rejected too, rather than
    // propagating into every segment time. jMax == 0 means "trapezoidal"
    // per TrajectoryLimits and is not a valid S-curve; see the header.
    if (!(limits.vMax > 0.0f) || !(limits.aMax > 0.0f) || !(limits.jMax > 0.0f)) {
        _jerk = 0.0f;
        for (int i = 0; i < 7; ++i) { _t[i] = _a[i] = _v[i] = _s[i] = 0.0f; }
        return false;
    }

    if (displacement == 0.0f) {
        // Nothing to do; duration 0, parked at qf (== q0). Matches
        // TrapezoidalProfile, and is what the MATLAB degenerates to.
        _jerk = 0.0f;
        for (int i = 0; i < 7; ++i) { _t[i] = 0.0f; _a[i] = 0.0f; _v[i] = 0.0f; _s[i] = q0; }
        _valid = true;
        return true;
    }

    // Minimum-time plan first: it is the answer when no dilation is asked
    // for, and the floor the dilation branch needs either way.
    computeSegments(displacement, limits.vMax, limits.aMax, limits.jMax, _t);
    _jerk = limits.jMax;

    const float minDuration = _t[6];
    bool requestSatisfied = true;

    if (targetDuration > minDuration && minDuration > 0.0f) {
        // Time dilation. The MATLAB has no notion of a target duration, so
        // rather than deriving a new closed form (the S-curve duration does
        // not invert as cleanly as the trapezoid quadratic), re-run the
        // *identical* segment math on uniformly scaled limits.
        //
        // Substituting (vMax/k, aMax/k^2, jMax/k^3) leaves every branch
        // predicate unchanged (v_a scales by 1/k alongside vMax; s_a and
        // s_v are invariant) and scales all seven segment times by exactly
        // k -- so the profile shape is preserved and the duration becomes
        // exactly targetDuration. Verified against the MATLAB for every
        // case (A/B/C.1/C.2/D.1/D.2) at several k. Because k > 1 here,
        // every effective limit shrinks, so none of vMax/aMax/jMax can be
        // exceeded by the dilated move.
        const float k = targetDuration / minDuration;
        _jerk = limits.jMax / (k * k * k);
        computeSegments(displacement, limits.vMax / k, limits.aMax / (k * k),
                        _jerk, _t);
    } else if (targetDuration > 0.0f && targetDuration < minDuration) {
        // Asked to go faster than the limits allow. Keep the minimum-time
        // plan and report the request unmet, exactly as TrapezoidalProfile
        // does, rather than silently violating a limit.
        requestSatisfied = false;
    }

    computeInitialConditions();
    _valid = true;
    return requestSatisfied;
}

// Port of the SCurve_CalculateTraj.m phase selector plus
// SCurve_calculateMotion.m. Allocation-free, exception-free, and bounded at
// seven iterations -- safe for the cyclic EtherCAT path.
bool SCurveProfile::evaluate(float t, float& pos, float& vel, float& accel) const {
    if (!_valid) {
        pos = _q0; vel = 0.0f; accel = 0.0f;
        return false;
    }

    if (t < 0.0f) t = 0.0f;

    if (_t[6] == 0.0f || t >= _t[6]) {
        // Settled. Snapped to _qf rather than reporting the accumulated
        // _s[6]: the MATLAB lands on the target to within double epsilon in
        // every case, but seven chained cubics in float drift a little, and
        // a residual standing offset on a joint is worse than a rounding
        // error mid-profile. TrapezoidalProfile snaps identically.
        pos = _qf; vel = 0.0f; accel = 0.0f;
        return false;
    }

    for (int i = 0; i < 7; ++i) {
        if (t < _t[i]) {
            const float jerk = kJerkSign[i] * _jerk * _polarity;
            if (i == 0) {
                scurveMotion(0.0f, 0.0f, _q0, 0.0f, jerk, t, accel, vel, pos);
            } else {
                scurveMotion(_a[i - 1], _v[i - 1], _s[i - 1], _t[i - 1],
                             jerk, t, accel, vel, pos);
            }
            return true;
        }
    }

    // Unreachable: t < _t[6] was established above, so some phase matched.
    pos = _qf; vel = 0.0f; accel = 0.0f;
    return false;
}

float SCurveProfile::getDuration() const {
    return _t[6];
}
