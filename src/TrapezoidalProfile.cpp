#include "TrapezoidalProfile.h"
#include <math.h>

bool TrapezoidalProfile::plan(float q0, float qf, const TrajectoryLimits& limits,
                               float targetDuration) {
    _q0   = q0;
    _qf   = qf;
    _dir  = (qf >= q0) ? 1.0f : -1.0f;
    _aMax = limits.aMax;
    _valid = false;

    float displacement = fabsf(qf - q0);

    if (displacement == 0.0f) {
        _vPeak = 0.0f;
        _t1 = _t2 = _t3 = 0.0f;
        _valid = true;
        return true;
    }

    // A real move needs both limits to be strictly positive -- vMax<=0 or
    // aMax<=0 (including NaN, which fails every comparison) would otherwise
    // divide by zero below and silently produce an infinite duration. Leave
    // the profile invalid rather than fabricate a nonsensical one; the axis
    // simply never moves (evaluate() reports parked at q0) until re-planned
    // with sane limits.
    if (!(limits.vMax > 0.0f) || !(limits.aMax > 0.0f)) {
        _vPeak = 0.0f;
        _t1 = _t2 = _t3 = 0.0f;
        return false;
    }

    float vMax = limits.vMax;
    float aMax = limits.aMax;

    // Minimum-time plan — port of Trap_calculateTimeSegments.m. Always
    // computed first: it's the answer when targetDuration<=0, and it's also
    // the fastest this move can possibly go, which the time-dilation branch
    // below needs as a floor.
    float s_v = (vMax * vMax) / aMax;
    float minT_a, minT_v, minVPeak;
    if (displacement >= s_v) {
        minT_a   = vMax / aMax;
        minT_v   = minT_a + (displacement - s_v) / vMax;
        minVPeak = vMax;
    } else {
        minT_a   = sqrtf(displacement / aMax);
        minT_v   = minT_a;
        minVPeak = aMax * minT_a;
    }
    float minDuration = minT_a + minT_v;

    float t_a, t_v;
    bool requestSatisfied = true;

    if (targetDuration <= 0.0f || targetDuration <= minDuration) {
        // Either no dilation requested, or the requested duration is not
        // actually slower than the minimum-time plan -- use the min-time
        // plan as-is. (Solving the quadratic below for T <= minDuration
        // would produce a vPeak that exceeds vMax, or -- for T below the
        // absolute aMax-only floor -- a nonsensical duration far longer
        // than either T or minDuration; see the regression tests.)
        t_a    = minT_a;
        t_v    = minT_v;
        _vPeak = minVPeak;
        if (targetDuration > 0.0f && targetDuration < minDuration) {
            requestSatisfied = false;
        }
    } else {
        // Time-dilation: find vPeak such that profile takes exactly targetDuration.
        // From t3 = vPeak/aMax + displacement/vPeak, solving the quadratic:
        //   vPeak^2/aMax - T*vPeak + displacement = 0
        // Since T > minDuration >= the aMax-only floor here, the
        // discriminant is guaranteed non-negative and the resulting vPeak
        // is guaranteed <= vMax (vPeak(T) is monotonically decreasing for
        // T past the floor, and vPeak(minDuration) == minVPeak <= vMax).
        float T    = targetDuration;
        float disc = T * T - 4.0f * displacement / aMax;
        if (disc < 0.0f) disc = 0.0f;  // precision guard; not expected to trigger
        _vPeak = (aMax / 2.0f) * (T - sqrtf(disc));
        t_a    = _vPeak / aMax;
        float cruise_dist = displacement - (_vPeak * _vPeak / aMax);
        t_v    = (cruise_dist > 0.0f) ? t_a + cruise_dist / _vPeak : t_a;
    }

    _t1    = t_a;
    _t2    = t_v;
    _t3    = t_a + t_v;
    _valid = true;
    return requestSatisfied;
}

// Evaluates kinematics at time t using the same phased kinematic equations
// as Trap_calculateMotion.m: s = s0 + v0*(t-t0) + 0.5*a*(t-t0)^2
bool TrapezoidalProfile::evaluate(float t, float& pos, float& vel, float& accel) const {
    if (!_valid) {
        pos = _q0; vel = 0.0f; accel = 0.0f;
        return false;
    }

    if (t < 0.0f) t = 0.0f;

    if (_t3 == 0.0f || t >= _t3) {
        pos = _qf; vel = 0.0f; accel = 0.0f;
        return false;
    }

    float a = _aMax * _dir;

    if (t < _t1) {
        // Acceleration phase (v0=0, s0=q0, t0=0)
        pos   = _q0 + 0.5f * a * t * t;
        vel   =              a * t;
        accel = a;
    } else if (t < _t2) {
        // Cruise phase (v0=v1, s0=s1, t0=t1, a=0)
        float v1 = a * _t1;
        float s1 = _q0 + 0.5f * a * _t1 * _t1;
        float dt = t - _t1;
        pos   = s1 + v1 * dt;
        vel   = v1;
        accel = 0.0f;
    } else {
        // Deceleration phase (v0=v1, s0=s2, t0=t2, a=-a)
        float v1 = a * _t1;
        float s1 = _q0 + 0.5f * a * _t1 * _t1;
        float s2 = s1 + v1 * (_t2 - _t1);
        float dt = t - _t2;
        pos   = s2 + v1 * dt - 0.5f * a * dt * dt;
        vel   = v1           -        a * dt;
        accel = -a;
    }

    return true;
}

float TrapezoidalProfile::getDuration() const {
    return _t3;
}
