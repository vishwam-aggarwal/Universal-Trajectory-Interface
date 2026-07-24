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

    float vMax = limits.vMax;
    float aMax = limits.aMax;
    float t_a, t_v;

    if (targetDuration <= 0.0f) {
        // Minimum-time plan — port of Trap_calculateTimeSegments.m
        float s_v = (vMax * vMax) / aMax;
        if (displacement >= s_v) {
            t_a    = vMax / aMax;
            t_v    = t_a + (displacement - s_v) / vMax;
            _vPeak = vMax;
        } else {
            t_a    = sqrtf(displacement / aMax);
            t_v    = t_a;
            _vPeak = aMax * t_a;
        }
    } else {
        // Time-dilation: find vPeak such that profile takes exactly targetDuration.
        // From t3 = vPeak/aMax + displacement/vPeak, solving the quadratic:
        //   vPeak^2/aMax - T*vPeak + displacement = 0
        float T    = targetDuration;
        float disc = T * T - 4.0f * displacement / aMax;
        if (disc < 0.0f) disc = 0.0f;
        _vPeak = (aMax / 2.0f) * (T - sqrtf(disc));
        t_a    = _vPeak / aMax;
        float cruise_dist = displacement - (_vPeak * _vPeak / aMax);
        t_v    = (cruise_dist > 0.0f) ? t_a + cruise_dist / _vPeak : t_a;
    }

    _t1    = t_a;
    _t2    = t_v;
    _t3    = t_a + t_v;
    _valid = true;
    return true;
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
