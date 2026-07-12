#include "TrajectoryGroup.h"

bool TrajectoryGroup::plan(ITrajectoryProfile** profiles,
                            const float*         q0,
                            const float*         qf,
                            const TrajectoryLimits* limits,
                            int count) {
    if (count < 1 || count > MAX_AXES) return false;

    _count = count;

    // Phase 1: plan each axis at minimum time, find the slowest
    float maxDuration = 0.0f;
    for (int i = 0; i < count; ++i) {
        _profiles[i] = profiles[i];
        _profiles[i]->plan(q0[i], qf[i], limits[i]);
        float d = _profiles[i]->getDuration();
        if (d > maxDuration) maxDuration = d;
    }

    _duration = maxDuration;

    // Phase 2: re-plan every axis to the shared duration so they arrive together
    for (int i = 0; i < count; ++i) {
        _profiles[i]->plan(q0[i], qf[i], limits[i], maxDuration);
    }

    return true;
}

bool TrajectoryGroup::evaluate(float t, float* pos, float* vel, float* accel) const {
    bool anyMoving = false;
    for (int i = 0; i < _count; ++i) {
        if (_profiles[i]->evaluate(t, pos[i], vel[i], accel[i]))
            anyMoving = true;
    }
    return anyMoving;
}

float TrajectoryGroup::getDuration() const { return _duration; }
int   TrajectoryGroup::getCount()    const { return _count; }
