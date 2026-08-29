#include "TrajectoryGroup.h"

bool TrajectoryGroup::plan(ITrajectoryProfile** profiles,
                            const float*         q0,
                            const float*         qf,
                            const TrajectoryLimits* limits,
                            int count) {
    if (count < 1 || count > MAX_AXES) {
        invalidatePlan();
        setError(ERR_INVALID_AXIS_COUNT);
        return false;
    }

    if (profiles == nullptr) {
        invalidatePlan();
        setError(ERR_NULL_PROFILE);
        return false;
    }
    for (int i = 0; i < count; ++i) {
        if (profiles[i] == nullptr) {
            invalidatePlan();
            setError(ERR_NULL_PROFILE);
            return false;
        }
    }

    // Phase 1: plan each axis at minimum time, find the slowest.
    //
    // Each axis's own plan() return value is checked. It is genuinely
    // reachable -- TrapezoidalProfile rejects a non-positive or NaN
    // vMax/aMax and leaves that axis parked -- and before this was checked,
    // the group reported overall success while one axis silently never
    // moved. Bail out on the first failure rather than planning the rest:
    // a synchronized group with a dead axis has no useful partial answer.
    float maxDuration = 0.0f;
    for (int i = 0; i < count; ++i) {
        if (!profiles[i]->plan(q0[i], qf[i], limits[i])) {
            invalidatePlan();
            setError(ERR_AXIS_PLAN_FAILED);
            return false;
        }
        float d = profiles[i]->getDuration();
        if (d > maxDuration) maxDuration = d;
    }

    // Phase 2: re-plan every axis to the shared duration so they arrive
    // together. Checked for the same reason as phase 1 -- time-dilating to
    // a longer duration is the easier problem, so a failure here means
    // something phase 1 didn't catch, and ignoring it would leave that axis
    // holding its phase-1 (unsynchronized) plan.
    for (int i = 0; i < count; ++i) {
        if (!profiles[i]->plan(q0[i], qf[i], limits[i], maxDuration)) {
            invalidatePlan();
            setError(ERR_AXIS_PLAN_FAILED);
            return false;
        }
    }

    // Commit only once every axis has planned successfully.
    for (int i = 0; i < count; ++i) {
        _profiles[i] = profiles[i];
    }
    _count    = count;
    _duration = maxDuration;
    _error    = ERR_NONE;
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

// ------------------------------------------------------------
// IDevice
// ------------------------------------------------------------
bool TrajectoryGroup::begin() {
    _error = ERR_NONE;
    return true;
}

DeviceState TrajectoryGroup::getState() const {
    if (_error != ERR_NONE) return DeviceState::ERRORED;
    return DeviceState::IDLE;   // never BUSY -- see the header
}

uint32_t TrajectoryGroup::getStatus() const {
    return (_count > 0) ? STATUS_PLANNED : STATUS_NONE;
}

uint32_t TrajectoryGroup::getError() const { return _error; }

const char* TrajectoryGroup::getStatusString(uint32_t status) const {
    switch (status) {
        case STATUS_NONE:    return "No plan loaded";
        case STATUS_PLANNED: return "Plan loaded";
        default:             return "Unknown status";
    }
}

const char* TrajectoryGroup::getErrorString(uint32_t err) const {
    switch (err) {
        case ERR_NONE:               return "No error";
        case ERR_INVALID_AXIS_COUNT: return "Axis count outside [1, MAX_AXES]";
        case ERR_NULL_PROFILE:       return "A null ITrajectoryProfile pointer was supplied";
        case ERR_AXIS_PLAN_FAILED:   return "An axis profile's plan() failed (check vMax/aMax are positive and finite)";
        default:                     return "Unknown error";
    }
}

void TrajectoryGroup::setError(uint32_t err) {
    _error = err;
    reportError("TrajectoryGroup", err);
}

void TrajectoryGroup::invalidatePlan() {
    _count    = 0;
    _duration = 0.0f;
}
