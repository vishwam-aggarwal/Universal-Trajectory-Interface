#include "CartesianMove.h"

bool CartesianMove::plan(IPathGeometry*      path,
                          const Quatf&        startOrientation,
                          const Quatf&        endOrientation,
                          ITrajectoryProfile* profile,
                          const TrajectoryLimits& limits,
                          float targetDuration) {
    // Validate before storing anything: a failed plan() must leave this
    // object inert, not half-configured pointing at one good pointer and
    // one null. (Previously both pointers were dereferenced immediately --
    // path->getLength() and profile->plan() -- so a null argument crashed
    // rather than reporting.)
    if (path == nullptr) {
        invalidatePlan();
        setError(ERR_NULL_PATH);
        return false;
    }
    if (profile == nullptr) {
        invalidatePlan();
        setError(ERR_NULL_PROFILE);
        return false;
    }

    const float length = path->getLength();

    // A zero-length path is legal, not an error: the profile plans a
    // zero-displacement move (settled immediately) and orientation still
    // SLERPs to the end pose -- a pure re-orientation in place.
    if (!profile->plan(0.0f, length, limits, targetDuration)) {
        invalidatePlan();
        setError(ERR_PROFILE_PLAN_FAILED);
        return false;
    }

    _path    = path;
    _profile = profile;
    _q0      = startOrientation;
    _q1      = endOrientation;
    _length  = length;
    _error   = ERR_NONE;
    return true;
}

bool CartesianMove::evaluate(float t,
                              Vec3&  position,
                              Vec3&  velocity,
                              Vec3&  accelTangential,
                              Quatf& orientation) const {
    // Inert with no plan loaded -- mirrors TrajectoryGroup::evaluate() with
    // _count == 0, and is what makes a failed plan() safe to ignore.
    if (_path == nullptr || _profile == nullptr) return false;

    float s, speed, tangAccel;
    bool moving = _profile->evaluate(t, s, speed, tangAccel);

    Vec3 tangent;
    _path->evaluate(s, position, tangent);
    velocity        = tangent * speed;
    accelTangential = tangent * tangAccel;

    float u = (_length > 0.0f) ? s / _length : 1.0f;
    orientation = Quatf::slerp(_q0, _q1, u);

    return moving;
}

float CartesianMove::getDuration() const {
    return (_profile != nullptr) ? _profile->getDuration() : 0.0f;
}

// ------------------------------------------------------------
// IDevice
// ------------------------------------------------------------
bool CartesianMove::begin() {
    _error = ERR_NONE;
    return true;
}

DeviceState CartesianMove::getState() const {
    if (_error != ERR_NONE) return DeviceState::ERRORED;
    return DeviceState::IDLE;   // never BUSY -- see the header
}

uint32_t CartesianMove::getStatus() const {
    return (_path != nullptr && _profile != nullptr) ? STATUS_PLANNED : STATUS_NONE;
}

uint32_t CartesianMove::getError() const { return _error; }

const char* CartesianMove::getStatusString(uint32_t status) const {
    switch (status) {
        case STATUS_NONE:    return "No plan loaded";
        case STATUS_PLANNED: return "Plan loaded";
        default:             return "Unknown status";
    }
}

const char* CartesianMove::getErrorString(uint32_t err) const {
    switch (err) {
        case ERR_NONE:                return "No error";
        case ERR_NULL_PATH:           return "A null IPathGeometry pointer was supplied";
        case ERR_NULL_PROFILE:        return "A null ITrajectoryProfile pointer was supplied";
        case ERR_PROFILE_PLAN_FAILED: return "The scalar profile's plan() failed (check vMax/aMax are positive and finite)";
        default:                      return "Unknown error";
    }
}

void CartesianMove::setError(uint32_t err) {
    _error = err;
    reportError("CartesianMove", err);
}

void CartesianMove::invalidatePlan() {
    _path    = nullptr;
    _profile = nullptr;
    _length  = 0.0f;
}
