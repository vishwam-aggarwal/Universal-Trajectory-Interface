#include "CartesianMove.h"

bool CartesianMove::plan(IPathGeometry*      path,
                          const Quatf&        startOrientation,
                          const Quatf&        endOrientation,
                          ITrajectoryProfile* profile,
                          const TrajectoryLimits& limits,
                          float targetDuration) {
    _path    = path;
    _profile = profile;
    _q0      = startOrientation;
    _q1      = endOrientation;
    _length  = path->getLength();
    return _profile->plan(0.0f, _length, limits, targetDuration);
}

bool CartesianMove::evaluate(float t,
                              Vec3&  position,
                              Vec3&  velocity,
                              Vec3&  accelTangential,
                              Quatf& orientation) const {
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
