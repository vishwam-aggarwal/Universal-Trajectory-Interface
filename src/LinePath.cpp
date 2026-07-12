#include "LinePath.h"

LinePath::LinePath(const Vec3& p0, const Vec3& p1) : _p0(p0) {
    Vec3 delta = p1 - p0;
    _length = delta.magnitude();
    _dir    = (_length > 0.0f) ? delta / _length : Vec3{1.0f, 0.0f, 0.0f};
}

void LinePath::evaluate(float s, Vec3& position, Vec3& tangent) const {
    position = _p0 + _dir * s;
    tangent  = _dir;
}

float LinePath::getLength() const { return _length; }
