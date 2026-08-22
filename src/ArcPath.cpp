#include "ArcPath.h"
#include <math.h>

ArcPath::ArcPath(const Vec3& center, const Vec3& startPoint,
                 const Vec3& normal,  float sweepAngle)
    : _center(center) {
    Vec3 radial = startPoint - center;
    _radius = radial.magnitude();
    _u      = (_radius > 0.0f) ? radial / _radius : Vec3{1.0f, 0.0f, 0.0f};
    _v      = normal.normalized().cross(_u).normalized();
    _sign   = (sweepAngle >= 0.0f) ? 1.0f : -1.0f;
    _length = fabsf(sweepAngle) * _radius;
}

// position(s) = center + radius*(cos(theta)*u + sin(theta)*v),  theta = sign*s/radius
// tangent (unit) = d(position)/ds
//                = sign * (-sin(theta)*u + cos(theta)*v)
void ArcPath::evaluate(float s, Vec3& position, Vec3& tangent) const {
    // Degenerate arc (startPoint == center -> radius 0, getLength() == 0):
    // theta = s/_radius would be a 0/0 division producing NaN. There's no
    // meaningful direction of travel on a zero-radius arc, so just report
    // the single point and the in-plane axis established at construction
    // time instead of propagating NaN into position/tangent.
    if (_radius <= 0.0f) {
        position = _center;
        tangent  = _v;
        return;
    }

    float theta = _sign * s / _radius;
    float c = cosf(theta), sn = sinf(theta);
    position = _center + _u * (_radius * c) + _v * (_radius * sn);
    tangent  = _v * (_sign * c) - _u * (_sign * sn);
}

float ArcPath::getLength() const { return _length; }
