#pragma once
#include "IPathGeometry.h"

// Circular arc in 3D space parameterised by arc length.
//
// center:      arc centre
// startPoint:  point on the arc at angle 0 — determines radius and the u-axis
// normal:      arc-plane normal; right-hand rule: positive sweep is CCW when
//              viewed from the normal direction
// sweepAngle:  total angle in radians (positive = CCW, negative = CW)
class ArcPath : public IPathGeometry {
public:
    ArcPath() = default;
    ArcPath(const Vec3& center, const Vec3& startPoint,
            const Vec3& normal,  float sweepAngle);

    void  evaluate(float s, Vec3& position, Vec3& tangent) const override;
    float getLength() const override;

private:
    Vec3  _center;
    Vec3  _u;              // unit radial vector at angle 0 (toward startPoint)
    Vec3  _v;              // unit tangent at angle 0 for positive sweep (= normal × u)
    float _radius = 0.0f;
    float _sign   = 1.0f;  // +1 CCW, -1 CW
    float _length = 0.0f;  // |sweepAngle| * radius
};
