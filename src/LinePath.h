#pragma once
#include "IPathGeometry.h"

// Straight-line path from p0 to p1 parameterised by arc length.
class LinePath : public IPathGeometry {
public:
    LinePath() = default;
    LinePath(const Vec3& p0, const Vec3& p1);

    void  evaluate(float s, Vec3& position, Vec3& tangent) const override;
    float getLength() const override;

private:
    Vec3  _p0;
    Vec3  _dir;            // unit direction from p0 to p1
    float _length = 0.0f;
};
