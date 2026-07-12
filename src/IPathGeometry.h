#pragma once
#include "Vec3.h"

class IPathGeometry {
public:
    virtual ~IPathGeometry() = default;

    // s in [0, getLength()]. Returns Cartesian position and unit tangent
    // direction at arc-length distance s along the path.
    virtual void evaluate(float s, Vec3& position, Vec3& tangent) const = 0;

    virtual float getLength() const = 0;
};
