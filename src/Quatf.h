#pragma once
#include <cmath>

// Unit quaternion representing orientation. Convention: w + xi + yj + zk.
struct Quatf {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;

    Quatf() = default;
    Quatf(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    float dot(const Quatf& o) const {
        return w*o.w + x*o.x + y*o.y + z*o.z;
    }

    Quatf normalized() const {
        float m = sqrtf(dot(*this));
        return (m > 0.0f) ? Quatf{w/m, x/m, y/m, z/m} : Quatf{};
    }

    // Shortest-path spherical linear interpolation, t in [0, 1].
    static Quatf slerp(Quatf a, Quatf b, float t) {
        float d = a.dot(b);
        if (d < 0.0f) { b = {-b.w, -b.x, -b.y, -b.z}; d = -d; }

        if (d > 0.9995f) {
            return Quatf{ a.w + t*(b.w - a.w),
                          a.x + t*(b.x - a.x),
                          a.y + t*(b.y - a.y),
                          a.z + t*(b.z - a.z) }.normalized();
        }

        float omega  = acosf(d);
        float sinOmg = sinf(omega);
        float s0 = sinf((1.0f - t) * omega) / sinOmg;
        float s1 = sinf(       t   * omega) / sinOmg;

        return { s0*a.w + s1*b.w,
                 s0*a.x + s1*b.x,
                 s0*a.y + s1*b.y,
                 s0*a.z + s1*b.z };
    }
};
