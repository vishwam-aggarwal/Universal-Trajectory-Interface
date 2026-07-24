#pragma once
#include <math.h>

struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;

    Vec3() = default;
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(float s)       const { return {x*s,   y*s,   z*s};   }
    Vec3 operator/(float s)       const { return {x/s,   y/s,   z/s};   }

    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator*=(float s)       { x*=s;   y*=s;   z*=s;   return *this; }

    float dot(const Vec3& o)   const { return x*o.x + y*o.y + z*o.z; }
    float magnitudeSq()        const { return dot(*this); }
    float magnitude()          const { return sqrtf(magnitudeSq()); }

    Vec3 cross(const Vec3& o) const {
        return { y*o.z - z*o.y,
                 z*o.x - x*o.z,
                 x*o.y - y*o.x };
    }

    Vec3 normalized() const {
        float m = magnitude();
        return (m > 0.0f) ? (*this / m) : Vec3{};
    }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }
