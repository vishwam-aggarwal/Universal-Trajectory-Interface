#pragma once

struct TrajectoryLimits {
    float vMax;
    float aMax;
    float jMax = 0.0f;  // 0 = trapezoidal (ignored); >0 = jerk-limited (s-curve)

    // Explicit constructors rather than relying on aggregate initialization.
    // jMax's default member initializer above disqualifies this struct as
    // an aggregate under C++11 (aggregates gained default member
    // initializers only in C++14), so `TrajectoryLimits{vMax, aMax}`
    // brace-init would silently require C++14+ without these -- and this
    // library's hard constraint is C++11, checked on every compiler it
    // targets, not just the ones that happen to be lenient about it.
    TrajectoryLimits() = default;
    TrajectoryLimits(float vMax, float aMax, float jMax = 0.0f)
        : vMax(vMax), aMax(aMax), jMax(jMax) {}
};
