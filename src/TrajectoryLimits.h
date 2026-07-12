#pragma once

struct TrajectoryLimits {
    float vMax;
    float aMax;
    float jMax = 0.0f;  // 0 = trapezoidal (ignored); >0 = jerk-limited (s-curve)
};
