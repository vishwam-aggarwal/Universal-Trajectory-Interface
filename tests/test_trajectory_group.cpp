#include <cstdio>
#include <cmath>
#include "TrapezoidalProfile.h"
#include "TrajectoryGroup.h"

static int s_passed = 0, s_failed = 0;
static const float TOL = 1e-4f;

static void check(bool cond, const char* label) {
    if (cond) { printf("  PASS  %s\n", label); ++s_passed; }
    else       { printf("  FAIL  %s\n", label); ++s_failed; }
}
static void checkNear(float a, float b, float tol, const char* label) {
    check(fabsf(a - b) <= tol, label);
}

int main() {
    printf("=== TrajectoryGroup ===\n\n");

    // ------------------------------------------------------------------
    // 1. Two axes with different minimum durations
    //    ax0: 0->20, vMax=10, aMax=10  -> t3_min = 3.0  (trapezoidal)
    //    ax1: 0->10, vMax=10, aMax=10  -> t3_min = 2.0  (boundary triangular)
    //    Shared duration: 3.0  (ax1 is time-dilated)
    // ------------------------------------------------------------------
    {
        printf("-- 1. two-axis sync (ax0 leads) --\n");
        TrapezoidalProfile ax0, ax1;
        ITrajectoryProfile* profiles[] = { &ax0, &ax1 };
        float q0[] = { 0.0f,  0.0f };
        float qf[] = { 20.0f, 10.0f };
        TrajectoryLimits lim[] = { {10.0f, 10.0f}, {10.0f, 10.0f} };

        TrajectoryGroup group;
        check(group.plan(profiles, q0, qf, lim, 2), "plan returns true");
        checkNear(group.getDuration(), 3.0f, TOL, "shared duration == 3");
        check(group.getCount() == 2, "count == 2");

        float pos[2], vel[2], accel[2];

        // Both start at q0
        group.evaluate(0.0f, pos, vel, accel);
        checkNear(pos[0], 0.0f, TOL,  "t=0  ax0 pos == 0");
        checkNear(pos[1], 0.0f, TOL,  "t=0  ax1 pos == 0");

        // Both settle at qf at t=3
        bool moving = group.evaluate(3.0f, pos, vel, accel);
        check(!moving,                  "t=3  not moving");
        checkNear(pos[0], 20.0f, TOL,  "t=3  ax0 pos == 20");
        checkNear(pos[1], 10.0f, TOL,  "t=3  ax1 pos == 10");

        // By symmetry, both axes reach their midpoint at t = duration/2 = 1.5
        group.evaluate(1.5f, pos, vel, accel);
        checkNear(pos[0], 10.0f, TOL,  "t=1.5  ax0 pos == 10 (midpoint)");
        checkNear(pos[1],  5.0f, TOL,  "t=1.5  ax1 pos == 5  (midpoint)");
    }

    // ------------------------------------------------------------------
    // 2. Three axes, one with zero distance
    //    ax0: 0->10, vMax=10, aMax=10  -> t3_min = 2.0
    //    ax1: 0->0   (zero distance)   -> t3_min = 0
    //    ax2: 0->3,  vMax=10, aMax=10  -> t3_min ≈ 1.095
    //    Shared duration: 2.0
    // ------------------------------------------------------------------
    {
        printf("\n-- 2. three axes, one zero-distance --\n");
        TrapezoidalProfile ax0, ax1, ax2;
        ITrajectoryProfile* profiles[] = { &ax0, &ax1, &ax2 };
        float q0[] = { 0.0f, 5.0f, 0.0f };
        float qf[] = { 10.0f, 5.0f, 3.0f };
        TrajectoryLimits lim[] = { {10.0f,10.0f}, {10.0f,10.0f}, {10.0f,10.0f} };

        TrajectoryGroup group;
        check(group.plan(profiles, q0, qf, lim, 3), "plan returns true");
        checkNear(group.getDuration(), 2.0f, TOL, "shared duration == 2");

        float pos[3], vel[3], accel[3];

        group.evaluate(0.0f, pos, vel, accel);
        checkNear(pos[0], 0.0f, TOL,  "t=0  ax0 == 0");
        checkNear(pos[1], 5.0f, TOL,  "t=0  ax1 == 5 (zero-dist start)");
        checkNear(pos[2], 0.0f, TOL,  "t=0  ax2 == 0");

        bool moving = group.evaluate(2.0f, pos, vel, accel);
        check(!moving,                 "t=2  not moving");
        checkNear(pos[0], 10.0f, TOL, "t=2  ax0 == 10");
        checkNear(pos[1],  5.0f, TOL, "t=2  ax1 == 5 (unchanged)");
        checkNear(pos[2],  3.0f, TOL, "t=2  ax2 == 3");
    }

    // ------------------------------------------------------------------
    // 3. Single-axis group (degenerate — no sync needed)
    //    0->5, vMax=10, aMax=10 -> t3 = sqrt(0.5)*2 ≈ 1.414
    // ------------------------------------------------------------------
    {
        printf("\n-- 3. single-axis group --\n");
        TrapezoidalProfile ax0;
        ITrajectoryProfile* profiles[] = { &ax0 };
        float q0[] = { 0.0f };
        float qf[] = { 5.0f };
        TrajectoryLimits lim[] = { {10.0f, 10.0f} };

        TrajectoryGroup group;
        check(group.plan(profiles, q0, qf, lim, 1), "plan returns true");

        float t3 = 2.0f * sqrtf(5.0f / 10.0f);
        checkNear(group.getDuration(), t3, TOL, "duration == 2*sqrt(0.5)");

        float pos[1], vel[1], accel[1];
        group.evaluate(t3, pos, vel, accel);
        checkNear(pos[0], 5.0f, TOL, "t=t3  pos == 5");
    }

    // ------------------------------------------------------------------
    // 4. count out of range
    // ------------------------------------------------------------------
    {
        printf("\n-- 4. invalid count --\n");
        TrajectoryGroup group;
        // plan() checks count before touching the arrays, so nullptr is safe here
        check(!group.plan(nullptr, nullptr, nullptr, nullptr, 0),
              "count=0 returns false");
        check(!group.plan(nullptr, nullptr, nullptr, nullptr, 7),
              "count=7 returns false");
    }

    printf("\n%d passed, %d failed\n", s_passed, s_failed);
    return s_failed == 0 ? 0 : 1;
}
