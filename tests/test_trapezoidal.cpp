#include <cstdio>
#include <cmath>
#include "TrapezoidalProfile.h"

static int s_passed = 0, s_failed = 0;
static const float TOL = 1e-4f;

static void check(bool cond, const char* label) {
    if (cond) { printf("  PASS  %s\n", label); ++s_passed; }
    else       { printf("  FAIL  %s\n", label); ++s_failed; }
}

static void checkNear(float a, float b, float tol, const char* label) {
    check(fabsf(a - b) <= tol, label);
}

static void evalCheck(TrapezoidalProfile& p, float t,
                      float ep, float ev, float ea, bool em,
                      const char* label) {
    float pos, vel, accel;
    bool moving = p.evaluate(t, pos, vel, accel);
    char buf[128];
    snprintf(buf, sizeof(buf), "%s  pos",    label); checkNear(pos,   ep, TOL, buf);
    snprintf(buf, sizeof(buf), "%s  vel",    label); checkNear(vel,   ev, TOL, buf);
    snprintf(buf, sizeof(buf), "%s  accel",  label); checkNear(accel, ea, TOL, buf);
    snprintf(buf, sizeof(buf), "%s  moving", label); check(moving == em,       buf);
}

int main() {
    printf("=== TrapezoidalProfile ===\n\n");

    // ------------------------------------------------------------------
    // 1. Zero-distance move
    // ------------------------------------------------------------------
    {
        printf("-- 1. zero-distance move --\n");
        TrapezoidalProfile p;
        TrajectoryLimits lim{ 10.0f, 10.0f };
        check(p.plan(5.0f, 5.0f, lim, 0.0f),   "plan returns true");
        checkNear(p.getDuration(), 0.0f, TOL, "duration == 0");
        evalCheck(p, 0.0f, 5.0f, 0.0f, 0.0f, false, "t=0");
        evalCheck(p, 1.0f, 5.0f, 0.0f, 0.0f, false, "t=1 (settled)");
    }

    // ------------------------------------------------------------------
    // 2. Positive trapezoidal: q0=0, qf=20, vMax=10, aMax=10
    //    s_v=10 < 20 → trapezoidal, t1=1, t2=2, t3=3
    // ------------------------------------------------------------------
    {
        printf("\n-- 2. trapezoidal  q0=0  qf=20  vMax=10  aMax=10 --\n");
        TrapezoidalProfile p;
        TrajectoryLimits lim{ 10.0f, 10.0f };
        check(p.plan(0.0f, 20.0f, lim, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 3.0f, TOL, "duration == 3");

        evalCheck(p, 0.0f,   0.0f,   0.0f,  10.0f, true,  "t=0.0  accel start");
        evalCheck(p, 0.5f,   1.25f,  5.0f,  10.0f, true,  "t=0.5  accel mid");
        evalCheck(p, 1.0f,   5.0f,  10.0f,   0.0f, true,  "t=1.0  cruise start");
        evalCheck(p, 1.5f,  10.0f,  10.0f,   0.0f, true,  "t=1.5  cruise mid");
        evalCheck(p, 2.0f,  15.0f,  10.0f, -10.0f, true,  "t=2.0  decel start");
        evalCheck(p, 2.5f,  18.75f,  5.0f, -10.0f, true,  "t=2.5  decel mid");
        evalCheck(p, 3.0f,  20.0f,   0.0f,   0.0f, false, "t=3.0  settled");
        evalCheck(p, 4.0f,  20.0f,   0.0f,   0.0f, false, "t=4.0  past end");
    }

    // ------------------------------------------------------------------
    // 3. Boundary triangular: q0=0, qf=10, vMax=10, aMax=10
    //    displacement == s_v → t1=1, t2=1, t3=2  (MATLAB script case)
    // ------------------------------------------------------------------
    {
        printf("\n-- 3. boundary triangular  q0=0  qf=10  vMax=10  aMax=10 --\n");
        TrapezoidalProfile p;
        TrajectoryLimits lim{ 10.0f, 10.0f };
        check(p.plan(0.0f, 10.0f, lim, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 2.0f, TOL, "duration == 2");

        evalCheck(p, 0.5f,  1.25f,  5.0f,  10.0f, true,  "t=0.5  accel");
        evalCheck(p, 1.0f,  5.0f,  10.0f, -10.0f, true,  "t=1.0  decel start (no cruise)");
        evalCheck(p, 1.5f,  8.75f,  5.0f, -10.0f, true,  "t=1.5  decel mid");
        evalCheck(p, 2.0f, 10.0f,   0.0f,   0.0f, false, "t=2.0  settled");
    }

    // ------------------------------------------------------------------
    // 4. Short triangular: q0=0, qf=3, vMax=10, aMax=10
    //    displacement < s_v → t1=sqrt(0.3), t2=t1, t3=2*t1
    // ------------------------------------------------------------------
    {
        printf("\n-- 4. short triangular  q0=0  qf=3  vMax=10  aMax=10 --\n");
        TrapezoidalProfile p;
        TrajectoryLimits lim{ 10.0f, 10.0f };
        check(p.plan(0.0f, 3.0f, lim, 0.0f), "plan returns true");

        float t1 = sqrtf(3.0f / 10.0f);
        checkNear(p.getDuration(), 2.0f * t1, TOL, "duration == 2*sqrt(0.3)");

        float pos, vel, accel;
        // At peak (t=t1): halfway in position, max velocity, switching to decel
        p.evaluate(t1, pos, vel, accel);
        checkNear(pos,   1.5f,        TOL, "t=t1  pos == 1.5 (midpoint)");
        checkNear(vel,   sqrtf(30.0f), TOL, "t=t1  vel == sqrt(30)");
        checkNear(accel, -10.0f,       TOL, "t=t1  accel == -aMax");
        // At end: settled at qf
        p.evaluate(2.0f * t1, pos, vel, accel);
        checkNear(pos,  3.0f, TOL, "t=t3  pos == 3");
        checkNear(vel,  0.0f, TOL, "t=t3  vel == 0");
    }

    // ------------------------------------------------------------------
    // 5. Negative direction: q0=10, qf=0, vMax=10, aMax=10
    //    t1=1, t2=1, t3=2, dir=-1
    // ------------------------------------------------------------------
    {
        printf("\n-- 5. negative direction  q0=10  qf=0  vMax=10  aMax=10 --\n");
        TrapezoidalProfile p;
        TrajectoryLimits lim{ 10.0f, 10.0f };
        check(p.plan(10.0f, 0.0f, lim, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 2.0f, TOL, "duration == 2");

        evalCheck(p, 0.5f,  8.75f, -5.0f, -10.0f, true,  "t=0.5  accel (neg)");
        evalCheck(p, 1.0f,  5.0f, -10.0f,  10.0f, true,  "t=1.0  decel start");
        evalCheck(p, 1.5f,  1.25f, -5.0f,  10.0f, true,  "t=1.5  decel mid");
        evalCheck(p, 2.0f,  0.0f,   0.0f,   0.0f, false, "t=2.0  settled");
    }

    // ------------------------------------------------------------------
    // 6. t < 0 clamping
    // ------------------------------------------------------------------
    {
        printf("\n-- 6. t < 0 clamping --\n");
        TrapezoidalProfile p;
        TrajectoryLimits lim{ 10.0f, 10.0f };
        p.plan(0.0f, 20.0f, lim, 0.0f);
        evalCheck(p, -1.0f, 0.0f, 0.0f, 10.0f, true, "t=-1 clamped to t=0");
    }

    // ------------------------------------------------------------------
    // 7. Time-dilation: stretch q0=0, qf=3 (t3_min≈1.095) to t=3.0
    //    Expected: lower vPeak, cruise phase present, settles at 3.0
    // ------------------------------------------------------------------
    {
        printf("\n-- 7. time-dilation  q0=0  qf=3  vMax=10  aMax=10  targetDuration=3 --\n");
        TrapezoidalProfile p;
        TrajectoryLimits lim{ 10.0f, 10.0f };
        check(p.plan(0.0f, 3.0f, lim, 3.0f), "plan returns true");
        checkNear(p.getDuration(), 3.0f, TOL, "duration == targetDuration");

        float pos, vel, accel;
        p.evaluate(3.0f, pos, vel, accel);
        checkNear(pos, 3.0f, TOL, "t=3.0  pos == qf");
        checkNear(vel, 0.0f, TOL, "t=3.0  vel == 0");

        // Midpoint should be at pos=1.5 (symmetric profile)
        p.evaluate(1.5f, pos, vel, accel);
        checkNear(pos, 1.5f, TOL, "t=1.5  pos == 1.5 (midpoint)");
    }

    printf("\n%d passed, %d failed\n", s_passed, s_failed);
    return s_failed == 0 ? 0 : 1;
}
