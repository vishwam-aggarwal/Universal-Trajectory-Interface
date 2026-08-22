#include <cstdio>
#include <cmath>
#include "LinePath.h"
#include "ArcPath.h"
#include "CartesianMove.h"
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
static void checkVec(const Vec3& v, float ex, float ey, float ez,
                     float tol, const char* label) {
    char buf[128];
    snprintf(buf, sizeof(buf), "%s x", label); checkNear(v.x, ex, tol, buf);
    snprintf(buf, sizeof(buf), "%s y", label); checkNear(v.y, ey, tol, buf);
    snprintf(buf, sizeof(buf), "%s z", label); checkNear(v.z, ez, tol, buf);
}

int main() {
    printf("=== Cartesian path classes ===\n\n");

    // ------------------------------------------------------------------
    // 1. LinePath geometry
    // ------------------------------------------------------------------
    {
        printf("-- 1. LinePath  p0=(0,0,0)  p1=(3,4,0) --\n");
        LinePath line({0,0,0}, {3,4,0});
        checkNear(line.getLength(), 5.0f, TOL, "length == 5");

        Vec3 pos, tan;
        line.evaluate(0.0f, pos, tan);
        checkVec(pos, 0.0f, 0.0f, 0.0f, TOL, "s=0   pos");
        checkVec(tan, 0.6f, 0.8f, 0.0f, TOL, "s=0   tangent");

        line.evaluate(2.5f, pos, tan);
        checkVec(pos, 1.5f, 2.0f, 0.0f, TOL, "s=2.5 pos (midpoint)");
        checkVec(tan, 0.6f, 0.8f, 0.0f, TOL, "s=2.5 tangent (constant)");

        line.evaluate(5.0f, pos, tan);
        checkVec(pos, 3.0f, 4.0f, 0.0f, TOL, "s=5   pos == p1");
    }

    // ------------------------------------------------------------------
    // 2. ArcPath geometry — quarter circle CCW in XY plane
    //    center=(0,0,0), start=(1,0,0), normal=(0,0,1), sweep=pi/2
    //    length = pi/2 * 1 = pi/2
    // ------------------------------------------------------------------
    {
        printf("\n-- 2. ArcPath  quarter circle CCW  r=1 --\n");
        const float PI  = 3.14159265f;
        const float S2  = 0.70710678f;   // sqrt(2)/2
        ArcPath arc({0,0,0}, {1,0,0}, {0,0,1}, PI / 2.0f);
        checkNear(arc.getLength(), PI / 2.0f, TOL, "length == pi/2");

        Vec3 pos, tan;

        // s=0: at start point, tangent pointing up (+y)
        arc.evaluate(0.0f, pos, tan);
        checkVec(pos, 1.0f, 0.0f, 0.0f, TOL, "s=0   pos == (1,0,0)");
        checkVec(tan, 0.0f, 1.0f, 0.0f, TOL, "s=0   tangent == (0,1,0)");

        // s=pi/4: 45-degree point
        arc.evaluate(PI / 4.0f, pos, tan);
        checkVec(pos, S2,  S2,  0.0f, TOL, "s=pi/4  pos == (sqrt2/2, sqrt2/2, 0)");
        checkVec(tan, -S2, S2,  0.0f, TOL, "s=pi/4  tangent");

        // s=pi/2: end point, tangent pointing left (-x)
        arc.evaluate(PI / 2.0f, pos, tan);
        checkVec(pos,  0.0f, 1.0f, 0.0f, TOL, "s=pi/2  pos == (0,1,0)");
        checkVec(tan, -1.0f, 0.0f, 0.0f, TOL, "s=pi/2  tangent == (-1,0,0)");
    }

    // ------------------------------------------------------------------
    // 3. ArcPath — quarter circle CW (negative sweep)
    // ------------------------------------------------------------------
    {
        printf("\n-- 3. ArcPath  quarter circle CW  r=1 --\n");
        const float PI = 3.14159265f;
        ArcPath arc({0,0,0}, {1,0,0}, {0,0,1}, -PI / 2.0f);
        checkNear(arc.getLength(), PI / 2.0f, TOL, "length == pi/2");

        Vec3 pos, tan;
        arc.evaluate(0.0f, pos, tan);
        checkVec(pos, 1.0f,  0.0f, 0.0f, TOL, "s=0   pos == (1,0,0)");
        checkVec(tan, 0.0f, -1.0f, 0.0f, TOL, "s=0   tangent == (0,-1,0) (CW)");

        arc.evaluate(PI / 2.0f, pos, tan);
        checkVec(pos,  0.0f, -1.0f, 0.0f, TOL, "s=pi/2  pos == (0,-1,0)");
        checkVec(tan, -1.0f,  0.0f, 0.0f, TOL, "s=pi/2  tangent == (-1,0,0)");
    }

    // ------------------------------------------------------------------
    // 4. CartesianMove over a line
    //    p0=(0,0,0) -> p1=(3,0,0), length=3, vMax=1, aMax=1
    //    s_v = 1, displacement=3 > s_v -> trapezoidal
    //    t1=1, t2=3, t3=4
    //    Identity orientation (no rotation)
    // ------------------------------------------------------------------
    {
        printf("\n-- 4. CartesianMove  line  (0,0,0)->(3,0,0)  vMax=1  aMax=1 --\n");
        LinePath line({0,0,0}, {3,0,0});
        TrapezoidalProfile profile;
        Quatf identity{1,0,0,0};

        CartesianMove move;
        TrajectoryLimits lim{1.0f, 1.0f};
        check(move.plan(&line, identity, identity, &profile, lim), "plan returns true");
        checkNear(move.getDuration(), 4.0f, TOL, "duration == 4");

        Vec3 pos, vel, accel; Quatf ori;

        // t=0: at start, stationary, accelerating
        move.evaluate(0.0f, pos, vel, accel, ori);
        checkVec(pos,   0.0f, 0.0f, 0.0f, TOL, "t=0   pos == p0");
        checkVec(vel,   0.0f, 0.0f, 0.0f, TOL, "t=0   vel == 0");
        checkVec(accel, 1.0f, 0.0f, 0.0f, TOL, "t=0   accel == (aMax,0,0)");

        // t=1: s=0.5, cruise starts — velocity = (1,0,0)
        move.evaluate(1.0f, pos, vel, accel, ori);
        checkVec(pos, 0.5f, 0.0f, 0.0f, TOL, "t=1   pos == (0.5,0,0)");
        checkVec(vel, 1.0f, 0.0f, 0.0f, TOL, "t=1   vel == (1,0,0)");

        // t=4: settled at p1
        bool moving = move.evaluate(4.0f, pos, vel, accel, ori);
        check(!moving,                        "t=4   not moving");
        checkVec(pos, 3.0f, 0.0f, 0.0f, TOL, "t=4   pos == p1");
        checkVec(vel, 0.0f, 0.0f, 0.0f, TOL, "t=4   vel == 0");
    }

    // ------------------------------------------------------------------
    // 5. CartesianMove over a quarter-circle arc
    //    center=(0,0,0), start=(1,0,0), normal=(0,0,1), sweep=pi/2
    //    length=pi/2, vMax=1, aMax=2
    //    s_v=0.5 < pi/2 -> trapezoidal
    // ------------------------------------------------------------------
    {
        printf("\n-- 5. CartesianMove  quarter-arc CCW  r=1  vMax=1  aMax=2 --\n");
        const float PI = 3.14159265f;
        ArcPath arc({0,0,0}, {1,0,0}, {0,0,1}, PI / 2.0f);
        TrapezoidalProfile profile;
        Quatf identity{1,0,0,0};

        CartesianMove move;
        TrajectoryLimits lim{1.0f, 2.0f};
        check(move.plan(&arc, identity, identity, &profile, lim), "plan returns true");

        Vec3 pos, vel, accel; Quatf ori;

        // At t=0: at start point (1,0,0), stationary
        move.evaluate(0.0f, pos, vel, accel, ori);
        checkVec(pos, 1.0f, 0.0f, 0.0f, TOL, "t=0   pos == start (1,0,0)");
        checkVec(vel, 0.0f, 0.0f, 0.0f, TOL, "t=0   vel == 0");

        // At t=duration: settled at end point (0,1,0)
        float dur = move.getDuration();
        bool moving = move.evaluate(dur, pos, vel, accel, ori);
        check(!moving,                        "t=dur  not moving");
        checkVec(pos, 0.0f, 1.0f, 0.0f, TOL, "t=dur  pos == end (0,1,0)");

        // Velocity direction at cruise should be along tangent (0,1,0) at start
        // t=t1=0.5 (end of accel, beginning of cruise): s=0.125, speed=1
        // tangent at s=0.125 (small arc): approximately (0,1,0)
        move.evaluate(0.5f, pos, vel, accel, ori);
        checkNear(vel.magnitude(), 1.0f, TOL, "t=t1  speed == vMax");
    }

    // ------------------------------------------------------------------
    // 6. Orientation SLERP through a CartesianMove
    //    Start: identity  End: 180-degree rotation about z  (w=0, x=0, y=0, z=1)
    //    At midpoint (s = length/2): should be 90-degree rotation about z
    // ------------------------------------------------------------------
    {
        printf("\n-- 6. orientation SLERP along move --\n");
        LinePath line({0,0,0}, {2,0,0});
        TrapezoidalProfile profile;

        Quatf q0{1.0f, 0.0f, 0.0f, 0.0f};   // identity
        Quatf q1{0.0f, 0.0f, 0.0f, 1.0f};   // 180 deg about z

        CartesianMove move;
        TrajectoryLimits lim{1.0f, 1.0f};
        move.plan(&line, q0, q1, &profile, lim);

        Vec3 pos, vel, accel; Quatf ori;

        // At t=0: orientation should be identity
        move.evaluate(0.0f, pos, vel, accel, ori);
        checkNear(ori.w, 1.0f, TOL, "t=0   ori.w == 1 (identity)");
        checkNear(ori.z, 0.0f, TOL, "t=0   ori.z == 0");

        // At midpoint of arc length: 90-deg rotation about z = (w=sqrt2/2, z=sqrt2/2)
        // Midpoint of move happens at t = duration/2 (by symmetry)
        float dur = move.getDuration();
        move.evaluate(dur / 2.0f, pos, vel, accel, ori);
        const float S2 = 0.70710678f;
        checkNear(ori.w, S2, TOL, "t=mid  ori.w == sqrt2/2 (90 deg)");
        checkNear(ori.z, S2, TOL, "t=mid  ori.z == sqrt2/2 (90 deg)");
    }

    // ------------------------------------------------------------------
    // 7. Degenerate ArcPath (startPoint == center -> radius 0) must not
    //    divide by zero into NaN position/tangent.
    // ------------------------------------------------------------------
    {
        printf("\n-- 7. ArcPath degenerate (radius == 0) --\n");
        const float PI = 3.14159265f;
        ArcPath arc({1,2,3}, {1,2,3}, {0,0,1}, PI / 2.0f);
        checkNear(arc.getLength(), 0.0f, TOL, "length == 0");

        Vec3 pos, tan;
        arc.evaluate(0.0f, pos, tan);
        check(!std::isnan(pos.x) && !std::isnan(pos.y) && !std::isnan(pos.z),
              "s=0  position has no NaN component");
        check(!std::isnan(tan.x) && !std::isnan(tan.y) && !std::isnan(tan.z),
              "s=0  tangent has no NaN component");
        checkVec(pos, 1.0f, 2.0f, 3.0f, TOL, "s=0  position == center");
    }

    printf("\n%d passed, %d failed\n", s_passed, s_failed);
    return s_failed == 0 ? 0 : 1;
}
