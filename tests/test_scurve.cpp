// Desktop tests for SCurveProfile.
//
// Every expected pos/vel/accel below was generated from a double-precision
// transcription of Resources/MATLAB/S_Curve_Gen/ (the same equations, run
// in double), not derived by hand -- so this suite is a numeric diff
// against the MATLAB reference, which is exactly the "Testing goal" in
// CLAUDE.md. Each block names the branch of
// SCurve_calculateTimeSegments.m it exercises.

#include <cstdio>
#include <cmath>
#include "SCurveProfile.h"
#include "TrapezoidalProfile.h"
#include "TrajectoryGroup.h"

static int s_passed = 0, s_failed = 0;
static const float TOL = 1e-4f;

static void check(bool cond, const char* label) {
    if (cond) { printf("  PASS  %s\n", label); ++s_passed; }
    else       { printf("  FAIL  %s\n", label); ++s_failed; }
}

static void checkNear(float a, float b, float tol, const char* label) {
    if (fabsf(a - b) <= tol) { printf("  PASS  %s\n", label); ++s_passed; }
    else {
        printf("  FAIL  %s  (got %.9f, want %.9f, diff %.3e)\n",
               label, a, b, fabsf(a - b));
        ++s_failed;
    }
}

// Note: the settled-instant rows below pass p.getDuration() rather than a
// decimal literal. t7 is a sum of float square/cube roots, and a decimal that
// prints as the same value can round-trip to one ulp below it -- leaving
// evaluate() correctly still inside its final phase. Each block asserts
// getDuration() against the expected literal separately, so nothing is lost.
static void evalCheck(const SCurveProfile& p, float t,
                      float ep, float ev, float ea, bool em,
                      const char* label) {
    float pos, vel, accel;
    bool moving = p.evaluate(t, pos, vel, accel);
    char buf[160];
    snprintf(buf, sizeof(buf), "%s  pos",    label); checkNear(pos,   ep, TOL, buf);
    snprintf(buf, sizeof(buf), "%s  vel",    label); checkNear(vel,   ev, TOL, buf);
    snprintf(buf, sizeof(buf), "%s  accel",  label); checkNear(accel, ea, TOL, buf);
    snprintf(buf, sizeof(buf), "%s  moving", label); check(moving == em,       buf);
}

// Sweeps a planned profile and verifies the properties that make it an
// S-curve at all: the three limits are respected, and acceleration is
// continuous (bounded rate of change == the jerk limit), which is the whole
// point of the profile and the thing a trapezoid does NOT satisfy.
static void sweepCheck(const SCurveProfile& p, float q0, float qf,
                       const TrajectoryLimits& lim, const char* label) {
    const float dur = p.getDuration();
    const int   N   = 4000;
    const float dt  = dur / N;

    float maxV = 0.0f, maxA = 0.0f, maxJerk = 0.0f;
    float maxPosJump = 0.0f;
    float prevPos = q0, prevVel = 0.0f, prevAcc = 0.0f;
    char buf[160];

    for (int i = 0; i <= N; ++i) {
        const float t = dt * i;
        float pos, vel, accel;
        p.evaluate(t, pos, vel, accel);

        if (fabsf(vel)   > maxV) maxV = fabsf(vel);
        if (fabsf(accel) > maxA) maxA = fabsf(accel);

        if (i > 0) {
            const float dAcc = fabsf(accel - prevAcc) / dt;
            if (dAcc > maxJerk) maxJerk = dAcc;
            const float posJump = fabsf(pos - prevPos);
            if (posJump > maxPosJump) maxPosJump = posJump;
        }
        prevPos = pos; prevVel = vel; prevAcc = accel;
    }

    // Small multiplicative slack: the sweep lands on segment boundaries only
    // by luck, so a finite-difference jerk estimate straddling a corner
    // reads slightly high.
    snprintf(buf, sizeof(buf), "%s  peak |vel| <= vMax",   label);
    check(maxV <= lim.vMax * 1.001f + TOL, buf);
    snprintf(buf, sizeof(buf), "%s  peak |accel| <= aMax", label);
    check(maxA <= lim.aMax * 1.001f + TOL, buf);
    snprintf(buf, sizeof(buf), "%s  |jerk| <= jMax (accel is continuous)", label);
    check(maxJerk <= lim.jMax * 1.05f + 1e-3f, buf);

    // Position advances smoothly -- no step discontinuity at any boundary.
    const float nominalStep = fabsf(qf - q0) / N;
    snprintf(buf, sizeof(buf), "%s  position continuous across all 7 boundaries", label);
    check(maxPosJump <= nominalStep * 4.0f + TOL, buf);

    float pos, vel, accel;
    p.evaluate(dur, pos, vel, accel);
    snprintf(buf, sizeof(buf), "%s  lands exactly on qf", label);
    checkNear(pos, qf, TOL, buf);
}

int main() {
    printf("=== SCurveProfile ===\n\n");

    // ------------------------------------------------------------------
    // 1. Zero-distance move
    // ------------------------------------------------------------------
    {
        printf("-- 1. zero-distance move --\n");
        SCurveProfile p;
        TrajectoryLimits lim{ 10.0f, 10.0f, 10.0f };
        check(p.plan(5.0f, 5.0f, lim, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 0.0f, TOL, "duration == 0");
        evalCheck(p, 0.0f, 5.0f, 0.0f, 0.0f, false, "t=0");
        evalCheck(p, 1.0f, 5.0f, 0.0f, 0.0f, false, "t=1 (settled)");
    }

    // ------------------------------------------------------------------
    // 2. Invalid limits -- profile left inert and parked at q0
    // ------------------------------------------------------------------
    {
        printf("\n-- 2. invalid limits --\n");
        const float nan = sqrtf(-1.0f);

        struct Case { TrajectoryLimits lim; const char* what; };
        Case cases[] = {
            { TrajectoryLimits( 0.0f, 10.0f, 10.0f), "vMax == 0"  },
            { TrajectoryLimits(-1.0f, 10.0f, 10.0f), "vMax < 0"   },
            { TrajectoryLimits(10.0f,  0.0f, 10.0f), "aMax == 0"  },
            { TrajectoryLimits(10.0f, -1.0f, 10.0f), "aMax < 0"   },
            { TrajectoryLimits(10.0f, 10.0f,  0.0f), "jMax == 0 (trapezoidal marker)" },
            { TrajectoryLimits(10.0f, 10.0f, -1.0f), "jMax < 0"   },
            { TrajectoryLimits(  nan, 10.0f, 10.0f), "vMax NaN"   },
            { TrajectoryLimits(10.0f,   nan, 10.0f), "aMax NaN"   },
            { TrajectoryLimits(10.0f, 10.0f,   nan), "jMax NaN"   },
        };

        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            SCurveProfile p;
            char buf[160];
            snprintf(buf, sizeof(buf), "%s: plan returns false", cases[i].what);
            check(!p.plan(2.0f, 9.0f, cases[i].lim, 0.0f), buf);
            snprintf(buf, sizeof(buf), "%s: duration == 0", cases[i].what);
            checkNear(p.getDuration(), 0.0f, TOL, buf);

            float pos, vel, accel;
            const bool moving = p.evaluate(0.5f, pos, vel, accel);
            snprintf(buf, sizeof(buf), "%s: parked at q0, not moving", cases[i].what);
            check(!moving && fabsf(pos - 2.0f) <= TOL
                          && fabsf(vel) <= TOL && fabsf(accel) <= TOL, buf);
        }
    }

    // ------------------------------------------------------------------
    // 3. Case D.1 -- full 7-segment profile with a cruise phase.
    //    q0=0 qf=20 J=10 A=10 V=10 -> t1..t7 = 1,1,2,2,3,3,4
    //    (t2==t1 and t6==t5: no constant-acceleration phase is needed here,
    //     which the MATLAB expresses as zero-length segments, not a
    //     separate branch.)
    // ------------------------------------------------------------------
    {
        printf("\n-- 3. case D.1  q0=0 qf=20 J=10 A=10 V=10 --\n");
        SCurveProfile p;
        TrajectoryLimits lim{ 10.0f, 10.0f, 10.0f };
        check(p.plan(0.0f, 20.0f, lim, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 4.0f, TOL, "duration == 4");

        evalCheck(p, 0.0f,  0.000000000f,  0.00f,   0.0f, true,  "t=0.0  start (jerk ramp)");
        evalCheck(p, 0.5f,  0.208333333f,  1.25f,   5.0f, true,  "t=0.5  accel ramping up");
        evalCheck(p, 1.0f,  1.666666667f,  5.00f,  10.0f, true,  "t=1.0  peak accel");
        evalCheck(p, 1.5f,  5.208333333f,  8.75f,   5.0f, true,  "t=1.5  accel ramping down");
        evalCheck(p, 2.0f, 10.000000000f, 10.00f,   0.0f, true,  "t=2.0  cruise / midpoint");
        evalCheck(p, 2.5f, 14.791666667f,  8.75f,  -5.0f, true,  "t=2.5  decel ramping up");
        evalCheck(p, 3.0f, 18.333333333f,  5.00f, -10.0f, true,  "t=3.0  peak decel");
        evalCheck(p, 3.5f, 19.791666667f,  1.25f,  -5.0f, true,  "t=3.5  decel ramping down");
        evalCheck(p, p.getDuration(), 20.000000000f, 0.00f, 0.0f, false, "t=t7   settled");
        evalCheck(p, 9.0f, 20.000000000f,  0.00f,   0.0f, false, "t=9.0  past end");
        evalCheck(p, -1.0f, 0.000000000f,  0.00f,   0.0f, true,  "t=-1.0 clamped to 0");

        sweepCheck(p, 0.0f, 20.0f, lim, "D.1");
    }

    // ------------------------------------------------------------------
    // 4. Case B, negative direction -- the SCurve_Script.m case verbatim:
    //    currentPos=10, setPoint_Pos=1, jerk=acc=vel=10
    // ------------------------------------------------------------------
    {
        printf("\n-- 4. case B  SCurve_Script.m  q0=10 qf=1 J=A=V=10 --\n");
        SCurveProfile p;
        TrajectoryLimits lim{ 10.0f, 10.0f, 10.0f };
        check(p.plan(10.0f, 1.0f, lim, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 3.065237730f, TOL, "duration == 3.06523773");

        evalCheck(p, 0.000000000f, 10.000000f,  0.000000000f,  0.000000000f, true,
                  "t=0        start");
        evalCheck(p, 0.383154716f,  9.906250f, -0.734037683f, -3.831547162f, true,
                  "t=0.38315  accel ramp (negative)");
        evalCheck(p, 0.766309432f,  9.250000f, -2.936150731f, -7.663094324f, true,
                  "t=0.76631  t1: peak decel");
        evalCheck(p, 1.149464149f,  7.656250f, -5.138263779f, -3.831547162f, true,
                  "t=1.14946  accel ramping back");
        evalCheck(p, 1.532618865f,  5.500000f, -5.872301462f,  0.000000000f, true,
                  "t=1.53262  t3/t4: peak speed, midpoint");
        evalCheck(p, 1.915773581f,  3.343750f, -5.138263779f,  3.831547162f, true,
                  "t=1.91577  decel ramping up");
        evalCheck(p, 2.298928297f,  1.750000f, -2.936150731f,  7.663094324f, true,
                  "t=2.29893  t5/t6: peak decel");
        evalCheck(p, 2.682083013f,  1.093750f, -0.734037683f,  3.831547162f, true,
                  "t=2.68208  decel ramping down");
        evalCheck(p, p.getDuration(), 1.000000f,  0.000000000f,  0.000000000f, false,
                  "t=t7       settled at qf");

        check(p.getDuration() > 0.0f, "negative move has positive duration");
        sweepCheck(p, 10.0f, 1.0f, lim, "B(neg)");
    }

    // ------------------------------------------------------------------
    // 5. Case C.1 -- vMax below the accel-ramp velocity, long cruise.
    //    q0=0 qf=5 J=10 A=10 V=1
    // ------------------------------------------------------------------
    {
        printf("\n-- 5. case C.1  q0=0 qf=5 J=10 A=10 V=1 --\n");
        SCurveProfile p;
        TrajectoryLimits lim{ 1.0f, 10.0f, 10.0f };
        check(p.plan(0.0f, 5.0f, lim, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 5.632455532f, TOL, "duration == 5.63245553");

        evalCheck(p, 0.316227766f, 0.052704628f, 0.5f,  3.162277660f, true,
                  "t=0.31623  t1: peak accel");
        evalCheck(p, 0.632455532f, 0.316227766f, 1.0f,  0.000000000f, true,
                  "t=0.63246  t3: cruise begins at vMax");
        evalCheck(p, 2.112170825f, 1.795943058f, 1.0f,  0.000000000f, true,
                  "t=2.11217  cruising");
        evalCheck(p, 5.000000000f, 4.683772234f, 1.0f,  0.000000000f, true,
                  "t=5.00000  t4: cruise ends");
        evalCheck(p, 5.316227766f, 4.947295372f, 0.5f, -3.162277660f, true,
                  "t=5.31623  t5/t6: peak decel");
        evalCheck(p, p.getDuration(), 5.000000000f, 0.0f,  0.000000000f, false,
                  "t=t7       settled");

        sweepCheck(p, 0.0f, 5.0f, lim, "C.1");
    }

    // ------------------------------------------------------------------
    // 6. Case C.2 -- move too short to reach vMax at all.
    //    q0=0 qf=0.05 J=10 A=10 V=1
    // ------------------------------------------------------------------
    {
        printf("\n-- 6. case C.2  q0=0 qf=0.05 J=10 A=10 V=1 --\n");
        SCurveProfile p;
        TrajectoryLimits lim{ 1.0f, 10.0f, 10.0f };
        check(p.plan(0.0f, 0.05f, lim, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 0.542883523f, TOL, "duration == 0.54288352");

        evalCheck(p, 0.067860440f, 0.000520833f, 0.023025197f,  0.678604404f, true,
                  "t=0.06786  accel ramp");
        evalCheck(p, 0.135720881f, 0.004166667f, 0.092100787f,  1.357208808f, true,
                  "t=0.13572  t1: peak accel (well under aMax)");
        evalCheck(p, 0.271441762f, 0.025000000f, 0.184201575f,  0.000000000f, true,
                  "t=0.27144  midpoint, peak speed (under vMax)");
        evalCheck(p, 0.407162642f, 0.045833333f, 0.092100787f, -1.357208808f, true,
                  "t=0.40716  t5: peak decel");
        evalCheck(p, p.getDuration(), 0.050000000f, 0.000000000f,  0.000000000f, false,
                  "t=t7       settled");

        // The defining property of this branch: neither vMax nor aMax is
        // reached -- the move is jerk-bound end to end.
        float pos, vel, accel, maxV = 0.0f, maxA = 0.0f;
        for (int i = 0; i <= 2000; ++i) {
            p.evaluate(p.getDuration() * i / 2000.0f, pos, vel, accel);
            if (fabsf(vel)   > maxV) maxV = fabsf(vel);
            if (fabsf(accel) > maxA) maxA = fabsf(accel);
        }
        check(maxV < lim.vMax, "short move never reaches vMax");
        check(maxA < lim.aMax, "short move never reaches aMax");

        sweepCheck(p, 0.0f, 0.05f, lim, "C.2");
    }

    // ------------------------------------------------------------------
    // 7. Case D.2 -- constant-acceleration phase present, no cruise.
    //    q0=0 qf=0.5 J=1000 A=50 V=20
    // ------------------------------------------------------------------
    {
        printf("\n-- 7. case D.2  q0=0 qf=0.5 J=1000 A=50 V=20 --\n");
        SCurveProfile p;
        TrajectoryLimits lim{ 20.0f, 50.0f, 1000.0f };
        check(p.plan(0.0f, 0.5f, lim, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 0.256155281f, TOL, "duration == 0.25615528");

        evalCheck(p, 0.032019410f, 0.005471277f, 0.512621314f,  32.019410160f, true,
                  "t=0.03202  accel ramping up");
        evalCheck(p, 0.050000000f, 0.020833333f, 1.250000000f,  50.000000000f, true,
                  "t=0.05000  t1: aMax reached");
        evalCheck(p, 0.078077641f, 0.075639232f, 2.653882032f,  50.000000000f, true,
                  "t=0.07808  t2: end of constant-accel phase");
        evalCheck(p, 0.128077641f, 0.250000000f, 3.903882032f,   0.000000000f, true,
                  "t=0.12808  t3/t4: peak speed (under vMax), midpoint");
        evalCheck(p, 0.178077641f, 0.424360768f, 2.653882032f, -50.000000000f, true,
                  "t=0.17808  t5: peak decel");
        evalCheck(p, 0.206155281f, 0.479166667f, 1.250000000f, -50.000000000f, true,
                  "t=0.20616  t6: end of constant-decel phase");
        evalCheck(p, p.getDuration(), 0.500000000f, 0.000000000f,   0.000000000f, false,
                  "t=t7       settled");

        sweepCheck(p, 0.0f, 0.5f, lim, "D.2");
    }

    // ------------------------------------------------------------------
    // 8. Case A -- vMax < v_a with a long enough move to cruise.
    //    q0=0 qf=7 J=5 A=4 V=3
    // ------------------------------------------------------------------
    {
        printf("\n-- 8. case A  q0=0 qf=7 J=5 A=4 V=3 --\n");
        SCurveProfile p;
        TrajectoryLimits lim{ 3.0f, 4.0f, 5.0f };
        check(p.plan(0.0f, 7.0f, lim, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 3.882526672f, TOL, "duration == 3.88252667");

        evalCheck(p, 0.485315834f, 0.095255955f, 0.588828647f,  2.426579170f, true,
                  "t=0.48532  accel ramping up");
        evalCheck(p, 0.774596669f, 0.387298335f, 1.500000000f,  3.872983346f, true,
                  "t=0.77460  t1: peak accel");
        evalCheck(p, 1.549193338f, 2.323790008f, 3.000000000f,  0.000000000f, true,
                  "t=1.54919  t3: cruise begins at vMax");
        evalCheck(p, 2.333333333f, 4.676209992f, 3.000000000f,  0.000000000f, true,
                  "t=2.33333  t4: cruise ends");
        evalCheck(p, 3.107930003f, 6.612701665f, 1.500000000f, -3.872983346f, true,
                  "t=3.10793  t5/t6: peak decel");
        evalCheck(p, p.getDuration(), 7.000000000f, 0.000000000f,  0.000000000f, false,
                  "t=t7       settled");

        sweepCheck(p, 0.0f, 7.0f, lim, "A");
    }

    // ------------------------------------------------------------------
    // 9. Time dilation (targetDuration) -- not in the MATLAB, required by
    //    ITrajectoryProfile/TrajectoryGroup. Implemented by re-running the
    //    same segment math on limits scaled (vMax/k, aMax/k^2, jMax/k^3),
    //    which stretches the profile by exactly k.
    // ------------------------------------------------------------------
    {
        printf("\n-- 9. time dilation --\n");
        TrajectoryLimits lim{ 10.0f, 10.0f, 10.0f };
        const float minDuration = 4.0f;   // from case D.1 above

        // 9a. Dilating 4s -> 8s reproduces the same shape at half speed.
        {
            SCurveProfile p;
            check(p.plan(0.0f, 20.0f, lim, 8.0f), "T=8: plan reports request satisfied");
            checkNear(p.getDuration(), 8.0f, TOL, "T=8: duration == requested");

            evalCheck(p, 1.0f,  0.208333333f, 0.625f,  1.25f, true, "T=8  t=1.0");
            evalCheck(p, 2.0f,  1.666666667f, 2.500f,  2.50f, true, "T=8  t=2.0");
            evalCheck(p, 4.0f, 10.000000000f, 5.000f,  0.00f, true, "T=8  t=4.0 midpoint");
            evalCheck(p, 6.0f, 18.333333333f, 2.500f, -2.50f, true, "T=8  t=6.0");
            evalCheck(p, p.getDuration(), 20.000000000f, 0.000f, 0.00f, false, "T=8  t=t7 settled");

            sweepCheck(p, 0.0f, 20.0f, lim, "T=8");
        }

        // 9b. A shorter-than-possible request is refused, not silently
        //     honoured by violating the limits.
        {
            SCurveProfile p;
            check(!p.plan(0.0f, 20.0f, lim, 1.0f), "T=1 (< min): plan reports request unmet");
            checkNear(p.getDuration(), minDuration, TOL,
                      "T=1: falls back to minimum-time duration");
            sweepCheck(p, 0.0f, 20.0f, lim, "T=1 clamped");
        }

        // 9c. Requesting exactly the minimum is satisfied exactly.
        {
            SCurveProfile p;
            check(p.plan(0.0f, 20.0f, lim, minDuration),
                  "T==minDuration: plan reports request satisfied");
            checkNear(p.getDuration(), minDuration, TOL, "T==minDuration: duration matches");
        }

        // 9d. Dilation on a zero-distance move stays a no-op.
        {
            SCurveProfile p;
            check(p.plan(3.0f, 3.0f, lim, 5.0f), "zero-distance with T=5: plan returns true");
            checkNear(p.getDuration(), 0.0f, TOL, "zero-distance with T=5: duration == 0");
        }

        // 9e. Dilation never exceeds the original limits, for every branch.
        {
            struct Case { float q0, qf, v, a, j; const char* what; };
            Case cases[] = {
                { 0.0f, 20.0f,  10.0f, 10.0f,   10.0f, "D.1" },
                { 10.0f, 1.0f,  10.0f, 10.0f,   10.0f, "B"   },
                { 0.0f,  5.0f,   1.0f, 10.0f,   10.0f, "C.1" },
                { 0.0f, 0.05f,   1.0f, 10.0f,   10.0f, "C.2" },
                { 0.0f,  0.5f,  20.0f, 50.0f, 1000.0f, "D.2" },
                { 0.0f,  7.0f,   3.0f,  4.0f,    5.0f, "A"   },
            };
            for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
                const Case& c = cases[i];
                TrajectoryLimits cl{ c.v, c.a, c.j };
                SCurveProfile base;
                base.plan(c.q0, c.qf, cl, 0.0f);
                const float target = base.getDuration() * 2.5f;

                SCurveProfile p;
                char buf[160];
                snprintf(buf, sizeof(buf), "dilate %s x2.5: request satisfied", c.what);
                check(p.plan(c.q0, c.qf, cl, target), buf);
                snprintf(buf, sizeof(buf), "dilate %s x2.5: duration exact", c.what);
                checkNear(p.getDuration(), target, TOL * fabsf(target) + TOL, buf);
                snprintf(buf, sizeof(buf), "dilate %s x2.5", c.what);
                sweepCheck(p, c.q0, c.qf, cl, buf);
            }
        }
    }

    // ------------------------------------------------------------------
    // 10. Polymorphic use -- the 3-arg convenience overload must work both
    //     through a base reference and directly on the concrete type (the
    //     latter is what `using ITrajectoryProfile::plan;` buys).
    // ------------------------------------------------------------------
    {
        printf("\n-- 10. ITrajectoryProfile polymorphism --\n");
        TrajectoryLimits lim{ 10.0f, 10.0f, 10.0f };

        SCurveProfile concrete;
        check(concrete.plan(0.0f, 20.0f, lim), "3-arg plan() on concrete type compiles and succeeds");
        checkNear(concrete.getDuration(), 4.0f, TOL, "concrete 3-arg: duration == 4");

        SCurveProfile owned;
        ITrajectoryProfile& base = owned;
        check(base.plan(0.0f, 20.0f, lim), "3-arg plan() through base reference succeeds");
        checkNear(base.getDuration(), 4.0f, TOL, "base 3-arg: duration == 4");

        float pos, vel, accel;
        check(base.evaluate(2.0f, pos, vel, accel), "base evaluate() dispatches virtually");
        checkNear(pos, 10.0f, TOL, "base evaluate(): pos at midpoint == 10");
    }

    // ------------------------------------------------------------------
    // 11. Composition with TrajectoryGroup -- proves SCurveProfile drops in
    //     with no change to TrajectoryGroup, including mixed with a
    //     TrapezoidalProfile on another axis.
    // ------------------------------------------------------------------
    {
        printf("\n-- 11. TrajectoryGroup composition --\n");
        SCurveProfile      axis0;
        SCurveProfile      axis1;
        TrapezoidalProfile axis2;
        ITrajectoryProfile* profiles[3] = { &axis0, &axis1, &axis2 };

        const float q0[3] = { 0.0f,  0.0f, 0.0f };
        const float qf[3] = { 20.0f, 2.0f, 5.0f };
        TrajectoryLimits lims[3] = {
            TrajectoryLimits(10.0f, 10.0f, 10.0f),
            TrajectoryLimits(10.0f, 10.0f, 10.0f),
            TrajectoryLimits(10.0f, 10.0f),
        };

        TrajectoryGroup group;
        check(group.plan(profiles, q0, qf, lims, 3), "group plan succeeds with S-curve axes");

        const float dur = group.getDuration();
        checkNear(dur, 4.0f, TOL, "group duration == slowest axis (S-curve axis0, 4s)");
        checkNear(axis0.getDuration(), dur, TOL, "axis0 synced to group duration");
        checkNear(axis1.getDuration(), dur, TOL, "axis1 synced to group duration");
        checkNear(axis2.getDuration(), dur, TOL, "axis2 (trapezoidal) synced to group duration");

        // All axes arrive together, and none moved early or late.
        float pos[6], vel[6], accel[6];
        group.evaluate(dur, pos, vel, accel);
        for (int i = 0; i < 3; ++i) {
            char buf[160];
            snprintf(buf, sizeof(buf), "axis%d arrives at its target", i);
            checkNear(pos[i], qf[i], TOL, buf);
        }

        // A jMax of 0 on an S-curve axis must fail the whole group plan
        // rather than quietly parking that axis -- the regression the
        // IDevice retrofit fixed, now exercised through SCurveProfile.
        {
            SCurveProfile a0, a1;
            ITrajectoryProfile* ps[2] = { &a0, &a1 };
            const float g0[2] = { 0.0f, 0.0f };
            const float gf[2] = { 20.0f, 2.0f };
            TrajectoryLimits bad[2] = {
                TrajectoryLimits(10.0f, 10.0f, 10.0f),
                TrajectoryLimits(10.0f, 10.0f, 0.0f),   // jMax == 0 -> invalid S-curve
            };
            TrajectoryGroup g;
            check(!g.plan(ps, g0, gf, bad, 2),
                  "group plan fails when an S-curve axis has jMax == 0");
        }
    }

    // ------------------------------------------------------------------
    // 12. Footprint -- SCurveProfile caches 7 boundary states, so it is
    //     materially bigger than TrapezoidalProfile. Recorded so an AVR
    //     SRAM regression is visible rather than silent.
    // ------------------------------------------------------------------
    {
        printf("\n-- 12. footprint --\n");
        printf("  INFO  sizeof(TrapezoidalProfile) = %u bytes\n",
               (unsigned)sizeof(TrapezoidalProfile));
        printf("  INFO  sizeof(SCurveProfile)      = %u bytes\n",
               (unsigned)sizeof(SCurveProfile));
        check(sizeof(SCurveProfile) <= 160, "sizeof(SCurveProfile) <= 160 bytes");
    }

    printf("\n%d passed, %d failed\n", s_passed, s_failed);
    return s_failed == 0 ? 0 : 1;
}
