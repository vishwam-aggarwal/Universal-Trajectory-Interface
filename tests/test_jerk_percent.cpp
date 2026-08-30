// Desktop tests for JerkPercentProfile.
//
// Expected pos/vel/accel values were generated from a double-precision
// transcription of Resources/MATLAB/JerkPercent2SCurve/ (Script.m's
// derivation feeding the unmodified 7-segment engine), not derived by hand,
// so this is a numeric diff against the MATLAB rather than a restatement of
// the C++.
//
// The defining property under test is that the S-curve duration equals the
// trapezoidal duration for the same nominal limits -- smoothness is bought
// with acceleration headroom instead of time.

#include <cstdio>
#include <cmath>
#include "JerkPercentProfile.h"
#include "TrapezoidalProfile.h"
#include "TrajectoryGroup.h"

static int s_passed = 0, s_failed = 0;
static const float TOL = 1e-3f;   // values here run to ~600 units

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

static void evalCheck(const JerkPercentProfile& p, float t,
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

// Minimum-time duration of a plain trapezoid on the same nominal limits --
// the figure this profile is supposed to match exactly.
static float trapDuration(float q0, float qf, const TrajectoryLimits& nominal) {
    TrapezoidalProfile t;
    t.plan(q0, qf, nominal, 0.0f);
    return t.getDuration();
}

static void sweepCheck(const JerkPercentProfile& p, float q0, float qf,
                       const TrajectoryLimits& nominal, const char* label) {
    const float dur = p.getDuration();
    const int   N   = 4000;
    const float dt  = dur / N;
    const TrajectoryLimits& d = p.getDerivedLimits();

    float maxV = 0.0f, maxA = 0.0f, maxJerk = 0.0f, prevAcc = 0.0f;
    char buf[160];

    for (int i = 0; i <= N; ++i) {
        float pos, vel, accel;
        p.evaluate(dt * i, pos, vel, accel);
        if (fabsf(vel)   > maxV) maxV = fabsf(vel);
        if (fabsf(accel) > maxA) maxA = fabsf(accel);
        if (i > 0) {
            const float dAcc = fabsf(accel - prevAcc) / dt;
            if (dAcc > maxJerk) maxJerk = dAcc;
        }
        prevAcc = accel;
    }

    // vMax still binds -- it is passed through the derivation untouched.
    snprintf(buf, sizeof(buf), "%s  peak |vel| <= vMax", label);
    check(maxV <= nominal.vMax * 1.001f + TOL, buf);

    // aMax does NOT bind: the whole mechanism is overshooting it up to the
    // derived A'. Both halves matter, so both are asserted.
    snprintf(buf, sizeof(buf), "%s  peak |accel| <= derived A'", label);
    check(maxA <= d.aMax * 1.001f + TOL, buf);
    snprintf(buf, sizeof(buf), "%s  peak |accel| >= nominal aMax (headroom is used)", label);
    check(maxA >= nominal.aMax * 0.999f, buf);

    snprintf(buf, sizeof(buf), "%s  |jerk| <= derived J (accel is continuous)", label);
    check(maxJerk <= d.jMax * 1.05f + 1e-2f, buf);

    float pos, vel, accel;
    p.evaluate(dur, pos, vel, accel);
    snprintf(buf, sizeof(buf), "%s  lands exactly on qf", label);
    checkNear(pos, qf, TOL, buf);
}

int main() {
    printf("=== JerkPercentProfile ===\n\n");

    // ------------------------------------------------------------------
    // 1. The headline property: duration matches the trapezoid exactly,
    //    across percentages, distances, and both trapezoid branches.
    // ------------------------------------------------------------------
    {
        printf("-- 1. duration == trapezoidal duration --\n");
        TrajectoryLimits nominal{ 150.0f, 50.0f };
        const float dists[]    = { 200.0f, 600.0f, 50.0f, 1000.0f, 5.0f };
        const float percents[] = { 10.0f, 33.0f, 50.0f, 66.0f, 90.0f, 100.0f };

        for (unsigned di = 0; di < sizeof(dists) / sizeof(dists[0]); ++di) {
            for (unsigned pi = 0; pi < sizeof(percents) / sizeof(percents[0]); ++pi) {
                JerkPercentProfile p(percents[pi]);
                char buf[160];
                snprintf(buf, sizeof(buf), "d=%.0f jp=%.0f%%: plan succeeds",
                         dists[di], percents[pi]);
                check(p.plan(0.0f, dists[di], nominal, 0.0f), buf);

                snprintf(buf, sizeof(buf), "d=%.0f jp=%.0f%%: duration == trapezoid",
                         dists[di], percents[pi]);
                checkNear(p.getDuration(), trapDuration(0.0f, dists[di], nominal),
                          TOL, buf);
            }
        }
    }

    // ------------------------------------------------------------------
    // 2. Derived limits -- Script.m's two formulas.
    //    A' = 2A/(2-p),  J = 2A'/(T_A * p)
    // ------------------------------------------------------------------
    {
        printf("\n-- 2. derived limits (Script.m) --\n");
        TrajectoryLimits nominal{ 150.0f, 50.0f };

        // d=200: trapezoid t_a = V/A = 3.0? No -- s_v = V^2/A = 450 > 200,
        // so t_a = sqrt(d/A) = sqrt(4) = 2.0.
        TrajectoryLimits d200 = JerkPercentProfile::deriveLimits(0.0f, 200.0f, nominal, 66.0f);
        checkNear(d200.vMax, 150.000000000f, TOL, "d=200 jp=66: vMax passed through");
        checkNear(d200.aMax,  74.626865672f, TOL, "d=200 jp=66: A' == 2A/(2-p)");
        checkNear(d200.jMax, 113.071008593f, 1e-2f, "d=200 jp=66: J == 2A'/(T_A*p)");

        // d=600 >= s_v=450 -> t_a = V/A = 3.0, so J differs while A' does not.
        TrajectoryLimits d600 = JerkPercentProfile::deriveLimits(0.0f, 600.0f, nominal, 66.0f);
        checkNear(d600.aMax,  74.626865672f, TOL,   "d=600 jp=66: A' unchanged (independent of distance)");
        checkNear(d600.jMax,  75.380672396f, 1e-2f, "d=600 jp=66: J lower (longer T_A)");

        // A' depends only on the percentage. Spot-check the headroom table
        // quoted in the header.
        struct HR { float pct, mult; };
        HR table[] = { {10.0f, 1.0526316f}, {33.0f, 1.1976048f}, {50.0f, 1.3333333f},
                       {66.0f, 1.4925373f}, {90.0f, 1.8181818f}, {100.0f, 2.0f} };
        for (unsigned i = 0; i < sizeof(table) / sizeof(table[0]); ++i) {
            TrajectoryLimits dl = JerkPercentProfile::deriveLimits(0.0f, 200.0f, nominal, table[i].pct);
            char buf[160];
            snprintf(buf, sizeof(buf), "jp=%.0f%%: peak accel == %.4fx nominal",
                     table[i].pct, table[i].mult);
            checkNear(dl.aMax / nominal.aMax, table[i].mult, 1e-4f, buf);
        }

        // Invalid input -> all-zero limits.
        TrajectoryLimits bad = JerkPercentProfile::deriveLimits(0.0f, 200.0f, nominal, 0.0f);
        check(bad.vMax == 0.0f && bad.aMax == 0.0f && bad.jMax == 0.0f,
              "deriveLimits(jp=0) returns all-zero limits");
    }

    // ------------------------------------------------------------------
    // 3. Case D.2  q0=0 qf=200 ACC=50 V=150 jp=66
    //    T_A=2, A'=74.626866, J=113.071009, segments on 0.66/1.34 boundaries
    // ------------------------------------------------------------------
    {
        printf("\n-- 3. q0=0 qf=200 ACC=50 V=150 jp=66 --\n");
        TrajectoryLimits nominal{ 150.0f, 50.0f };
        JerkPercentProfile p(66.0f);
        check(p.plan(0.0f, 200.0f, nominal, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 4.0f, TOL, "duration == 4 (== trapezoid)");

        evalCheck(p, 0.00f,   0.000000000f,   0.000000000f,   0.000000000f, true,
                  "t=0.00  start");
        evalCheck(p, 0.66f,   5.417910448f,  24.626865672f,  74.626865672f, true,
                  "t=0.66  t1: end of jerk ramp, at A'");
        evalCheck(p, 1.00f,  18.104477612f,  50.000000000f,  74.626865672f, true,
                  "t=1.00  constant accel at A'");
        evalCheck(p, 1.34f,  39.417910448f,  75.373134328f,  74.626865672f, true,
                  "t=1.34  t2: end of constant-accel phase");
        evalCheck(p, 2.00f, 100.000000000f, 100.000000000f,   0.000000000f, true,
                  "t=2.00  t3/t4: midpoint, peak speed (under vMax)");
        evalCheck(p, 2.66f, 160.582089552f,  75.373134328f, -74.626865672f, true,
                  "t=2.66  t5: at -A'");
        evalCheck(p, 3.34f, 194.582089552f,  24.626865672f, -74.626865672f, true,
                  "t=3.34  t6: end of constant-decel phase");
        evalCheck(p, p.getDuration(), 200.0f, 0.0f, 0.0f, false,
                  "t=t7    settled");

        checkNear(p.getDerivedLimits().aMax, 74.626865672f, TOL,
                  "getDerivedLimits().aMax reports the real peak accel");
        sweepCheck(p, 0.0f, 200.0f, nominal, "d=200 jp=66");
    }

    // ------------------------------------------------------------------
    // 4. Case D.1 -- long enough to cruise at vMax.
    //    q0=0 qf=600, T_A=3, J=75.380672
    // ------------------------------------------------------------------
    {
        printf("\n-- 4. q0=0 qf=600 ACC=50 V=150 jp=66 (cruise phase) --\n");
        TrajectoryLimits nominal{ 150.0f, 50.0f };
        JerkPercentProfile p(66.0f);
        check(p.plan(0.0f, 600.0f, nominal, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 7.0f, TOL, "duration == 7 (== trapezoid)");

        evalCheck(p, 0.99f,  12.190298507f,  36.940298507f,  74.626865672f, true,
                  "t=0.99  t1: end of jerk ramp");
        evalCheck(p, 2.01f,  88.690298507f, 113.059701493f,  74.626865672f, true,
                  "t=2.01  t2: end of constant-accel phase");
        evalCheck(p, 3.00f, 225.000000000f, 150.000000000f,   0.000000000f, true,
                  "t=3.00  t3: cruise begins at vMax");
        evalCheck(p, 3.50f, 300.000000000f, 150.000000000f,   0.000000000f, true,
                  "t=3.50  cruising");
        evalCheck(p, 4.00f, 375.000000000f, 150.000000000f,   0.000000000f, true,
                  "t=4.00  t4: cruise ends");
        evalCheck(p, 4.99f, 511.309701493f, 113.059701493f, -74.626865672f, true,
                  "t=4.99  t5");
        evalCheck(p, 6.01f, 587.809701493f,  36.940298507f, -74.626865672f, true,
                  "t=6.01  t6");
        evalCheck(p, p.getDuration(), 600.0f, 0.0f, 0.0f, false, "t=t7    settled");

        sweepCheck(p, 0.0f, 600.0f, nominal, "d=600 jp=66");
    }

    // ------------------------------------------------------------------
    // 5. Negative direction  q0=150 qf=-50 (same 200-unit distance as #3,
    //    so every value mirrors)
    // ------------------------------------------------------------------
    {
        printf("\n-- 5. negative direction  q0=150 qf=-50 --\n");
        TrajectoryLimits nominal{ 150.0f, 50.0f };
        JerkPercentProfile p(66.0f);
        check(p.plan(150.0f, -50.0f, nominal, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 4.0f, TOL, "duration == 4");

        evalCheck(p, 0.66f, 144.582089552f, -24.626865672f, -74.626865672f, true,
                  "t=0.66  t1 (negative)");
        evalCheck(p, 2.00f,  50.000000000f, -100.00000000f,   0.000000000f, true,
                  "t=2.00  midpoint");
        evalCheck(p, 3.34f, -44.582089552f, -24.626865672f,  74.626865672f, true,
                  "t=3.34  t6");
        evalCheck(p, p.getDuration(), -50.0f, 0.0f, 0.0f, false, "t=t7    settled");

        sweepCheck(p, 150.0f, -50.0f, nominal, "negative");
    }

    // ------------------------------------------------------------------
    // 6. Percentage extremes.
    //    jp=100 -> A'=2A, accel is triangular (no constant-accel phase).
    //    jp=10  -> nearly trapezoidal, brief jerk ramps.
    // ------------------------------------------------------------------
    {
        printf("\n-- 6. jp=100 (2x accel, triangular accel profile) --\n");
        TrajectoryLimits nominal{ 150.0f, 50.0f };
        JerkPercentProfile p(100.0f);
        check(p.plan(0.0f, 200.0f, nominal, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 4.0f, TOL, "duration still == 4");
        checkNear(p.getDerivedLimits().aMax, 100.0f, TOL, "A' == 2 * nominal aMax");
        checkNear(p.getDerivedLimits().jMax, 100.0f, 1e-2f, "J == 100");

        evalCheck(p, 1.0f,  16.666666667f,  50.0f,  100.0f, true, "t=1.0  peak accel");
        evalCheck(p, 2.0f, 100.000000000f, 100.0f,    0.0f, true, "t=2.0  midpoint");
        evalCheck(p, 3.0f, 183.333333333f,  50.0f, -100.0f, true, "t=3.0  peak decel");
        evalCheck(p, p.getDuration(), 200.0f, 0.0f, 0.0f, false, "t=t7   settled");
        sweepCheck(p, 0.0f, 200.0f, nominal, "jp=100");

        printf("\n-- 6b. jp=10 (nearly trapezoidal) --\n");
        JerkPercentProfile q(10.0f);
        check(q.plan(0.0f, 200.0f, nominal, 0.0f), "plan returns true");
        checkNear(q.getDuration(), 4.0f, TOL, "duration still == 4");
        checkNear(q.getDerivedLimits().aMax, 52.631578947f, TOL, "A' == 1.0526 * nominal");
        evalCheck(q, 0.10f,   0.087719298f,  2.631578947f, 52.631578947f, true,
                  "t=0.10  t1: brief jerk ramp done");
        evalCheck(q, 1.90f,  90.087719298f, 97.368421053f, 52.631578947f, true,
                  "t=1.90  t2");
        evalCheck(q, q.getDuration(), 200.0f, 0.0f, 0.0f, false, "t=t7   settled");
        sweepCheck(q, 0.0f, 200.0f, nominal, "jp=10");
    }

    // ------------------------------------------------------------------
    // 7. Zero-distance move -- Script.m's own configuration, which has
    //    currentPos == setPoint_Pos == 150. This is the case the
    //    max(1e-6, t1) floor exists for: the trapezoid's accel time is 0,
    //    so the jerk derivation would otherwise divide by zero.
    // ------------------------------------------------------------------
    {
        printf("\n-- 7. zero-distance (Script.m's own config) --\n");
        TrajectoryLimits nominal{ 150.0f, 50.0f };
        JerkPercentProfile p(66.0f);
        check(p.plan(150.0f, 150.0f, nominal, 0.0f), "plan returns true");
        checkNear(p.getDuration(), 0.0f, TOL, "duration == 0");
        evalCheck(p, 0.0f, 150.0f, 0.0f, 0.0f, false, "t=0  parked");
        evalCheck(p, 1.0f, 150.0f, 0.0f, 0.0f, false, "t=1  still parked");

        // The floor kept the derivation finite rather than producing inf/NaN.
        const TrajectoryLimits& d = p.getDerivedLimits();
        check(d.jMax > 0.0f && d.jMax == d.jMax, "derived jerk is finite (1e-6 floor held)");
    }

    // ------------------------------------------------------------------
    // 8. Invalid inputs -- inert and parked at q0, as the siblings do.
    // ------------------------------------------------------------------
    {
        printf("\n-- 8. invalid inputs --\n");
        const float nan = sqrtf(-1.0f);
        TrajectoryLimits nominal{ 150.0f, 50.0f };

        struct Case { float pct; TrajectoryLimits lim; const char* what; };
        Case cases[] = {
            {  66.0f, TrajectoryLimits(  0.0f, 50.0f), "vMax == 0"        },
            {  66.0f, TrajectoryLimits( -1.0f, 50.0f), "vMax < 0"         },
            {  66.0f, TrajectoryLimits(150.0f,  0.0f), "aMax == 0"        },
            {  66.0f, TrajectoryLimits(150.0f, -1.0f), "aMax < 0"         },
            {  66.0f, TrajectoryLimits(   nan, 50.0f), "vMax NaN"         },
            {  66.0f, TrajectoryLimits(150.0f,   nan), "aMax NaN"         },
            {   0.0f, nominal,                         "jerkPercent == 0" },
            {  -5.0f, nominal,                         "jerkPercent < 0"  },
            { 101.0f, nominal,                         "jerkPercent > 100"},
            {    nan, nominal,                         "jerkPercent NaN"  },
        };

        for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
            JerkPercentProfile p(cases[i].pct);
            char buf[160];
            snprintf(buf, sizeof(buf), "%s: plan returns false", cases[i].what);
            check(!p.plan(20.0f, 90.0f, cases[i].lim, 0.0f), buf);
            snprintf(buf, sizeof(buf), "%s: duration == 0", cases[i].what);
            checkNear(p.getDuration(), 0.0f, TOL, buf);

            float pos, vel, accel;
            const bool moving = p.evaluate(0.5f, pos, vel, accel);
            snprintf(buf, sizeof(buf), "%s: parked at q0, not moving", cases[i].what);
            check(!moving && fabsf(pos - 20.0f) <= TOL
                          && fabsf(vel) <= TOL && fabsf(accel) <= TOL, buf);
        }

        // A failed plan() must not leave a previously-good plan runnable.
        {
            JerkPercentProfile p(66.0f);
            check(p.plan(0.0f, 200.0f, nominal, 0.0f), "first plan succeeds");
            p.setJerkPercent(0.0f);
            check(!p.plan(0.0f, 200.0f, nominal, 0.0f), "second plan (jp=0) fails");
            checkNear(p.getDuration(), 0.0f, TOL, "failed re-plan clears the old duration");
            float pos, vel, accel;
            check(!p.evaluate(1.0f, pos, vel, accel),
                  "failed re-plan leaves evaluate() inert, not replaying the old move");
        }
    }

    // ------------------------------------------------------------------
    // 9. Time dilation -- delegated to SCurveProfile's uniform scaling,
    //    which is shape-preserving, so the profile stays a jerk-percent
    //    S-curve rather than degenerating.
    // ------------------------------------------------------------------
    {
        printf("\n-- 9. time dilation --\n");
        TrajectoryLimits nominal{ 150.0f, 50.0f };

        JerkPercentProfile p(66.0f);
        check(p.plan(0.0f, 200.0f, nominal, 8.0f), "T=8: request satisfied");
        checkNear(p.getDuration(), 8.0f, TOL, "T=8: duration == requested");

        // Dilating by k=2 scales accelerations by 1/k^2, so the actual peak
        // is a quarter of the undilated derived A' -- which is exactly why
        // getDerivedLimits() is documented as an upper bound under dilation.
        float pos, vel, accel, maxA = 0.0f, maxV = 0.0f;
        for (int i = 0; i <= 4000; ++i) {
            p.evaluate(8.0f * i / 4000.0f, pos, vel, accel);
            if (fabsf(accel) > maxA) maxA = fabsf(accel);
            if (fabsf(vel)   > maxV) maxV = fabsf(vel);
        }
        checkNear(maxA, 74.626865672f / 4.0f, 1e-1f, "T=8: peak accel == A'/k^2");
        check(maxA <= p.getDerivedLimits().aMax,
              "T=8: getDerivedLimits().aMax is an upper bound");
        check(maxV <= nominal.vMax + TOL, "T=8: still under vMax");
        p.evaluate(8.0f, pos, vel, accel);
        checkNear(pos, 200.0f, TOL, "T=8: lands on qf");

        // Shorter than achievable -> clamp and report unmet.
        JerkPercentProfile q(66.0f);
        check(!q.plan(0.0f, 200.0f, nominal, 1.0f), "T=1 (< min): request unmet");
        checkNear(q.getDuration(), 4.0f, TOL, "T=1: falls back to minimum time");
    }

    // ------------------------------------------------------------------
    // 10. Polymorphism and composition.
    // ------------------------------------------------------------------
    {
        printf("\n-- 10. polymorphism and TrajectoryGroup --\n");
        TrajectoryLimits nominal{ 150.0f, 50.0f };

        JerkPercentProfile concrete(66.0f);
        check(concrete.plan(0.0f, 200.0f, nominal), "3-arg plan() on concrete type");

        JerkPercentProfile owned(66.0f);
        ITrajectoryProfile& base = owned;
        check(base.plan(0.0f, 200.0f, nominal), "3-arg plan() through base reference");
        float pos, vel, accel;
        check(base.evaluate(2.0f, pos, vel, accel), "base evaluate() dispatches virtually");
        checkNear(pos, 100.0f, TOL, "base evaluate(): midpoint == 100");

        // Mixed group: jerk-percent axis, plain S-curve axis, trapezoid axis.
        JerkPercentProfile axis0(66.0f);
        SCurveProfile      axis1;
        TrapezoidalProfile axis2;
        ITrajectoryProfile* profiles[3] = { &axis0, &axis1, &axis2 };
        const float q0[3] = { 0.0f, 0.0f, 0.0f };
        const float qf[3] = { 600.0f, 20.0f, 30.0f };
        TrajectoryLimits lims[3] = {
            TrajectoryLimits(150.0f, 50.0f),            // jMax unused here
            TrajectoryLimits(150.0f, 50.0f, 100.0f),
            TrajectoryLimits(150.0f, 50.0f),
        };

        TrajectoryGroup group;
        check(group.plan(profiles, q0, qf, lims, 3), "group plan succeeds with a mixed axis set");
        const float dur = group.getDuration();
        checkNear(dur, 7.0f, TOL, "group duration == slowest axis (7 s)");

        float gp[6], gv[6], ga[6];
        group.evaluate(dur, gp, gv, ga);
        for (int i = 0; i < 3; ++i) {
            char buf[160];
            snprintf(buf, sizeof(buf), "axis%d arrives at its target", i);
            checkNear(gp[i], qf[i], TOL, buf);
        }

        // A jerk-percent axis with a bad percentage fails the whole group.
        JerkPercentProfile bad(0.0f);
        ITrajectoryProfile* ps[1] = { &bad };
        const float b0[1] = { 0.0f }, bf[1] = { 100.0f };
        TrajectoryLimits bl[1] = { TrajectoryLimits(150.0f, 50.0f) };
        TrajectoryGroup g;
        check(!g.plan(ps, b0, bf, bl, 1), "group plan fails when a jerk-percent axis has jp=0");
    }

    // ------------------------------------------------------------------
    // 11. Footprint.
    // ------------------------------------------------------------------
    {
        printf("\n-- 11. footprint --\n");
        printf("  INFO  sizeof(SCurveProfile)      = %u bytes\n",
               (unsigned)sizeof(SCurveProfile));
        printf("  INFO  sizeof(JerkPercentProfile) = %u bytes\n",
               (unsigned)sizeof(JerkPercentProfile));
        check(sizeof(JerkPercentProfile) <= 192, "sizeof(JerkPercentProfile) <= 192 bytes");
    }

    printf("\n%d passed, %d failed\n", s_passed, s_failed);
    return s_failed == 0 ? 0 : 1;
}
