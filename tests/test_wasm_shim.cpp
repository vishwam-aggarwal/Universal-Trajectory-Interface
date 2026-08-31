// Desktop tests for the WebAssembly shim in wasm/uti_wasm.cpp.
//
// The shim is what the browser-side Trajectory Lab (website/app.html) calls
// into. Its job is purely marshalling -- routing a kind index to the right
// profile object, sampling into a static buffer, and reporting the derived
// limits -- so this suite tests exactly that, not the profile math, which
// the five profile suites already cover against the MATLAB reference.
//
// Why this exists as an ordinary desktop suite rather than only being
// exercised through the built .wasm: the shim compiles unchanged with a
// host compiler (the EMSCRIPTEN_KEEPALIVE macro stubs itself out when
// __EMSCRIPTEN__ is undefined), so the marshalling logic can be held to the
// same CI both platforms already run. An emsdk toolchain is then needed
// only to *package* known-good code, not to find bugs in it.
//
// The end-to-end check that the compiled .wasm agrees numerically with the
// reference math is a separate thing and lives in wasm/verify_parity.mjs.

#include <cstdio>
#include <cmath>
#include "TrapezoidalProfile.h"
#include "SCurveProfile.h"
#include "JerkPercentProfile.h"

// Declared rather than included: this is the same flat C surface JavaScript
// binds to, so testing it through the same declarations keeps the two
// honest about each other.
extern "C" {
int    uti_plan(int kind, float q0, float qf, float vMax, float aMax,
                float jMax, float jerkPercent, float targetDuration);
int    uti_planned(int kind);
float  uti_duration(int kind);
float  uti_derived_aMax(int kind);
float  uti_derived_jMax(int kind);
int    uti_max_samples(void);
float* uti_sample(int kind, float t0, float t1, int n);
}

static int s_passed = 0, s_failed = 0;

static void check(bool cond, const char* label) {
    if (cond) { printf("  PASS  %s\n", label); ++s_passed; }
    else      { printf("  FAIL  %s\n", label); ++s_failed; }
}

static void checkNear(float a, float b, float tol, const char* label) {
    if (fabsf(a - b) <= tol) { printf("  PASS  %s\n", label); ++s_passed; }
    else {
        printf("  FAIL  %s  (got %.9f, want %.9f, diff %.3e)\n",
               label, a, b, fabsf(a - b));
        ++s_failed;
    }
}

// Mirrors the Kind enum in wasm/uti_wasm.cpp.
static const int TRAP    = 0;
static const int SCURVE  = 1;
static const int JERKPCT = 2;

// Column offsets within a sample row.
static const int STRIDE = 4;
static const int COL_T = 0, COL_POS = 1, COL_VEL = 2, COL_ACC = 3;

int main() {
    printf("test_wasm_shim\n");

    // ------------------------------------------------------------------
    // 1. Each kind routes to the right profile.
    //
    // Uses the three example sketches' documented moves, so the expected
    // durations are the ones examples/*/README-level docs already state:
    // trapezoidal 1.50 s, S-curve 1.75 s, and jerk-percent equal to the
    // trapezoid at 1.50 s. A mis-routed kind index would show up here as
    // the wrong duration rather than as a crash.
    // ------------------------------------------------------------------
    {
        printf("\n-- 1. kind routing, via the example sketches' moves --\n");

        check(uti_plan(TRAP, 0.0f, 90.0f, 90.0f, 180.0f, 0.0f, 0.0f, 0.0f) == 1,
              "trapezoidal plan succeeds");
        checkNear(uti_duration(TRAP), 1.50f, 1e-4f, "trapezoidal duration");

        check(uti_plan(SCURVE, 0.0f, 90.0f, 90.0f, 180.0f, 720.0f, 0.0f, 0.0f) == 1,
              "s-curve plan succeeds");
        checkNear(uti_duration(SCURVE), 1.75f, 1e-4f, "s-curve duration");

        check(uti_plan(JERKPCT, 0.0f, 90.0f, 90.0f, 180.0f, 0.0f, 66.0f, 0.0f) == 1,
              "jerk-percent plan succeeds");
        checkNear(uti_duration(JERKPCT), 1.50f, 1e-4f,
                  "jerk-percent duration equals the trapezoid's");
    }

    // ------------------------------------------------------------------
    // 2. All three stay planned at once.
    //
    // Not incidental: the app's "compare all three" overlay plans the same
    // move into each kind and then samples them one after another. A single
    // shared profile instance would silently return the last-planned curve
    // three times.
    // ------------------------------------------------------------------
    {
        printf("\n-- 2. the three kinds are independent --\n");
        check(uti_planned(TRAP) == 1,    "trapezoidal still planned");
        check(uti_planned(SCURVE) == 1,  "s-curve still planned");
        check(uti_planned(JERKPCT) == 1, "jerk-percent still planned");
        check(uti_duration(TRAP) != uti_duration(SCURVE),
              "trapezoidal and s-curve report different durations");
    }

    // ------------------------------------------------------------------
    // 3. Derived limits are reported, and only for jerk-percent.
    //
    // This is the figure the app puts in front of the user, because
    // JerkPercentProfile is the one class where limits.aMax is a nominal
    // value it overshoots rather than a ceiling: A' = 2A/(2-p), so
    // 180 * 2/(2-0.66) = 268.66 at the default 66%.
    // ------------------------------------------------------------------
    {
        printf("\n-- 3. derived limits --\n");
        checkNear(uti_derived_aMax(JERKPCT), 2.0f * 180.0f / (2.0f - 0.66f),
                  1e-2f, "derived aMax = 2A/(2-p)");
        check(uti_derived_aMax(JERKPCT) > 180.0f,
              "derived aMax exceeds the nominal aMax passed in");
        check(uti_derived_jMax(JERKPCT) > 0.0f, "derived jMax is reported");
        check(uti_derived_aMax(TRAP) == 0.0f,
              "trapezoidal reports no derived aMax");
        check(uti_derived_aMax(SCURVE) == 0.0f,
              "s-curve reports no derived aMax");
    }

    // ------------------------------------------------------------------
    // 4. Sampling.
    // ------------------------------------------------------------------
    {
        printf("\n-- 4. sampling --\n");
        check(uti_max_samples() == 2048, "max samples is 2048");

        uti_plan(TRAP, 0.0f, 90.0f, 90.0f, 180.0f, 0.0f, 0.0f, 0.0f);
        float* b = uti_sample(TRAP, 0.0f, 1.5f, 3);
        check(b != 0, "sample returns a buffer");

        checkNear(b[0 * STRIDE + COL_T],   0.00f, 1e-6f, "row 0 t");
        checkNear(b[0 * STRIDE + COL_POS], 0.00f, 1e-3f, "row 0 pos is q0");
        checkNear(b[1 * STRIDE + COL_T],   0.75f, 1e-6f, "row 1 t is the midpoint");
        checkNear(b[2 * STRIDE + COL_T],   1.50f, 1e-6f, "row 2 t is the end");
        checkNear(b[2 * STRIDE + COL_POS], 90.0f, 1e-3f, "row 2 pos is qf");
        checkNear(b[2 * STRIDE + COL_VEL],  0.0f, 1e-3f, "row 2 vel is zero");

        // The even-spacing divisor is n - 1, so n == 1 is the case that
        // would divide by zero if it were not guarded.
        float* one = uti_sample(TRAP, 0.75f, 1.5f, 1);
        check(one != 0, "n == 1 returns a buffer");
        checkNear(one[COL_T], 0.75f, 1e-6f, "n == 1 evaluates at t0, not t1");

        // Clamped rather than overrunning the static buffer.
        check(uti_sample(TRAP, 0.0f, 1.5f, 99999) != 0,
              "n above the cap is clamped, not refused");
        check(uti_sample(TRAP, 0.0f, 1.5f, 0) != 0,
              "n below 1 is clamped, not refused");

        // Sampling past the end is deliberate -- the app draws a settled
        // tail so the end of the move is visible.
        float* tail = uti_sample(TRAP, 1.5f, 3.0f, 2);
        checkNear(tail[1 * STRIDE + COL_POS], 90.0f, 1e-3f,
                  "past the duration, position holds at qf");
        checkNear(tail[1 * STRIDE + COL_VEL], 0.0f, 1e-3f,
                  "past the duration, velocity is zero");
    }

    // ------------------------------------------------------------------
    // 5. Bad input is rejected rather than crashing the page.
    // ------------------------------------------------------------------
    {
        printf("\n-- 5. invalid input --\n");
        check(uti_plan(7, 0.0f, 90.0f, 90.0f, 180.0f, 0.0f, 0.0f, 0.0f) == 0,
              "unknown kind: plan returns 0");
        check(uti_sample(7, 0.0f, 1.0f, 4) == 0,
              "unknown kind: sample returns null");
        check(uti_sample(-1, 0.0f, 1.0f, 4) == 0,
              "negative kind: sample returns null");
        check(uti_planned(7) == 0, "unknown kind: not planned");
        check(uti_duration(-1) == 0.0f, "unknown kind: zero duration");
        check(uti_derived_aMax(99) == 0.0f, "unknown kind: zero derived aMax");

        // jMax <= 0 is rejected by SCurveProfile rather than silently
        // downgraded to a trapezoid -- the app has to be able to tell the
        // user that, so the shim must pass the false through.
        check(uti_plan(SCURVE, 0.0f, 90.0f, 90.0f, 180.0f, 0.0f, 0.0f, 0.0f) == 0,
              "s-curve with jMax == 0 is rejected");
        check(uti_planned(SCURVE) == 0, "a failed plan clears the planned flag");

        check(uti_plan(TRAP, 0.0f, 90.0f, -1.0f, 180.0f, 0.0f, 0.0f, 0.0f) == 0,
              "trapezoidal with negative vMax is rejected");
        check(uti_plan(JERKPCT, 0.0f, 90.0f, 90.0f, 180.0f, 0.0f, 0.0f, 0.0f) == 0,
              "jerk-percent with jerkPercent == 0 is rejected");
        check(uti_derived_aMax(JERKPCT) == 0.0f,
              "a failed jerk-percent plan zeroes the derived limits");

        // A failure in one kind must not disturb another.
        check(uti_plan(TRAP, 0.0f, 90.0f, 90.0f, 180.0f, 0.0f, 0.0f, 0.0f) == 1,
              "a good plan still succeeds after a failed one");
        check(uti_planned(TRAP) == 1, "and is marked planned");
    }

    // ------------------------------------------------------------------
    // 6. Time dilation is passed through.
    //
    // The app exposes targetDuration so a visitor can see the same
    // mechanism TrajectoryGroup uses to make axes arrive together.
    // ------------------------------------------------------------------
    {
        printf("\n-- 6. time dilation --\n");
        check(uti_plan(TRAP, 0.0f, 90.0f, 90.0f, 180.0f, 0.0f, 0.0f, 3.0f) == 1,
              "dilating to 3.0 s succeeds");
        checkNear(uti_duration(TRAP), 3.0f, 1e-4f, "dilated duration is honoured");

        // Shorter than the minimum clamps to minimum time AND returns
        // false -- the library's documented contract, which the app relies
        // on to tell the user the request was not met.
        check(uti_plan(TRAP, 0.0f, 90.0f, 90.0f, 180.0f, 0.0f, 0.0f, 0.1f) == 0,
              "a shorter-than-possible duration is refused");
        checkNear(uti_duration(TRAP), 1.50f, 1e-4f,
                  "and clamps to the minimum-time plan");
    }

    printf("\n%d passed, %d failed\n", s_passed, s_failed);
    return s_failed == 0 ? 0 : 1;
}
