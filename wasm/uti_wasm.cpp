// WebAssembly shim over the three scalar trajectory profiles, for the
// browser-side Trajectory Lab (website/app.html).
//
// The point of this file is that the web app runs the *real* library.
// src/TrapezoidalProfile.cpp, src/SCurveProfile.cpp and
// src/JerkPercentProfile.cpp are compiled unmodified and in `float`, so a
// curve plotted in the browser is the same arithmetic an ATmega328P
// executes -- not a JavaScript re-derivation that can silently drift from
// the C++ (there are already three transcriptions of this math: the MATLAB
// it was ported from, src/, and tools/reference_profiles.py; a fourth in JS
// would be one more thing to keep in sync).
//
// Those three profiles are the only ones exposed here because they are the
// only ones that are dependency-free: TrajectoryGroup.h and CartesianMove.h
// include <IDevice.h> from the extern/Universal-Device-Interface submodule,
// so pulling either in would drag the submodule into this build for no
// benefit to the simulator.
//
// Deliberately a flat extern "C" surface rather than embind: it keeps the
// generated glue small, and none of these calls need to marshal anything
// richer than floats. Sampled output goes through one static buffer that
// JavaScript reads directly as a Float32Array view over the heap, so the
// hot path allocates nothing -- matching the library's own contract.
//
// Build: see wasm/build.sh. This file also compiles with an ordinary host
// compiler (the EMSCRIPTEN_KEEPALIVE macro is stubbed out below), which is
// what wasm/selftest.cpp uses to exercise it without an emsdk install.

#include "TrapezoidalProfile.h"
#include "SCurveProfile.h"
#include "JerkPercentProfile.h"

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

namespace {

// Profile kinds, mirrored by PROFILE_KIND in website/app.html.
enum Kind {
    KIND_TRAPEZOIDAL = 0,
    KIND_SCURVE      = 1,
    KIND_JERKPERCENT = 2,
    KIND_COUNT       = 3
};

// One instance per kind, all three live at once. The app's "compare all
// three" overlay plans the same move into each and samples them
// independently, so a single reused instance would not do.
TrapezoidalProfile g_trap;
SCurveProfile      g_scurve;
JerkPercentProfile g_jerkPct;

// Whether the last plan() for this kind succeeded. evaluate() on a profile
// whose plan() failed is inert (parked at q0) rather than undefined, but
// the app still needs to tell "flat because it failed" from "flat because
// the move is zero-distance".
bool g_planned[KIND_COUNT] = { false, false, false };

// What deriveLimits() produced, for KIND_JERKPERCENT only. Zero otherwise.
TrajectoryLimits g_derived[KIND_COUNT] = {
    TrajectoryLimits(0.0f, 0.0f, 0.0f),
    TrajectoryLimits(0.0f, 0.0f, 0.0f),
    TrajectoryLimits(0.0f, 0.0f, 0.0f)
};

ITrajectoryProfile* profileFor(int kind) {
    switch (kind) {
        case KIND_TRAPEZOIDAL: return &g_trap;
        case KIND_SCURVE:      return &g_scurve;
        case KIND_JERKPERCENT: return &g_jerkPct;
        default:               return 0;
    }
}

// Interleaved [t, pos, vel, accel] per sample. Sized for the widest plot
// the app draws (one sample per horizontal pixel on a very wide screen,
// with headroom); 2048 * 4 floats is 32 KB of static data, which is
// nothing in a browser but keeps the sampling path allocation-free.
const int   MAX_SAMPLES = 2048;
const int   STRIDE      = 4;
float       g_buf[MAX_SAMPLES * STRIDE];

}  // namespace

extern "C" {

// Plans a move into the profile identified by `kind`.
//
//   kind            0 = trapezoidal, 1 = s-curve, 2 = jerk-percent
//   jMax            used by kind 1 only; kind 0 ignores it, kind 2 derives
//                   its own from jerkPercent
//   jerkPercent     used by kind 2 only, valid range (0, 100]
//   targetDuration  0 = plan for minimum time; > 0 = time-dilate to exactly
//                   this duration
//
// Returns 1 on success, 0 on failure. A failure leaves the profile inert
// and parked at q0, exactly as the underlying plan() does -- note that a
// targetDuration shorter than the achievable minimum both clamps to the
// minimum-time plan *and* returns 0, which is the library's documented
// contract and not a partial failure.
EMSCRIPTEN_KEEPALIVE
int uti_plan(int kind, float q0, float qf, float vMax, float aMax,
             float jMax, float jerkPercent, float targetDuration) {
    ITrajectoryProfile* p = profileFor(kind);
    if (!p) {
        return 0;
    }

    g_planned[kind] = false;
    g_derived[kind] = TrajectoryLimits(0.0f, 0.0f, 0.0f);

    TrajectoryLimits limits(vMax, aMax, jMax);
    bool ok;

    if (kind == KIND_JERKPERCENT) {
        g_jerkPct.setJerkPercent(jerkPercent);
        ok = g_jerkPct.plan(q0, qf, limits, targetDuration);
        if (ok) {
            g_derived[kind] = g_jerkPct.getDerivedLimits();
        }
    } else {
        ok = p->plan(q0, qf, limits, targetDuration);
    }

    g_planned[kind] = ok;
    return ok ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE
int uti_planned(int kind) {
    if (kind < 0 || kind >= KIND_COUNT) {
        return 0;
    }
    return g_planned[kind] ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE
float uti_duration(int kind) {
    ITrajectoryProfile* p = profileFor(kind);
    return p ? p->getDuration() : 0.0f;
}

// The acceleration JerkPercentProfile actually commands, which is
// 2/(2-p) times the nominal aMax that was passed in -- 1.49x at the
// default 66%, 2x at 100%. This is the one class in the library where
// limits.aMax is not a ceiling, so the app surfaces this figure rather
// than letting the overshoot go unnoticed. Zero for the other two kinds.
EMSCRIPTEN_KEEPALIVE
float uti_derived_aMax(int kind) {
    if (kind < 0 || kind >= KIND_COUNT) {
        return 0.0f;
    }
    return g_derived[kind].aMax;
}

EMSCRIPTEN_KEEPALIVE
float uti_derived_jMax(int kind) {
    if (kind < 0 || kind >= KIND_COUNT) {
        return 0.0f;
    }
    return g_derived[kind].jMax;
}

EMSCRIPTEN_KEEPALIVE
int uti_max_samples(void) {
    return MAX_SAMPLES;
}

// Samples `n` points evenly across [t0, t1] into the static buffer and
// returns its address, for JavaScript to wrap in a Float32Array of length
// n * 4. Layout is interleaved [t, pos, vel, accel].
//
// n is clamped to [1, MAX_SAMPLES]; n == 1 evaluates at t0 alone. Sampling
// past getDuration() is fine and intentional -- evaluate() clamps, so the
// app draws a short settled tail to make the end of the move visible.
//
// Returns 0 for an unknown kind. Note this samples whatever state the
// profile is in: if plan() failed, every sample is the parked q0, which is
// why uti_planned() exists.
EMSCRIPTEN_KEEPALIVE
float* uti_sample(int kind, float t0, float t1, int n) {
    ITrajectoryProfile* p = profileFor(kind);
    if (!p) {
        return 0;
    }

    if (n < 1) {
        n = 1;
    }
    if (n > MAX_SAMPLES) {
        n = MAX_SAMPLES;
    }

    // Guard the n == 1 case explicitly: the even-spacing divisor is n - 1.
    const float dt = (n > 1) ? (t1 - t0) / static_cast<float>(n - 1) : 0.0f;

    for (int i = 0; i < n; ++i) {
        const float t = t0 + dt * static_cast<float>(i);
        float pos = 0.0f, vel = 0.0f, accel = 0.0f;
        p->evaluate(t, pos, vel, accel);

        float* row = &g_buf[i * STRIDE];
        row[0] = t;
        row[1] = pos;
        row[2] = vel;
        row[3] = accel;
    }

    return g_buf;
}

}  // extern "C"
