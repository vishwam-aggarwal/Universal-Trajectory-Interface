#!/usr/bin/env python3
"""Emits reference pos/vel/accel traces as JSON on stdout, for the WASM
parity check in wasm/verify_parity.mjs.

The values come from tools/reference_profiles.py -- the independent
double-precision transcription of the same MATLAB that src/ was ported
from, already used to hold the on-device output to 0.001 deg in the
hardware validation rig. Using it again here means the browser build is
checked against the reference math, not against itself.

Nothing is committed: verify_parity.mjs spawns this at test time, so the
expected values cannot go stale relative to reference_profiles.py.

Pure stdlib (as is reference_profiles.py), so CI needs no pip install.
"""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tools"))

import reference_profiles as ref  # noqa: E402


# Each case is (id, kind, q0, qf, vMax, aMax, jMax, jerkPercent).
#
# The six s-curve branch selectors are lifted verbatim from
# tests/test_scurve.cpp, which documents which limit set drives which
# branch of SCurve_calculateTimeSegments.m -- so this sweep covers the
# same six branches the C++ suite does, rather than whichever ones a
# freshly invented parameter set happens to hit.
CASES = [
    # -- trapezoidal ------------------------------------------------------
    # Long enough to reach vMax: accel / cruise / decel.
    ("trap-cruise",     "trap",    0.0,  90.0,  90.0, 180.0,    0.0,  0.0),
    # Too short to reach vMax -- the triangular degenerate case.
    ("trap-triangular", "trap",    0.0,   5.0,  90.0, 180.0,    0.0,  0.0),
    ("trap-negative",   "trap",   10.0,   1.0,  10.0,  10.0,    0.0,  0.0),
    ("trap-zero",       "trap",    3.0,   3.0,  10.0,  10.0,    0.0,  0.0),

    # -- s-curve, all six branches ---------------------------------------
    ("scurve-D1",       "scurve",  0.0,  20.0,  10.0,  10.0,   10.0,  0.0),
    ("scurve-B",        "scurve", 10.0,   1.0,  10.0,  10.0,   10.0,  0.0),
    ("scurve-C1",       "scurve",  0.0,   5.0,   1.0,  10.0,   10.0,  0.0),
    ("scurve-C2",       "scurve",  0.0,  0.05,   1.0,  10.0,   10.0,  0.0),
    ("scurve-D2",       "scurve",  0.0,   0.5,  20.0,  50.0, 1000.0,  0.0),
    ("scurve-A",        "scurve",  0.0,   7.0,   3.0,   4.0,    5.0,  0.0),
    # The SCurveTrajectoryDemo sketch's exact move -- all seven segments
    # non-degenerate, boundaries on exact quarter-seconds.
    ("scurve-demo",     "scurve",  0.0,  90.0,  90.0, 180.0,  720.0,  0.0),

    # -- jerk-percent -----------------------------------------------------
    # The JerkPercentTrajectoryDemo sketch's exact move, at the default 66%.
    ("jerkpct-demo",    "jerkpct", 0.0,  90.0,  90.0, 180.0,    0.0, 66.0),
    ("jerkpct-10",      "jerkpct", 0.0,  90.0,  90.0, 180.0,    0.0, 10.0),
    ("jerkpct-100",     "jerkpct", 0.0,  90.0,  90.0, 180.0,    0.0, 100.0),
    # Short enough to stay in the trapezoid's triangular branch, which is
    # where T_A comes from.
    ("jerkpct-short",   "jerkpct", 0.0,   5.0,  90.0, 180.0,    0.0, 66.0),
    ("jerkpct-negative", "jerkpct", 40.0, 10.0,  90.0, 180.0,   0.0, 66.0),
    ("jerkpct-zero",    "jerkpct", 3.0,   3.0,  90.0, 180.0,    0.0, 66.0),
]

# Samples per case. Deliberately not a round divisor of any segment
# boundary, so sample instants land inside segments as well as on them.
N = 257


def main():
    out = []
    for case_id, kind, q0, qf, v_max, a_max, j_max, jerk_percent in CASES:
        eval_fn, duration = ref.make(kind, q0, qf, v_max, a_max, j_max,
                                     jerk_percent)

        # Sample a little past the end so the settled tail is compared too:
        # evaluate() clamps there, and "still holding qf after the move" is
        # a real property worth checking, not padding.
        t_end = duration * 1.1 if duration > 0.0 else 1.0

        samples = []
        for i in range(N):
            t = t_end * i / (N - 1)
            s, v, a = eval_fn(t)
            samples.append([t, s, v, a])

        entry = {
            "id": case_id,
            "kind": kind,
            "q0": q0,
            "qf": qf,
            "vMax": v_max,
            "aMax": a_max,
            "jMax": j_max,
            "jerkPercent": jerk_percent,
            "duration": duration,
            "tEnd": t_end,
            "samples": samples,
        }

        if kind == "jerkpct":
            _, derived_a, derived_j = ref.jerk_percent_limits(
                q0, qf, v_max, a_max, jerk_percent)
            entry["derivedAMax"] = derived_a
            entry["derivedJMax"] = derived_j

        out.append(entry)

    json.dump(out, sys.stdout)


if __name__ == "__main__":
    main()
