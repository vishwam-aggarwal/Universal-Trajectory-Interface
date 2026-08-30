#!/usr/bin/env python3
"""Compare a HardwareValidation capture against the independent reference math.

    python tools/plot_validation.py validation.csv

Produces three things:

  1. A per-profile position plot: what the board commanded, what the encoder
     measured, and what reference_profiles.py says the command should have
     been. The last of these is the one that matters for "is the port
     correct" -- it is computed here in double precision from the MATLAB
     equations, with no input from the board.

  2. A tracking-error plot (measured minus commanded), all three overlaid.

  3. A settling plot: the tail after each move nominally ends, which is where
     overshoot and ringing show up.

Also prints a numeric summary of the same, so the findings are quotable
without reading pixels off a chart.

Requires numpy and matplotlib.
"""

import argparse
import csv
import math
import os
import sys

try:
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("numpy and matplotlib are required:  pip install numpy matplotlib")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import reference_profiles as ref

# Must match the constants at the top of examples/HardwareValidation.ino.
Q0, QF = 4.0, 84.0
V_MAX, A_MAX, J_MAX, JERK_PERCENT = 90.0, 180.0, 720.0, 66.0

LABEL = {"trap": "Trapezoidal", "scurve": "S-curve (jerk limit)",
         "jerkpct": "S-curve (jerk percent)"}
ORDER = ["trap", "scurve", "jerkpct"]


def load(path):
    data = {k: {"t": [], "cmd": [], "meas": []} for k in ORDER}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            k = row["profile"]
            if k not in data:
                continue
            try:
                meas = float(row["meas_deg"])
            except (ValueError, TypeError):
                meas = math.nan
            data[k]["t"].append(float(row["t_ms"]) / 1000.0)
            data[k]["cmd"].append(float(row["cmd_deg"]))
            data[k]["meas"].append(meas)
    out = {}
    for k, v in data.items():
        if v["t"]:
            out[k] = {kk: np.asarray(vv, dtype=float) for kk, vv in v.items()}
    if not out:
        sys.exit(f"No usable rows in {path}")
    return out


def orient(meas, cmd):
    """The encoder's sense depends on how the magnet was fitted. Detect it
    rather than making the user reflash to fix a sign."""
    m = meas - (meas[~np.isnan(meas)][0] if np.any(~np.isnan(meas)) else 0.0)
    c = cmd - cmd[0]
    ok = ~np.isnan(m)
    if ok.sum() < 3 or np.std(c[ok]) == 0:
        return meas, False
    if np.corrcoef(m[ok], c[ok])[0, 1] < 0:
        return (cmd[0] - (meas - meas[~np.isnan(meas)][0])), True
    return meas, False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="file written by capture_validation.py")
    ap.add_argument("--out-prefix", default="validation",
                    help="prefix for the written PNGs")
    ap.add_argument("--band", type=float, default=0.5,
                    help="settling band in degrees (default 0.5)")
    args = ap.parse_args()

    data = load(args.csv)
    kinds = [k for k in ORDER if k in data]

    print("=" * 74)
    print("ON-DEVICE COMMAND vs INDEPENDENT REFERENCE")
    print("  (max |board commanded - double-precision MATLAB transcription|)")
    print("=" * 74)
    refmax = {}
    for k in kinds:
        d = data[k]
        fn, dur = ref.make(k, Q0, QF, V_MAX, A_MAX, J_MAX, JERK_PERCENT)
        expected = np.array([fn(t)[0] for t in d["t"]])
        err = np.abs(d["cmd"] - expected)
        refmax[k] = (expected, err.max())
        print(f"  {LABEL[k]:<26} duration {dur:6.4f}s   max deviation {err.max():.5f} deg")
    print("  A deviation of a few thousandths is float32-vs-float64 rounding.")
    print("  Anything larger is a real difference between the C++ and the MATLAB.")

    print()
    print("=" * 74)
    print("MEASURED RESPONSE")
    print("=" * 74)
    summary = {}
    for k in kinds:
        d = data[k]
        meas, flipped = orient(d["meas"], d["cmd"])
        _, dur = ref.make(k, Q0, QF, V_MAX, A_MAX, J_MAX, JERK_PERCENT)
        ok = ~np.isnan(meas)
        if ok.sum() < 3:
            print(f"  {LABEL[k]:<26} no encoder data")
            summary[k] = None
            continue
        err = meas - d["cmd"]
        tail = d["t"] >= dur
        # Overshoot past the target, and how long until it stays inside the band.
        travel = QF - Q0
        over = (np.nanmax(meas[tail]) - QF) if travel > 0 else (QF - np.nanmin(meas[tail]))
        settle = math.nan
        idx = np.where(tail & ok)[0]
        for i in idx:
            if np.all(np.abs(meas[idx[idx >= i]] - QF) <= args.band):
                settle = d["t"][i] - dur
                break
        summary[k] = dict(meas=meas, err=err, dur=dur, over=over,
                          settle=settle, flipped=flipped,
                          peak_err=np.nanmax(np.abs(err[~tail])) if np.any(~tail) else math.nan,
                          final=meas[ok][-1])
        settle_txt = (f"settle(+/-{args.band}) {settle:6.3f}s"
                      if not math.isnan(settle) else "never settles within band")
        print(f"  {LABEL[k]:<26} peak tracking err {summary[k]['peak_err']:6.2f} deg"
              f"   overshoot {over:+6.2f} deg   {settle_txt}")
        if flipped:
            print("      (encoder sense auto-detected as inverted and corrected)")

    # ---------------------------------------------------------------- plots --
    n = len(kinds)
    fig, axes = plt.subplots(n, 1, figsize=(9, 3.1 * n), sharex=True)
    if n == 1:
        axes = [axes]
    for ax, k in zip(axes, kinds):
        d, s = data[k], summary[k]
        expected, _ = refmax[k]
        ax.plot(d["t"], expected, "--", lw=2.4, color="#999999",
                label="reference (double precision)")
        ax.plot(d["t"], d["cmd"], "-", lw=1.4, color="#0072BD", label="commanded (on board)")
        if s is not None:
            ax.plot(d["t"], s["meas"], "-", lw=1.4, color="#D95319", label="measured (AS5600)")
            ax.axvline(s["dur"], color="#7E2F8E", ls=":", lw=1.2, label="profile ends")
        ax.axhline(QF, color="#BBBBBB", lw=0.8, zorder=0)
        ax.set_ylabel("angle (deg)")
        ax.set_title(LABEL[k], loc="left", fontsize=11)
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8, loc="lower right")
    axes[-1].set_xlabel("time (s)")
    fig.suptitle("Commanded vs measured vs independent reference", fontsize=12)
    fig.tight_layout()
    p1 = f"{args.out_prefix}_position.png"
    fig.savefig(p1, dpi=150)

    fig2, ax = plt.subplots(figsize=(9, 4))
    for k, c in zip(kinds, ["#0072BD", "#D95319", "#7E2F8E"]):
        if summary[k] is None:
            continue
        ax.plot(data[k]["t"], summary[k]["err"], lw=1.4, color=c, label=LABEL[k])
    ax.axhline(0, color="#BBBBBB", lw=0.8)
    ax.set_xlabel("time (s)"); ax.set_ylabel("measured - commanded (deg)")
    ax.set_title("Tracking error", loc="left")
    ax.grid(alpha=0.25); ax.legend(fontsize=9)
    fig2.tight_layout()
    p2 = f"{args.out_prefix}_error.png"
    fig2.savefig(p2, dpi=150)

    fig3, ax = plt.subplots(figsize=(9, 4))
    for k, c in zip(kinds, ["#0072BD", "#D95319", "#7E2F8E"]):
        s = summary[k]
        if s is None:
            continue
        t_rel = data[k]["t"] - s["dur"]
        m = t_rel >= -0.15
        ax.plot(t_rel[m], s["meas"][m] - QF, lw=1.4, color=c, label=LABEL[k])
    ax.axhline(0, color="#BBBBBB", lw=0.8)
    ax.axhline(args.band, color="#CCCCCC", ls=":", lw=0.8)
    ax.axhline(-args.band, color="#CCCCCC", ls=":", lw=0.8)
    ax.axvline(0, color="#999999", ls=":", lw=1.0)
    ax.set_xlabel("time after the profile ends (s)")
    ax.set_ylabel("measured - target (deg)")
    ax.set_title(f"Settling (dotted band = +/-{args.band} deg)", loc="left")
    ax.grid(alpha=0.25); ax.legend(fontsize=9)
    fig3.tight_layout()
    p3 = f"{args.out_prefix}_settling.png"
    fig3.savefig(p3, dpi=150)

    print(f"\nWrote {p1}, {p2}, {p3}")


if __name__ == "__main__":
    main()
