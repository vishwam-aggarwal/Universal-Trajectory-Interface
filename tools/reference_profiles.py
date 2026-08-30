"""Independent reference implementation of the three profiles, in double precision.

This is a direct transcription of the MATLAB in Resources/MATLAB/ -- the same
equations the C++ in src/ was ported from, written a second time in a different
language so the two can be diffed against each other.

The point is that it is *independent*: plot_validation.py compares what the
board actually commanded against what this module says it should have
commanded. If the C++ port, the float arithmetic, or the AVR toolchain had
introduced an error, that comparison is what would show it. Comparing the
board's output against itself would prove nothing.

Pure standard library, no numpy needed.
"""

import math

# --------------------------------------------------------------------------
# Trapezoidal -- Trap_calculateTimeSegments.m + Trap_calculateMotion.m
# --------------------------------------------------------------------------


def trap_segments(q0, qf, a_max, v_max):
    d = abs(qf - q0)
    s_v = (v_max * v_max) / a_max
    if d >= s_v:
        t_a = v_max / a_max
        t_v = t_a + (d - s_v) / v_max
    else:
        t_a = math.sqrt(d / a_max)
        t_v = t_a
    return t_a, t_v, t_a + t_v


def trap_eval(t, q0, qf, a_max, v_max):
    """Returns (pos, vel, accel) at time t."""
    d = abs(qf - q0)
    if d == 0.0:
        return qf, 0.0, 0.0
    pol = 1.0 if qf > q0 else -1.0
    t_a, t_v, t_3 = trap_segments(q0, qf, a_max, v_max)
    a = a_max * pol
    if t < 0.0:
        t = 0.0
    if t >= t_3:
        return qf, 0.0, 0.0
    if t < t_a:
        return q0 + 0.5 * a * t * t, a * t, a
    v1 = a * t_a
    s1 = q0 + 0.5 * a * t_a * t_a
    if t < t_v:
        return s1 + v1 * (t - t_a), v1, 0.0
    s2 = s1 + v1 * (t_v - t_a)
    dt = t - t_v
    return s2 + v1 * dt - 0.5 * a * dt * dt, v1 - a * dt, -a


# --------------------------------------------------------------------------
# S-curve -- SCurve_calculateTimeSegments.m, _calculateInitialConditions.m,
#            _calculateMotion.m, _CalculateTraj.m
# --------------------------------------------------------------------------

_JERK_SIGN = (1.0, 0.0, -1.0, 0.0, -1.0, 0.0, 1.0)


def scurve_segments(displacement, v_max, a_max, j_max):
    """Port of SCurve_calculateTimeSegments.m. Returns absolute times t1..t7."""
    v_a = (a_max * a_max) / j_max
    s_a = 2.0 * (a_max ** 3) / (j_max * j_max)
    if v_max * j_max < a_max * a_max:
        s_v = 2.0 * v_max * math.sqrt(v_max / j_max)
    else:
        s_v = v_max * ((v_max / a_max) + (a_max / j_max))

    d = displacement
    if v_max < v_a and d >= s_a:                                  # A
        t_j = math.sqrt(v_max / j_max); t_a = t_j; t_v = d / v_max
    elif v_max >= v_a and d < s_a:                                # B
        t_j = (d / (2.0 * j_max)) ** (1.0 / 3.0); t_a = t_j; t_v = 2.0 * t_j
    elif v_max < v_a and d < s_a:                                 # C
        if d >= s_v:                                              # C.1
            t_j = math.sqrt(v_max / j_max); t_a = t_j; t_v = d / v_max
        else:                                                     # C.2
            t_j = (d / (2.0 * j_max)) ** (1.0 / 3.0); t_a = t_j; t_v = 2.0 * t_j
    else:                                                         # D
        if d >= s_v:                                              # D.1
            t_j = a_max / j_max; t_a = v_max / a_max; t_v = d / v_max
        else:                                                     # D.2
            t_j = a_max / j_max
            t_a = 0.5 * (math.sqrt((4.0 * d * j_max * j_max + a_max ** 3)
                                   / (a_max * j_max * j_max)) - (a_max / j_max))
            t_v = t_a + t_j
    return [t_j, t_a, t_j + t_a, t_v, t_j + t_v, t_v + t_a, t_v + t_a + t_j]


def _motion(a0, v0, s0, t0, jerk, T):
    dt = T - t0
    return (a0 + jerk * dt,
            v0 + a0 * dt + jerk * dt * dt / 2.0,
            s0 + v0 * dt + a0 * dt * dt / 2.0 + jerk * dt ** 3 / 6.0)


def _scurve_boundaries(q0, jerk, pol, seg):
    """Port of SCurve_calculateInitialConditions.m."""
    a, v, s, t0 = 0.0, 0.0, q0, 0.0
    out = []
    for i in range(7):
        a, v, s = _motion(a, v, s, t0, _JERK_SIGN[i] * jerk * pol, seg[i])
        out.append((a, v, s))
        t0 = seg[i]
    return out


class SCurve:
    def __init__(self, q0, qf, v_max, a_max, j_max):
        self.q0, self.qf = q0, qf
        d = abs(qf - q0)
        self.pol = 1.0 if qf > q0 else (-1.0 if qf < q0 else 0.0)
        self.jerk = j_max
        if d == 0.0:
            self.seg = [0.0] * 7
            self.bnd = [(0.0, 0.0, q0)] * 7
        else:
            self.seg = scurve_segments(d, v_max, a_max, j_max)
            self.bnd = _scurve_boundaries(q0, self.jerk, self.pol, self.seg)

    @property
    def duration(self):
        return self.seg[6]

    def eval(self, t):
        if t < 0.0:
            t = 0.0
        if self.seg[6] == 0.0 or t >= self.seg[6]:
            return self.qf, 0.0, 0.0
        for i in range(7):
            if t < self.seg[i]:
                jerk = _JERK_SIGN[i] * self.jerk * self.pol
                if i == 0:
                    a, v, s = _motion(0.0, 0.0, self.q0, 0.0, jerk, t)
                else:
                    pa, pv, ps = self.bnd[i - 1]
                    a, v, s = _motion(pa, pv, ps, self.seg[i - 1], jerk, t)
                return s, v, a
        return self.qf, 0.0, 0.0


# --------------------------------------------------------------------------
# Jerk-percent -- JerkPercent2SCurve/Script.m
# --------------------------------------------------------------------------


def jerk_percent_limits(q0, qf, v_max, a_max_nominal, jerk_percent):
    """Script.m's derivation. Returns (v_max, derived_a_max, derived_jerk)."""
    t_a, _, _ = trap_segments(q0, qf, a_max_nominal, v_max)
    T_A = max(1e-6, t_a)
    p = jerk_percent / 100.0
    a2 = 2.0 * a_max_nominal / (2.0 - p)
    j = 2.0 * a2 / (T_A * p)
    return v_max, a2, j


class JerkPercent(SCurve):
    def __init__(self, q0, qf, v_max, a_max_nominal, jerk_percent):
        v, a2, j = jerk_percent_limits(q0, qf, v_max, a_max_nominal, jerk_percent)
        self.derived = (v, a2, j)
        super().__init__(q0, qf, v, a2, j)


# --------------------------------------------------------------------------
# Uniform accessor used by the plotting script
# --------------------------------------------------------------------------


def make(kind, q0, qf, v_max, a_max, j_max, jerk_percent):
    """Returns (eval_fn, duration) for one of 'trap' | 'scurve' | 'jerkpct'."""
    if kind == "trap":
        _, _, dur = trap_segments(q0, qf, a_max, v_max)
        return (lambda t: trap_eval(t, q0, qf, a_max, v_max)), dur
    if kind == "scurve":
        p = SCurve(q0, qf, v_max, a_max, j_max)
        return p.eval, p.duration
    if kind == "jerkpct":
        p = JerkPercent(q0, qf, v_max, a_max, jerk_percent)
        return p.eval, p.duration
    raise ValueError("unknown profile kind: %r" % (kind,))
