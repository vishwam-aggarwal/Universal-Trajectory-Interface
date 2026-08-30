#pragma once

#include "ITrajectoryProfile.h"
#include "SCurveProfile.h"

// Jerk-limited profile parameterised by a *jerk percentage* and a nominal
// acceleration limit, rather than by an absolute jerk limit. Port of
// Resources/MATLAB/JerkPercent2SCurve/Script.m.
//
// ------------------------------------------------------------------
// How this differs from SCurveProfile -- the tradeoff is inverted
// ------------------------------------------------------------------
// SCurveProfile takes an absolute jMax and pays for smoothness in *time*:
// jerk-limiting a move makes it take longer than the trapezoid would.
//
// This class instead keeps the duration **exactly equal to the trapezoidal
// duration for the same nominal limits** and pays for smoothness in
// *acceleration headroom*. The acceleration phase still lasts T_A (the
// trapezoid's own accel-phase duration) and still produces the same
// velocity change, but instead of a rectangular accel pulse it uses a
// trapezoidal one: jerk ramps occupy a fraction `jerkPercent` of T_A
// (split between the two ends), and the plateau is raised to compensate.
//
// Equating the areas -- A'*(T_A - t_j) = A*T_A with t_j = p*T_A/2 -- gives
// the two lines Script.m computes:
//
//     A' = 2*A / (2 - p)          derived ("modified") acceleration
//     J  = 2*A' / (T_A * p)       derived jerk
//
// then runs the *unmodified* 7-segment engine on (vMax, A', J). That engine
// is byte-identical to the one SCurveProfile already implements, which is
// why this class composes one rather than duplicating the math.
//
// ------------------------------------------------------------------
// !! Peak acceleration EXCEEDS the nominal limit you pass in !!
// ------------------------------------------------------------------
// A' = 2A/(2-p) is strictly greater than A for any p > 0:
//
//     jerkPercent    10%     33%     50%     66%     90%    100%
//     peak accel   1.05A   1.20A   1.33A   1.49A   1.82A    2.00A
//
// So `limits.aMax` here is a *nominal* figure the profile is allowed to
// overshoot -- NOT a hard ceiling. If aMax is your actuator's true maximum,
// derate it by 2/(2-p) before passing it in, or the move will command more
// acceleration than the hardware can deliver. This is the central
// difference from every other class in this library, where aMax binds.
// getDerivedLimits() reports what was actually used.
//
// On jerkPercent = 66: that figure comes from an MEI controller, where it
// is treated as the "optimum". It is a sweet spot rather than a
// mathematical optimum -- derived jerk falls monotonically as the
// percentage rises, so 100% minimises jerk outright, but needs 2x the
// acceleration. At 66% the jerk is within ~13% of that minimum while
// needing only ~1.49x. Hence the default below.
class JerkPercentProfile : public ITrajectoryProfile {
public:
    // Same rationale as TrapezoidalProfile's and SCurveProfile's: without
    // this, declaring the 4-arg override hides ITrajectoryProfile's 3-arg
    // convenience overload for calls made through a
    // JerkPercentProfile-typed expression.
    using ITrajectoryProfile::plan;

    // Default-constructible (needed for arrays of axes, e.g. a
    // TrajectoryGroup's). 66% is the MEI figure discussed above; it is a
    // starting point, not a universally correct value.
    explicit JerkPercentProfile(float jerkPercent = 66.0f)
        : _jerkPercent(jerkPercent) {}

    // Valid range is (0, 100]. 0 would mean "no jerk ramp at all" -- an
    // infinite jerk, i.e. a plain trapezoid -- and divides by zero in the
    // derivation, so it is rejected by plan() rather than special-cased.
    // Use TrapezoidalProfile if that is what you want.
    void  setJerkPercent(float jerkPercent) { _jerkPercent = jerkPercent; }
    float getJerkPercent() const            { return _jerkPercent; }

    // Port of Script.m's first four lines. Converts a *nominal* limit set
    // into the derived (vMax, A', J) that the 7-segment engine is actually
    // run with. Exposed publicly because it is useful on its own -- for
    // checking how much acceleration headroom a given percentage needs
    // before committing to it -- and because it makes the derivation
    // independently testable.
    //
    // vMax is passed through unchanged; aMax and jMax in the result are the
    // derived values. Depends on q0/qf because T_A does, so it must be
    // recomputed per move rather than cached across moves.
    //
    // Assumes valid inputs (nominal.vMax > 0, nominal.aMax > 0,
    // jerkPercent in (0, 100]); returns an all-zero TrajectoryLimits if
    // they are not, which any profile will then reject. plan() validates
    // before calling this.
    static TrajectoryLimits deriveLimits(float q0, float qf,
                                         const TrajectoryLimits& nominal,
                                         float jerkPercent);

    // limits.aMax is the NOMINAL acceleration (see the warning above);
    // limits.jMax is ignored entirely -- jerk comes from the percentage.
    //
    // Returns false, leaving the profile inert and parked at q0, when
    // nominal vMax or aMax is not strictly positive (NaN included), or
    // jerkPercent is outside (0, 100]. As with the sibling classes, a
    // targetDuration shorter than the achievable minimum clamps to the
    // minimum-time plan and returns false.
    bool plan(float q0, float qf, const TrajectoryLimits& limits,
              float targetDuration) override;

    bool evaluate(float t, float& pos, float& vel, float& accel) const override {
        return _inner.evaluate(t, pos, vel, accel);
    }

    float getDuration() const override { return _inner.getDuration(); }

    // What deriveLimits() produced for the last successful plan() --
    // .aMax is the peak acceleration the move will command, .jMax the jerk.
    // All zero before a successful plan(), and zeroed again by a failed one.
    //
    // Note this is the *pre-dilation* derivation. If plan() was given a
    // targetDuration longer than the minimum, the move is additionally
    // time-scaled, and the accelerations actually commanded are lower than
    // reported here (by k^2, for k = targetDuration/minDuration). It is
    // therefore an upper bound under dilation, and exact without it --
    // which is the direction that matters for sizing headroom.
    const TrajectoryLimits& getDerivedLimits() const { return _derived; }

private:
    float            _jerkPercent;
    // Explicitly zeroed: TrajectoryLimits' defaulted constructor leaves
    // vMax and aMax uninitialised (only jMax has a member initialiser), so
    // a bare `TrajectoryLimits _derived;` would start as garbage.
    TrajectoryLimits _derived{0.0f, 0.0f, 0.0f};
    SCurveProfile    _inner;
};
