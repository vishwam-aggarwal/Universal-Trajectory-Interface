# Universal-Trajectory-Interface

## What this is

A standalone, platform-agnostic C++ library for real-time trajectory (motion
profile) generation. Given a start value, a target value, and kinematic limits,
it produces a time-parameterized position/velocity/acceleration setpoint on
demand — the piece that turns a step change in target into a smooth, bounded
motion profile.

## Relationship to other repos

Sibling project: **Universal-Motor-Interface**
(https://github.com/vishwam-aggarwal/Universal-Motor-Interface) — a hardware
abstraction layer for motors (RC servo, ODrive over CAN, simulated), already
built. This repo has **no dependency on it and never will** — they only get
composed together later, at a "Motion Device" layer that doesn't live here.

Same design philosophy as that repo: an abstract interface (`I...`) with
swappable concrete implementations, and a platform-independent core with any
hardware-specific glue isolated to the edges (and in this repo's case — there
shouldn't be any hardware-specific glue at all).

## Hard constraints — read before writing any code

- **Zero hardware/platform dependency.** No `Arduino.h`, no `millis()`, no OS
  calls, no dynamic memory tied to a specific target. Plain C++11+.
- **Must build and run identically on two very different targets:**
  1. Arduino/Teensy — the near-term bring-up target (a 4-DOF RC-servo test
     arm), used for physically validating the profile shape.
  2. **Linux x86 running under a SOEM EtherCAT master** — the ultimate target
     for this whole project. Hard real-time, cyclic exchange typically
     1–4 kHz.
- **Real-time-safe hot path.** `evaluate()` (and anything that runs every
  control tick) must be allocation-free, exception-free, and have bounded/
  deterministic execution time. No `new`, no allocating STL containers on
  that path. This is a hard requirement for the EtherCAT target, not just
  embedded good practice.
- **Stateless with respect to time.** These classes never own a clock.
  `plan()` establishes a move relative to t=0; the caller supplies elapsed
  time `t` (seconds since the move was planned) into `evaluate()`. This is
  what makes the library unit-testable on a desktop build, independent of
  any hardware, by feeding it the same `t` values used against a MATLAB
  reference trace.
- **Units are generic.** Just `float`s — radians, mm, degrees, whatever the
  caller uses consistently. Not robot-specific; should be equally usable for
  a CNC axis or a gimbal as for a robot joint.

## AVR (arduino:avr:*) support

`arduino:avr:*` (Uno, Nano, Mega) is a supported compile target alongside
SAMD/ARM and Linux x86. avr-gcc ships no C++ standard library (no `<cmath>`,
`<vector>`, etc.), only the C `math.h`. The fix was swapping
`#include <cmath>` → `#include <math.h>` in the four files that used it
(`Vec3.h`, `Quatf.h`, `TrapezoidalProfile.cpp`, `ArcPath.cpp`) — none of them
called anything via `std::`, so this was a drop-in change, not a rewrite.
Because Arduino's build system compiles every `.cpp` in `src/` regardless of
what the sketch includes, this had to be fixed everywhere, not just in the
joint-space files, even though the Cartesian-path classes
(`ArcPath`/`LinePath`/`CartesianMove`/`Vec3`/`Quatf`) are compile-verified on
AVR only — not performance-verified. Running float trig/sqrt on a 16 MHz,
no-FPU AVR chip is expected to be slow; that's an accepted tradeoff for a
sketch that never calls them, not a bug to design around.

## Class structure (agreed design — implement to this spec)

### `ITrajectoryProfile` — single scalar axis, the reusable atomic unit

```cpp
struct TrajectoryLimits {
    float vMax;
    float aMax;
    float jMax = 0.0f;   // 0 = unused (trapezoidal); >0 = jerk limit (s-curve)
};
```

(The real `src/TrajectoryLimits.h` also adds an explicit 2/3-arg constructor
alongside `= default`, not shown above -- needed so `TrajectoryLimits{vMax,
aMax}` brace-init compiles under strict C++11, not just C++14+. A struct
with a default member initializer like `jMax` above stops being an
aggregate until C++14, and that gap only surfaced once CI started building
with a real C++11-strict compiler (gcc on Linux) instead of just MSVC.)

```cpp
class ITrajectoryProfile {
public:
    virtual ~ITrajectoryProfile() = default;

    // targetDuration = 0  -> plan for minimum time given the limits
    // targetDuration > 0  -> stretch (time-dilate) the move to exactly this
    //                        duration; used later for multi-axis sync
    virtual bool plan(float q0, float qf, const TrajectoryLimits& limits,
                       float targetDuration) = 0;

    // Convenience overload for callers going through an ITrajectoryProfile&/*
    // who don't need to specify targetDuration. Deliberately non-virtual and
    // forwards to the full signature -- a default argument on the virtual
    // itself would resolve statically off the pointer/reference's declared
    // type, not polymorphically off the override actually invoked, so a
    // future implementation with a different default would be silently
    // ignored through a base pointer. See ITrajectoryProfile.h for the full
    // rationale.
    bool plan(float q0, float qf, const TrajectoryLimits& limits) {
        return plan(q0, qf, limits, 0.0f);
    }

    // Pure query, no side effects. Clamped for t < 0 or t > duration.
    // Returns true while still in motion, false once settled at qf.
    virtual bool evaluate(float t, float& pos, float& vel, float& accel) const = 0;

    virtual float getDuration() const = 0;
};
```

- `evaluate()` outputs **kinematic** quantities only (pos/vel/accel) — no
  torque. Torque feedforward would require a dynamics model (per-link
  inertia) that doesn't exist yet; that's out of scope here.
- **Every new `ITrajectoryProfile` implementation's `plan()` override must
  declare all 4 params with no default of its own** (matching the pure
  virtual exactly, e.g. `TrapezoidalProfile::plan(...)`  below) — the
  3-arg convenience path is the base class's job, not each override's.
- **Every new implementation must also add `using ITrajectoryProfile::plan;`
  in its own class body**, alongside the `plan(...)` override (see
  `TrapezoidalProfile.h`). Without it, declaring the override **hides**
  every base-class overload of that name from lookup on the concrete type
  (ordinary C++ name-hiding — lookup stops at the first class scope where
  the name appears at all, before overload resolution runs — independent
  of the virtual-dispatch fix above, and it bites regardless of whether the
  override's signature actually conflicts with the hidden overload or not).
  Originally this repo's answer to that was "callers holding a concrete
  type must always pass all 4 args explicitly" (the desktop tests still do,
  and that's still completely valid) — but `using` removes the restriction
  instead of just documenting around it, so both call forms work whether
  you're holding an `ITrajectoryProfile&`/`*` or a concrete
  `TrapezoidalProfile` directly. Forgetting the `using` line on a future
  implementation (e.g. `SCurveProfile`) won't fail to compile — it'll just
  silently bring back the "3-arg form only works through a base
  reference" restriction for that one class, so don't skip it.

### `TrapezoidalProfile : ITrajectoryProfile` — build this first

Classic trapezoidal velocity profile (accel / cruise / decel), with the
degenerate case of a move too short to reach `vMax` (triangular profile)
handled internally. `jMax` in `TrajectoryLimits` is ignored. Implements the
full 4-arg `plan(q0, qf, limits, targetDuration)` via `override`, with no
default argument of its own (see the note on `ITrajectoryProfile::plan()`
above).

### `SCurveProfile : ITrajectoryProfile` — build after Trapezoidal is validated

Jerk-limited profile using `jMax`. Not needed yet — do not start on this
until Trapezoidal is ported, tested standalone on desktop, and physically
validated on one real RC servo joint.

### `TrajectoryGroup` — multi-axis synchronization

**Status: implemented** (`src/TrajectoryGroup.h`/`.cpp`, tested in
`tests/test_trajectory_group.cpp`) — ahead of the original plan below, which
called for waiting until `SCurveProfile` also existed. In practice it only
ever needed `ITrajectoryProfile` (any implementation), so it was built once
`TrapezoidalProfile` was validated rather than waiting on `SCurveProfile`
too. Kept below as the authoritative spec for what it does.

Owns up to 6 `ITrajectoryProfile*`. Plans every axis at minimum time, takes
the max `getDuration()` across axes, then re-plans every axis to that shared
duration (via `targetDuration`) so all axes arrive together.

## Architecture note: move types and the Cartesian path extension

**Status: implemented** (`src/IPathGeometry.h`, `LinePath.h`/`.cpp`,
`ArcPath.h`/`.cpp`, `CartesianMove.h`/`.cpp`, `Vec3.h`, `Quatf.h`, tested in
`tests/test_cartesian.cpp`) — this was originally written as future-scope
design notes before any of it was built; it's kept below as-is because it's
still the correct description of the decomposition actually implemented.

This section corrects a subtle scope assumption and documents the right
decomposition for Cartesian moves, so it isn't re-derived later.

### `TrajectoryGroup` is joint-space only

`TrajectoryGroup` synchronizes arrival *time* across axes, not path *shape*.
Each axis runs its own independent trapezoidal/S-curve profile shaped by its
own distance/vMax/aMax. Even perfectly duration-synchronized axes will not
trace a straight Cartesian line between arbitrary start and end poses unless
the per-axis distances happen to be proportional. Joint-space moves (go from
joint configuration A to joint configuration B) are exactly what
`TrajectoryGroup` is designed for.

### Cartesian moves (lines and arcs) need a different composition

Cartesian moves require two independent, separately-testable pieces — neither
of which is a new `ITrajectoryProfile` implementation:

**`IPathGeometry`** — pure geometry, no timing. Given arc-length distance `s`
traveled along the path, returns position and tangent direction at that point:

```cpp
class IPathGeometry {
public:
    virtual ~IPathGeometry() = default;
    // s in [0, getLength()]
    virtual void evaluate(float s, Vec3& position, Vec3& tangent) const = 0;
    virtual float getLength() const = 0;
};
```

Concrete implementations:
- `LinePath` — straight line between two poses; `getLength()` is Euclidean
  distance.
- `ArcPath` — circular arc defined by center, radius, plane normal,
  start/end angle; `getLength() = radius * |deltaTheta|`.

Both produce geometry only. Neither knows about time, IK, or robots.

**Timing along that geometry reuses `ITrajectoryProfile` unchanged.** One
scalar profile instance runs over `s` from `0` to `path->getLength()`,
exactly as it already does for a single joint. The profile only ever sees a
1-D distance; it is genuinely indifferent to whether the underlying path is
straight or curved.

**`CartesianMove`** (the composition) owns one `IPathGeometry*` + one
`ITrajectoryProfile*`, plus SLERP between start/end orientation (orientation
is independent of which path geometry is used). At each control tick:
1. Profile gives `s(t)` and scalar speed.
2. Path geometry maps `s(t)` → Cartesian position + tangent direction.
3. Cartesian velocity = speed × tangent direction.

### Physical subtlety: centripetal acceleration on curved paths

On a `LinePath`, the profile's `accel` output equals the Cartesian
acceleration magnitude (path is straight, so tangential = total). On an
`ArcPath`, there is also centripetal acceleration (`v² / radius`, perpendicular
to travel direction) arising purely from the path curving, even at constant
path speed. The profile's `accel` is only the *tangential* component. This
becomes relevant once torque feedforward or dynamics exist downstream — a
future layer must not assume `accel(t)` is the full Cartesian acceleration
vector on curved paths. Not relevant now; just don't design it away.

### Downstream IK calling pattern (Motion Device layer, not this repo)

IK calling frequency becomes a per-move-type decision at the Motion Device
layer:
- **Joint-space moves** (`TrajectoryGroup`): IK called once at plan time to
  convert the target Cartesian pose to joint angles q0/qf. Hot path is
  allocation-free scalar arithmetic — fast.
- **Cartesian moves** (`CartesianMove`): IK called every control tick on
  `pose(t)`. IK speed and failure handling (bounded execution time, graceful
  behavior near singularities) become hard requirements, much more demanding
  than the joint-space case.

Nothing here changes the design of this repo. It is recorded so the split
is made correctly when the Motion Device layer is built.

## Scope for v1 (historical — superseded, see below)

This was the original v1 scope, written before any code existed. It's kept
verbatim for history; **the repo has since moved past it** —
`TrajectoryGroup`, `CartesianMove`, `LinePath`, and `ArcPath` are all
implemented and tested (see the status notes on their sections above).
`SCurveProfile` is the one item below still genuinely not started.

- `TrapezoidalProfile` only, single axis. That's the entire deliverable for
  right now — don't build `SCurveProfile` or `TrajectoryGroup` until told to.
- **No mid-motion re-planning.** A move always runs to completion once
  planned; interrupting/blending into a new target mid-move is an explicit
  v2 concern, not v1.
- **No torque feedforward**, per the interface above.

### Actual current scope

- `TrapezoidalProfile`, `TrajectoryGroup`, and the Cartesian path classes
  (`IPathGeometry`, `LinePath`, `ArcPath`, `CartesianMove`) are implemented,
  desktop-tested, and AVR/SAMD/Teensy compile-verified.
- `SCurveProfile` is still not started — per the section above, it's gated
  on `TrapezoidalProfile` being physically validated on real hardware first,
  which hasn't happened yet (see Testing goal below and the Example sketch
  section).
- **No mid-motion re-planning** and **no torque feedforward** are still
  correct — both remain explicitly out of scope, not just for v1.

## MATLAB reference

I have already-validated MATLAB implementations of trapezoidal (and
separately, S-curve) trajectory math, including handling for edge cases
(moves too short to reach `vMax`, zero-distance moves, negative-direction
moves). I will point you to the specific files — **port that math
faithfully rather than re-deriving from generic textbook equations.** Ask me
for the file location before writing the planning math.

## Testing goal

Build a desktop-testable target (no Arduino toolchain required) so the
profile can be validated by feeding it the same `(q0, qf, vMax, aMax, t)`
values used in the MATLAB reference and diffing the output numerically.
Physical validation on real hardware (one RC servo joint, via
Universal-Motor-Interface, driven manually) comes only after the desktop
comparison matches.

## Repo layout

Originally written as a suggestion before any code existed; now describes
what's actually there (see README.md's "Repository layout" section for the
canonical up-to-date version — keep the two in sync):

```
src/                  # the library itself, no build-system assumptions baked in
  ITrajectoryProfile.h
  TrajectoryLimits.h
  TrapezoidalProfile.h / .cpp
  TrajectoryGroup.h / .cpp
  IPathGeometry.h
  LinePath.h / .cpp
  ArcPath.h / .cpp
  CartesianMove.h / .cpp
  Vec3.h
  Quatf.h
tests/                # desktop unit tests (plain C++, no hardware)
examples/              # Arduino sketches (see SimpleTrajectoryDemo below).
                       # Folder must be named exactly "examples" -- this is
                       # an Arduino IDE hard requirement (File > Examples
                       # discovery), not a stylistic convention, since
                       # library.properties makes this a real Arduino
                       # library. Do not rename it.
docs/                  # educational reference (explainer.html)
website/               # content pulled by vishwamaggarwal.com at build
                       # time (article.md, optionally data.md/images/) —
                       # see the website-content entry below
library.properties     # Arduino/PlatformIO library metadata
LICENSE                # MIT
CMakeLists.txt         # desktop build for tests
```

## Original bootstrapping task (done)

The original instruction here was: implement `ITrajectoryProfile` and
`TrapezoidalProfile` to the spec above, plus a desktop test scaffold, porting
the planning math from the MATLAB reference rather than re-deriving it. That
happened; `TrapezoidalProfile` is implemented, desktop-tested against the
MATLAB reference cases, and the repo has since grown well past it (see
"Actual current scope" above). Left here for history, not as an active task.

## Example sketch

`examples/SimpleTrajectoryDemo` exists: one `TrapezoidalProfile` driving a
`SimulatedMotor` (a struct local to the sketch that just remembers the
commanded position — no real actuator, no dependency on
Universal-Motor-Interface). Compile-verified via `arduino-cli` on Uno, Nano,
Mega, Leonardo, Nano 33 IoT (SAMD21), Teensy 4.0, and Teensy 3.2. This
satisfies "Arduino example sketch" in the README roadmap, but **not** the
"physical validation on one real RC servo joint" goal in Testing goal above
— that still requires real hardware and Universal-Motor-Interface at the
Motion Device layer, and hasn't been done.

## Website content moved into a website/ folder (2026-08-22)

`article.md` (already live at
[vishwamaggarwal.com/articles/universal-trajectory-interface/](https://vishwamaggarwal.com/articles/universal-trajectory-interface/),
`draft: false`) moved to `website/article.md`, matching a new site-wide
convention: every project repo the website pulls from keeps everything
it feeds to the site under one `website/` folder instead of loose at
repo root (see the website repo's own `CLAUDE.md`, and the
Servo-Calibrator repo's matching entry from the same session — that
repo also had a tool landing page and web app to relocate, this one
doesn't). No `data.md` here yet (never authored), so nothing else to
move. The website repo's `content.config.ts` was updated to fetch from
the new path in the same session — **both sides need to land together**:
until this repo's move is merged, the website's next build looks for
`website/article.md` here and won't find it (non-fatal — the loader
just warns and skips, same as a missing token — but the live article
would silently drop off the site until this merges).

Also added `website/` to both this file's and README's "Repo layout"
diagrams, which had never listed `article.md` even before this move —
a pre-existing gap, not something this change introduced, just noticed
and fixed while touching the same section.
