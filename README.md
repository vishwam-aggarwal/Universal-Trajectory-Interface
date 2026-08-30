# Universal Trajectory Interface

[![build](https://github.com/vishwam-aggarwal/Universal-Trajectory-Interface/actions/workflows/build.yml/badge.svg)](https://github.com/vishwam-aggarwal/Universal-Trajectory-Interface/actions/workflows/build.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A standalone, platform-agnostic C++ library for real-time trajectory (motion profile) generation.

Given a start position, a target position, and kinematic limits (max velocity, max acceleration), it produces smooth, bounded position/velocity/acceleration setpoints on demand — the piece that turns a step change in target into a properly-shaped motion profile.

No hardware dependencies. Runs identically on Arduino, Teensy, and Linux x86.

---

## What it does

- **Trapezoidal velocity profile** — classic accel / cruise / decel shape, with automatic triangular fallback for short moves that never reach `vMax`.
- **Multi-axis synchronization** (`TrajectoryGroup`) — plans up to 6 independent axes so they all arrive at their targets at the same time, regardless of individual distances.
- **Cartesian path moves** (`CartesianMove`) — drives a tool along a straight line or circular arc in 3D space, with smooth orientation interpolation (SLERP) between start and end orientations.
- **S-curve (jerk-limited) profile** (`SCurveProfile`) — 7-segment jerk-limited shape driven by `TrajectoryLimits::jMax`, so acceleration ramps instead of stepping. Handles the degenerate cases (too short to reach `vMax`, too short to reach `aMax`, both) internally.
- **Jerk-percent S-curve** (`JerkPercentProfile`) — the same 7-segment shape, but specified as a *percentage* of the acceleration phase spent ramping, against a **nominal** acceleration limit. Keeps the move's duration exactly equal to the trapezoidal duration, buying smoothness with acceleration headroom instead of time. Note this means peak acceleration **exceeds** the `aMax` you pass in — by 1.49× at 66%, and 2× at 100%.

---

## Usage: planning a move

Every profile (and `CartesianMove`'s underlying profile) is driven through `ITrajectoryProfile`:

```cpp
class ITrajectoryProfile {
public:
    // Full signature -- targetDuration = 0 plans for minimum time given the
    // limits; > 0 time-dilates the move to exactly that duration (used by
    // TrajectoryGroup to synchronize multiple axes).
    virtual bool plan(float q0, float qf, const TrajectoryLimits& limits,
                       float targetDuration) = 0;

    // 3-arg convenience overload -- targetDuration defaults to 0.0f.
    bool plan(float q0, float qf, const TrajectoryLimits& limits);

    virtual bool evaluate(float t, float& pos, float& vel, float& accel) const = 0;
    virtual float getDuration() const = 0;
};
```

Both forms work whether you're going through a base pointer/reference or holding a concrete profile type directly:

```cpp
TrapezoidalProfile p;
p.plan(q0, qf, limits);              // targetDuration defaults to 0.0f
p.plan(q0, qf, limits, 1.5f);        // explicit duration

ITrajectoryProfile* base = &p;
base->plan(q0, qf, limits);          // same defaulting, through a base pointer
```

That works without falling into the usual "default argument on a `virtual` function" trap: a default value written directly on a virtual signature resolves off the **static** type of the call expression, not the object's actual runtime type — so a pointer declared as the base class could silently use a different default than one declared as the derived class, for the identical object. Instead, `targetDuration`'s default lives on a genuinely separate, non-virtual 3-arg overload in `ITrajectoryProfile` that forwards to the pure-virtual 4-arg one — the forwarding call still dispatches virtually, so it always reaches whatever concrete profile is actually behind it, correctly, regardless of how it was called.

The one thing this requires from every concrete implementation: declaring your own `plan(...)` override **hides** every base-class overload of that name from lookup on your concrete type (ordinary C++ member-hiding — lookup stops at the first class scope where the name appears at all, before overload resolution even runs), which would otherwise make the 3-arg form uncallable except through a base reference. `TrapezoidalProfile` pulls it back into scope with one line alongside its override:

```cpp
using ITrajectoryProfile::plan;
```

`SCurveProfile` and `JerkPercentProfile` carry the same line, and any future `ITrajectoryProfile` implementation needs it too for the 3-arg convenience form to work when called on its own concrete type.

---

## Platform compatibility

The library compiles and runs on any platform with a C++11 compiler. The limiting factor for `CartesianMove` (which calls `sinf`, `cosf`, and `acosf` every control tick) is whether the processor has a hardware FPU.

**How to find your board's architecture:** look up your board's MCU in its datasheet or the Arduino board manager, then find it in the table below.

---

### 8-bit AVR — no FPU
**Boards:** Arduino Uno, Nano, Pro Mini (ATmega328P) · Arduino Mega 2560 (ATmega2560) · Arduino Leonardo, Micro (ATmega32U4)

| Class | Support |
|---|---|
| `TrapezoidalProfile` | ✅ Full support |
| `TrajectoryGroup` | ✅ Full support |
| `CartesianMove` + `LinePath` | ⚠️ Works, but SLERP adds ~500–1000 µs per tick |
| `CartesianMove` + `ArcPath` | ⚠️ Works, but ~1–1.4 ms per tick — not recommended for real-time |

All float math is software-emulated. `sinf()`/`cosf()` cost ~100–200 µs each at 16 MHz; `acosf()` costs ~200–400 µs. `TrapezoidalProfile::evaluate()` uses only arithmetic and is fast on any platform.

**Toolchain note:** avr-gcc ships no C++ standard library at all — no `<cmath>`, no `<vector>`, nothing beyond the plain C runtime. The library uses `<math.h>` internally for this reason (functionally identical to `<cmath>` here, since no call in this codebase is `std::`-qualified). This is transparent to callers; it's mentioned only because it's the reason AVR compiles at all.

**Recommended approach on AVR:** use `TrapezoidalProfile` + `TrajectoryGroup` for joint-space moves. Compute inverse kinematics once at plan time to convert your target pose into joint angles, then feed those angles into `TrajectoryGroup`. This leaves the CPU free for servo output and communication.

SRAM is the tighter constraint: ATmega328P has 2 KB total. A 4-axis `TrajectoryGroup` uses ~170 bytes, leaving ~1.8 KB for the Arduino runtime and your sketch.

---

### 32-bit ARM Cortex-M0 / M0+ / M3 — no FPU
**Boards:** Arduino Zero, Nano 33 IoT, MKR series (SAMD21, Cortex-M0+) · Arduino Due (SAM3X8E, Cortex-M3) · Arduino Nano RP2040 Connect, Raspberry Pi Pico (RP2040, Cortex-M0+)

| Class | Support |
|---|---|
| `TrapezoidalProfile` | ✅ Full support |
| `TrajectoryGroup` | ✅ Full support |
| `CartesianMove` + `LinePath` | ✅ Full support |
| `CartesianMove` + `ArcPath` | ✅ Full support |

No hardware FPU, but ARM's optimized soft-float library runs 20–50× faster than AVR soft-float because these are 32-bit cores. `sinf()`/`cosf()` cost ~2–5 µs; `acosf()` costs ~3–8 µs. `CartesianMove` + `ArcPath` + SLERP totals ~15–30 µs per tick — fully real-time at 1 kHz. SRAM ranges from 32 KB (SAMD21) to 264 KB (RP2040); no memory concerns.

---

### 32-bit ARM Cortex-M4F / M7 — hardware FPU
**Boards:** Teensy 3.2, 3.5, 3.6 (Cortex-M4F) · Teensy 4.0, 4.1 (Cortex-M7) · Arduino Nano 33 BLE / BLE Sense (nRF52840, Cortex-M4F) · most STM32F4 / STM32H7 based boards

| Class | Support |
|---|---|
| All classes | ✅ Full support, hardware-accelerated float |

The hardware FPU executes single-precision float in 1–2 clock cycles. All trig functions cost well under 1 µs. Every class in this library is fully real-time capable at several kHz on these boards.

---

### ESP32 — hardware FPU (Xtensa LX6/LX7 core)
**Boards:** ESP32 DevKit and variants · ESP32-S2 · ESP32-S3

| Class | Support |
|---|---|
| All classes | ✅ Full support |

The ESP32 and ESP32-S series include a hardware single-precision FPU. All classes are fully supported. Note: the **ESP32-C series** (RISC-V core, no FPU) behaves more like the Cortex-M0+ tier — still fine for all classes, just without hardware float acceleration.

---

### Linux / macOS / Windows (x86-64)
All classes fully supported. This is the primary target for development, unit-testing, and validation against MATLAB reference traces before deploying to hardware.

---

### Compiler requirement

Requires C++11 or later. Arduino IDE 1.8+ ships with C++11-capable compilers for all supported architectures. Older IDE versions (pre-1.6.6) will fail to compile due to missing C++11 features (`= default`, struct member initializers).

---

## Building and testing on desktop (Linux / Windows with MSVC)

The `extern/Universal-Device-Interface` submodule must be present — clone with
`--recurse-submodules`, or run `git submodule update --init` in an existing
checkout.

```bash
git submodule update --init
mkdir build && cd build
cmake ..
cmake --build .
ctest --output-on-failure
```

On Windows with MSVC, open a Developer Command Prompt and use:

```bat
cmake -S . -B build -G "NMake Makefiles"
cmake --build build
cd build && ctest --output-on-failure
```

---

## Arduino / PlatformIO

Copy the `src/` folder into your project or add this repo as a library. Include only what you need:

```cpp
#include "TrapezoidalProfile.h"   // single-axis profile
#include "TrajectoryGroup.h"      // multi-axis sync (joint space)
#include "CartesianMove.h"        // Cartesian path moves (Nano 33 IoT / Teensy / x86)
```

See `examples/SimpleTrajectoryDemo` for a minimal, dependency-free sketch: one `TrapezoidalProfile` driving a stand-in `SimulatedMotor` (just remembers the commanded position — no real actuator, no external library needed). Compile-verified on Uno, Nano, Mega, Leonardo, Nano 33 IoT (SAMD21), Teensy 4.0, and Teensy 3.2.

`examples/SCurveTrajectoryDemo` and `examples/JerkPercentTrajectoryDemo` are the same sketch with `SCurveProfile` and `JerkPercentProfile` swapped in. All three run the **same move (0 → 90) with the same nominal limits**, so they can be compared directly:

| Sketch | Duration | Peak acceleration | Acceleration shape |
|---|---|---|---|
| `SimpleTrajectoryDemo` | 1.50 s | 180 | Steps instantly |
| `SCurveTrajectoryDemo` | 1.75 s | 180 | Ramps at `jMax` |
| `JerkPercentTrajectoryDemo` | 1.50 s | 269 | Ramps, but harder |

That contrast is the point: `SCurveProfile` pays for smooth acceleration with time, `JerkPercentProfile` pays for it with acceleration headroom. The latter prints its derived limits at startup so the overshoot above nominal `aMax` is visible.

---

## Repository layout

```
src/                  # library source — no build-system or hardware dependencies
  ITrajectoryProfile.h
  TrapezoidalProfile.h / .cpp
  SCurveProfile.h / .cpp
  JerkPercentProfile.h / .cpp
  TrajectoryGroup.h / .cpp
  IPathGeometry.h
  LinePath.h / .cpp
  ArcPath.h / .cpp
  CartesianMove.h / .cpp
  Vec3.h
  Quatf.h
tests/                # desktop unit tests (plain C++, no hardware)
examples/             # Arduino sketches (no hardware required to compile/run)
  SimpleTrajectoryDemo/
  SCurveTrajectoryDemo/
  JerkPercentTrajectoryDemo/
  HardwareValidation/   # needs a servo + AS5600 encoder
tools/                # host-side scripts (Python) for the hardware capture
  reference_profiles.py
  capture_validation.py
  plot_validation.py
docs/                 # educational reference (explainer.html)
website/              # content pulled by vishwamaggarwal.com at build
                      # time (article.md, optionally data.md/images/) —
                      # see CLAUDE.md's website-content entry
CMakeLists.txt
library.properties    # Arduino/PlatformIO metadata
```

---

## Validating against real hardware

`examples/HardwareValidation` runs the same 90&deg; move under all three
profiles and streams the commanded *and* measured angle over serial. It needs
a rig: a hobby servo on pin A3, an AS5600 magnetic encoder on I2C reading the
servo horn, and a separate 5&nbsp;V supply for the servo (a servo under load
will brown out a USB-powered board mid-capture).

```
python tools/capture_validation.py --port COM4 --out run.csv
python tools/plot_validation.py run.csv
```

`tools/reference_profiles.py` is a second, independent implementation of all
three profiles in double-precision Python, transcribed from the same MATLAB
the C++ was ported from. The plotting script compares the board's actual
commanded output against it &mdash; so the check is C++-on-target versus the
reference math, not the board against itself. It then plots measured position,
tracking error, and the settling tail after each move nominally ends.

## Roadmap

- [x] `SCurveProfile` — jerk-limited (S-curve) profile using `jMax`, ported from the same MATLAB reference as `TrapezoidalProfile` and desktop-tested against it (354 checks). Not yet validated on physical hardware.
- [x] Arduino example sketch — `TrapezoidalProfile` driving a simulated motor; compiles on AVR, SAMD, and Teensy. Physical validation against a real RC servo joint (via Universal-Motor-Interface, composed at the Motion Device layer) is still open.
- [x] `JerkPercentProfile` — jerk specified as a percentage of the acceleration phase against a nominal `aMax`, preserving trapezoidal duration. Ported from the MATLAB reference and desktop-tested against it (283 checks). Not yet validated on physical hardware.
- [ ] Blend / re-plan — interrupt a move mid-motion and transition smoothly into a new target.

---

## State, status, and errors on the planners

`TrajectoryGroup` and `CartesianMove` derive from
[`IDevice`](https://github.com/vishwam-aggarwal/Universal-Device-Interface), so
a failed `plan()` explains itself instead of returning a bare `false`:

```cpp
IDevice::setGlobalErrorSink(printError);   // once, for the whole stack

TrajectoryGroup group("jointGroup");
if (!group.plan(profiles, q0, qf, limits, 3)) {
    // printError already fired with layer="TrajectoryGroup", source="jointGroup"
    Serial.println(group.getErrorString(group.getError()));
}
```

| | Meaning |
|---|---|
| `getState()` | `ERRORED` after a failed `plan()`, `IDLE` otherwise. **Never `BUSY`** — these classes are stateless with respect to time (you own `t`; `evaluate()` is `const`), so they cannot honestly know whether a move is in progress. |
| `getStatus()` | `STATUS_NONE` (no plan loaded) or `STATUS_PLANNED`. |
| `getError()` | `TrajectoryGroup`: `ERR_INVALID_AXIS_COUNT`, `ERR_NULL_PROFILE`, `ERR_AXIS_PLAN_FAILED`. `CartesianMove`: `ERR_NULL_PATH`, `ERR_NULL_PROFILE`, `ERR_PROFILE_PLAN_FAILED`. |
| `isOnline()` | Always `true` — pure computation has nothing to be offline from. |

**`ITrajectoryProfile`, `TrapezoidalProfile`, `IPathGeometry`, `LinePath` and
`ArcPath` are deliberately *not* `IDevice`s.** They're stateless per-axis math
where `plan()`'s bool already says everything true; only the composing planners
gained the contract.

`evaluate()` on both planners is unchanged and still non-virtual, so the
real-time hot path picks up no dynamic dispatch, allocation, or exceptions —
the only cost is a vtable pointer per planner object.

### Two `plan()` bugs fixed alongside the retrofit

- `TrajectoryGroup::plan()` **ignored every per-axis `plan()` return value**, so
  it could report success while one axis silently never moved (reachable with a
  zero or NaN `vMax`/`aMax`, which `TrapezoidalProfile` rejects). Both planning
  phases are now checked.
- `CartesianMove::plan()` **dereferenced its `path`/`profile` arguments without
  a null check**, crashing instead of reporting.

Both now validate before storing anything and leave no plan loaded on failure,
so `evaluate()` is inert rather than acting on a half-configured planner.

---

## Related

**Universal Motor Interface** — hardware abstraction layer for motors (RC servo, ODrive over CAN, simulated). This library has no dependency on it; they are composed at a higher "Motion Device" layer that lives in neither repo.

---

## License

MIT
