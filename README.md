# Universal Trajectory Interface

A standalone, platform-agnostic C++ library for real-time trajectory (motion profile) generation.

Given a start position, a target position, and kinematic limits (max velocity, max acceleration), it produces smooth, bounded position/velocity/acceleration setpoints on demand — the piece that turns a step change in target into a properly-shaped motion profile.

No hardware dependencies. Runs identically on Arduino, Teensy, and Linux x86.

---

## What it does

- **Trapezoidal velocity profile** — classic accel / cruise / decel shape, with automatic triangular fallback for short moves that never reach `vMax`.
- **Multi-axis synchronization** (`TrajectoryGroup`) — plans up to 6 independent axes so they all arrive at their targets at the same time, regardless of individual distances.
- **Cartesian path moves** (`CartesianMove`) — drives a tool along a straight line or circular arc in 3D space, with smooth orientation interpolation (SLERP) between start and end orientations.

---

## Platform compatibility

| Class | Arduino Uno (ATmega328P) | Teensy 3.x / 4.x | Linux x86 / EtherCAT |
|---|---|---|---|
| `TrapezoidalProfile` | ✅ Full support | ✅ Full support | ✅ Full support |
| `TrajectoryGroup` | ✅ Full support | ✅ Full support | ✅ Full support |
| `CartesianMove` + `LinePath` | ⚠️ Works, but SLERP is slow (no FPU) | ✅ Full support | ✅ Full support |
| `CartesianMove` + `ArcPath` | ⚠️ Works, but too slow for real-time use | ✅ Full support | ✅ Full support |

### Arduino Uno notes

The ATmega328P has no hardware floating-point unit. All `float` math is emulated in software by the compiler, which makes trigonometric functions expensive:

| Function | Cost on 16 MHz AVR |
|---|---|
| `sqrtf()` | ~50 µs |
| `sinf()` / `cosf()` | ~100–200 µs each |
| `acosf()` | ~200–400 µs |

**`TrapezoidalProfile` and `TrajectoryGroup`** use only `sqrtf()` (once, at plan time) and simple arithmetic in the real-time `evaluate()` path. They are fully suitable for Uno.

**`CartesianMove` + `ArcPath`** calls `sinf()` and `cosf()` every control tick for position, plus `acosf()` and additional `sinf()` calls for orientation SLERP — adding up to ~1–1.4 ms per `evaluate()` call. At RC servo rates (50 Hz), this technically fits in the 20 ms budget, but leaves almost no headroom and is not recommended.

**Recommended approach for Uno:** use `TrapezoidalProfile` and `TrajectoryGroup` for joint-space moves. Compute inverse kinematics once at plan time to convert your target Cartesian pose into joint angles, then feed those joint angles into `TrajectoryGroup`. This is fast, deterministic, and leaves the CPU free for communication and servo output.

For Cartesian path following on embedded hardware, a Teensy 4.0/4.1 (Cortex-M7 with FPU) is a much better fit.

### Memory (Arduino Uno)

- `TrapezoidalProfile` instance: ~38 bytes SRAM
- `TrajectoryGroup` (6-axis): ~18 bytes SRAM + 6 profile pointers
- A 4-DOF `TrajectoryGroup` with 4 `TrapezoidalProfile` instances: ~170 bytes total — feasible within the Uno's 2 KB SRAM

### Compiler requirement

Requires C++11 or later. Arduino IDE 1.8+ (ships avr-gcc 7.3) is supported. Older IDE versions (pre-1.6.6) will fail to compile due to missing C++11 features.

---

## Building and testing on desktop (Linux / Windows with MSVC)

```bash
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
#include "CartesianMove.h"        // Cartesian path moves (Teensy/x86 recommended)
```

---

## Repository layout

```
src/                  # library source — no build-system or hardware dependencies
  ITrajectoryProfile.h
  TrapezoidalProfile.h / .cpp
  TrajectoryGroup.h / .cpp
  IPathGeometry.h
  LinePath.h / .cpp
  ArcPath.h / .cpp
  CartesianMove.h / .cpp
  Vec3.h
  Quatf.h
tests/                # desktop unit tests (plain C++, no hardware)
docs/                 # educational reference (explainer.html)
CMakeLists.txt
library.properties    # Arduino/PlatformIO metadata
```

---

## Roadmap

- [ ] `SCurveProfile` — jerk-limited (S-curve) profile using `jMax`. Will be implemented once `TrapezoidalProfile` has been physically validated on hardware.
- [ ] Arduino example sketch — single RC servo joint driven by `TrapezoidalProfile`.
- [ ] Blend / re-plan — interrupt a move mid-motion and transition smoothly into a new target.

---

## Related

**Universal Motor Interface** — hardware abstraction layer for motors (RC servo, ODrive over CAN, simulated). This library has no dependency on it; they are composed at a higher "Motion Device" layer that lives in neither repo.

---

## License

MIT
