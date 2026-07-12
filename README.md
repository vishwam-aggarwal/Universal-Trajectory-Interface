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

| Class | Arduino Uno (ATmega328P) | Arduino Nano 33 IoT (SAMD21) | Teensy 3.x / 4.x | Linux x86 / EtherCAT |
|---|---|---|---|---|
| `TrapezoidalProfile` | ✅ Full support | ✅ Full support | ✅ Full support | ✅ Full support |
| `TrajectoryGroup` | ✅ Full support | ✅ Full support | ✅ Full support | ✅ Full support |
| `CartesianMove` + `LinePath` | ⚠️ Works, slow SLERP | ✅ Full support | ✅ Full support | ✅ Full support |
| `CartesianMove` + `ArcPath` | ⚠️ Too slow for real-time | ✅ Full support | ✅ Full support | ✅ Full support |

### Trig function cost by platform

All `float` math on processors without an FPU is emulated in software. The difference between platforms is how efficiently that software runs:

| Function | Arduino Uno (8-bit AVR, 16 MHz) | Arduino Nano 33 IoT (32-bit ARM, 48 MHz) |
|---|---|---|
| `sqrtf()` | ~50 µs | ~1–3 µs |
| `sinf()` / `cosf()` | ~100–200 µs each | ~2–5 µs each |
| `acosf()` | ~200–400 µs | ~3–8 µs |

Neither processor has a hardware FPU, but the Nano 33 IoT's 32-bit Cortex-M0+ runs ARM's optimized soft-float library — roughly 20–50× faster than AVR soft-float. `CartesianMove` + `ArcPath` costs ~5–10 µs per tick on the Nano 33 IoT, which is fully real-time capable at 1 kHz and well within any servo update rate.

### Arduino Uno notes

**`TrapezoidalProfile` and `TrajectoryGroup`** use only `sqrtf()` once at plan time and plain arithmetic in `evaluate()`. They are fully suitable for Uno.

**`CartesianMove` + `ArcPath`** calls `sinf()` and `cosf()` every control tick for position, plus `acosf()` and additional `sinf()` calls for orientation SLERP — totalling ~1–1.4 ms per `evaluate()` call. At RC servo rates (50 Hz) this technically fits in the 20 ms budget, but leaves almost no headroom and is not recommended.

**Recommended approach for Uno:** use `TrapezoidalProfile` and `TrajectoryGroup` for joint-space moves. Compute inverse kinematics once at plan time to convert your target Cartesian pose into joint angles, then feed those angles into `TrajectoryGroup`. This is fast, deterministic, and leaves the CPU free for communication and servo output.

### Arduino Nano 33 IoT notes

All classes are fully supported. The SAMD21's 32-bit ARM core and ARM soft-float library make trig functions fast enough for real-time Cartesian path following at typical servo update rates. 32 KB SRAM and 256 KB flash leave ample headroom.

### Memory

| Board | SRAM | 4-DOF TrajectoryGroup footprint | Headroom |
|---|---|---|---|
| Arduino Uno (ATmega328P) | 2 KB | ~170 bytes | Tight — feasible, leave room for sketch and runtime (~400 bytes) |
| Arduino Nano 33 IoT (SAMD21) | 32 KB | ~170 bytes | Comfortable |
| Teensy 4.x (iMXRT1062) | 1 MB | ~170 bytes | No concern |

### Compiler requirement

Requires C++11 or later. Arduino IDE 1.8+ is supported for all ARM-based boards. For AVR (Uno), Arduino IDE 1.8+ ships avr-gcc 7.3 with C++11 support. Older IDE versions (pre-1.6.6) will fail to compile.

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
#include "CartesianMove.h"        // Cartesian path moves (Nano 33 IoT / Teensy / x86)
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
