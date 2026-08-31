---
title: "Trajectory Lab"
description: "An interactive playground for trapezoidal and S-curve motion profiles, running the actual C++ library compiled to WebAssembly rather than a JavaScript imitation of it."
tags: ["WebAssembly", "Motion Control", "Robotics"]
status: active
repo: "https://github.com/vishwam-aggarwal/Universal-Trajectory-Interface"
draft: true
---

Tell a motor to go from here to there and it will try to get there
instantly. What stops that from tearing the machine apart is a *motion
profile* — a curve that decides how position, velocity and acceleration
evolve over the move, given limits the hardware can actually honour.
[Universal-Trajectory-Interface](https://github.com/vishwam-aggarwal/Universal-Trajectory-Interface)
is a small C++ library that generates those curves. Trajectory Lab is that
library, running in your browser.

Drag a jerk limit and watch the corner round off the acceleration trace.
Shorten a move until the trapezoid collapses into a triangle because it
never gets a chance to reach full speed. Put all three profile types on the
same axes and see what each one costs.

## The curves here are not a re-implementation

This matters more than it might sound. The obvious way to build a page like
this is to rewrite the profile equations in JavaScript — at which point the
page and the library are two different pieces of software that agree only
until one of them changes.

Instead, `TrapezoidalProfile.cpp`, `SCurveProfile.cpp` and
`JerkPercentProfile.cpp` are compiled to WebAssembly with Emscripten and
called in single-precision `float`, exactly as an 8-bit AVR would. What you
see plotted is the same arithmetic the microcontroller executes, down to the
rounding. Every build is checked sample-by-sample against an independent
double-precision transcription of the original MATLAB reference — the same
check that held the library's output on real hardware to within 0.001°.

## The three profiles

**Trapezoidal** is the classic: accelerate hard at a constant rate, cruise
at `vMax`, decelerate hard. Fastest possible move inside the limits, but
acceleration steps instantly between values, and that step is what a
drivetrain feels as a jolt.

**S-curve** limits *jerk* — the rate of change of acceleration — so
acceleration ramps instead of stepping. Seven segments instead of three. It
pays for that smoothness in time: the same move takes longer.

**Jerk %** makes the opposite trade. Instead of an absolute jerk limit, you
give it a percentage of the acceleration phase to spend ramping, and it
keeps the duration *exactly equal to the trapezoid's* — paying for
smoothness with acceleration headroom rather than time. The catch, which the
tool shows you at runtime: it deliberately overshoots the `aMax` you give
it, by a factor of 2/(2−p). At the default 66% that is 1.49×, and at 100% it
is 2×. If your acceleration figure is a real actuator limit rather than a
nominal one, derate it first.

The three preset moves are the ones the library's bundled example sketches
run, so the durations the page reports — 1.50 s, 1.75 s and 1.50 s for the
same 0 → 90 move — are the numbers you can reproduce on a board.

## Units

There aren't any, deliberately. The library works in plain `float`s, so the
numbers are degrees, millimetres, revolutions or anything else you use
consistently. A CNC axis, a gimbal and a robot joint are the same problem to
it.

## Driving a real servo

The second tab flashes a small demo sketch onto your own Arduino straight
from the page — no toolchain, no IDE, no library installs — then lets you
command moves and watch the servo run them. The board plans and executes
each move itself and streams back what it commanded; the page draws that on
top of the same profile computed in the browser, so you can see an 8-bit
microcontroller and a WebAssembly build agreeing on the same curve.

You'll need:

- **Chrome, Edge or Opera on desktop.** Flashing and serial both go through
  the Web Serial API, which Firefox ships only in Nightly behind a flag and
  Safari doesn't implement at all.
- **An ATmega328P board** — an Uno, or a Nano of either bootloader vintage.
  All three take the same image; only the upload settings differ. Nano
  clones are split between two bootloaders and nothing on the port says
  which, so if flashing won't sync, pick the other Nano option.
- **A hobby servo**, signal on pin **A3**, powered from its own 4.8–6 V
  supply with a common ground. A servo's stall current will brown out a
  USB-powered board mid-move, which looks like the profile glitching.

### Why it's in microseconds

The charts on that tab are labelled in microseconds of pulse width, not
degrees, and that's deliberate. Without an encoder there's no way to know
what a pulse width does to *your* servo, and the usual assumption is wrong:
"1000–2000 µs is 180°" is a convention, not a specification. The servo on
this library's bench rig moves 0.0888 °/µs — that range spans 89.7°, about
half what everyone assumes.

That matters more than it sounds. Commanding as though the range were 180°
produces a curve with the right shape at the wrong amplitude, which reads
like the servo failing to track rather than like a calibration error. It
cost real time to diagnose once already, so this demo doesn't repeat it.
There's a field for your own measured °/µs if you have it, and the page
labels the result as your figure rather than as a measurement.

If you'd rather not use the browser at all, the repo ships five example
sketches — four of them need nothing but a board.
