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

A second tab will let you flash a small demo sketch onto an Arduino over USB
— no toolchain, no library installs — and then command moves and watch your
own servo run them, with the board's streamed trace drawn against the
simulated one. That part is still being built; the simulator above works
today and needs no hardware at all.

When it lands it will want Chrome or Edge on desktop (Web Serial is not
available in Firefox or Safari), an ATmega328P board — an Uno, or a Nano of
either bootloader vintage — and a hobby servo on its own 5 V supply.

If you want to run the library on hardware right now, the repo ships four
example sketches, three of which need nothing but a board.
