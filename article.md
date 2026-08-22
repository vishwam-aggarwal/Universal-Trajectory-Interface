---
title: "Your Motor Doesn't Know How To Get There"
description: "A target angle is not a motion plan. Why real motors need a shape for the move between two points, not just the destination — and the tiny, portable library that computes it in real time."
pubDate: 2026-08-22
tags: ["Robotics", "Embedded", "Motion Control"]
draft: false
---

`servo.write(90)`. One line, and the servo goes to 90°. It works — right up until it doesn't: a wobbly arm that overshoots and settles late, a stepper that skips under load, a gripper that arrives at its target hard enough to rattle the whole frame. The servo isn't broken. Nobody ever told it *how* to get to 90° — only that it should.

That gap — between naming a destination and actually planning the motion that gets there — is what a trajectory generator solves. It's a small enough idea to explain without equations, and it's one of the layers in [Universal Interface Stack](/projects/universal-interface-stack/), a hardware-agnostic C++ control stack for robots — the layer that sits above motor and encoder drivers and decides what curve they should be tracking in the first place. This piece covers the part of it that plans motion for a single motor, and touches on what happens once a robot has more than one.

## A target is not a motion plan

No real motor can teleport. Between "start" and "target," something happens — some curve gets traced through position, velocity, and acceleration over time. The only question is whether that curve was *decided*, or whether it just happened, shaped by whatever the motor's own internal control loop does when it's asked to close a large error as fast as it can.

<div class="chart-figure">
<p class="chart-title">Two ways to arrive at the same 90&#176;</p>
<svg viewBox="0 0 680 260" role="img" aria-label="Position vs time chart comparing a naive instant step command against a smooth trapezoidal trajectory. The step jumps from 0 to 90 degrees almost instantly at t=0, while the trajectory reaches 90 degrees smoothly at t=0.8 seconds">
<g stroke="var(--border)" stroke-width="1">
<line x1="48" y1="220" x2="656" y2="220" />
<line x1="48" y1="170" x2="656" y2="170" />
<line x1="48" y1="120" x2="656" y2="120" />
<line x1="48" y1="70" x2="656" y2="70" />
<line x1="48" y1="20" x2="656" y2="20" />
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">
<text x="40" y="224">0&#176;</text>
<text x="40" y="174">25&#176;</text>
<text x="40" y="124">50&#176;</text>
<text x="40" y="74">75&#176;</text>
<text x="40" y="24">100&#176;</text>
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">
<text x="48" y="240">0.0s</text>
<text x="176" y="240">0.2s</text>
<text x="304" y="240">0.4s</text>
<text x="432" y="240">0.6s</text>
<text x="560" y="240">0.8s</text>
</g>
<path d="M 48.00 220.00 L 60.80 40.00 L 656.00 40.00" fill="none" stroke="var(--series-2)" stroke-width="2" stroke-dasharray="6 4"/>
<path d="M 48.00 220.00 L 58.24 219.85 L 68.48 219.39 L 78.72 218.62 L 88.96 217.54 L 99.20 216.16 L 109.44 214.47 L 119.68 212.47 L 129.92 210.17 L 140.16 207.56 L 150.40 204.64 L 160.64 201.41 L 170.88 197.88 L 181.12 194.04 L 191.36 189.89 L 201.60 185.44 L 211.84 180.68 L 222.08 175.61 L 232.32 170.23 L 242.56 164.56 L 252.80 158.80 L 263.04 153.04 L 273.28 147.28 L 283.52 141.52 L 293.76 135.76 L 304.00 130.00 L 314.24 124.24 L 324.48 118.48 L 334.72 112.72 L 344.96 106.96 L 355.20 101.20 L 365.44 95.44 L 375.68 89.77 L 385.92 84.39 L 396.16 79.32 L 406.40 74.56 L 416.64 70.11 L 426.88 65.96 L 437.12 62.12 L 447.36 58.59 L 457.60 55.36 L 467.84 52.44 L 478.08 49.83 L 488.32 47.53 L 498.56 45.53 L 508.80 43.84 L 519.04 42.46 L 529.28 41.38 L 539.52 40.61 L 549.76 40.15 L 560.00 40.00 L 656.00 40.00" fill="none" stroke="var(--series-1)" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round"/>
<circle cx="60.80" cy="40.00" r="4" fill="var(--bg-raised)" stroke="var(--series-2)" stroke-width="2.5"/>
<text x="70" y="60" font-family="var(--font-mono)" font-size="11" fill="var(--series-2)" text-anchor="start">commanded instantly &#8212; no real motor can do this</text>
<circle cx="560.00" cy="40.00" r="4.5" fill="var(--bg-raised)" stroke="var(--series-1)" stroke-width="2.5"/>
<text x="552" y="30" font-family="var(--font-mono)" font-size="11" fill="var(--text)" font-weight="600" text-anchor="end">arrives at rest, 0.8s</text>
</svg>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--series-1);"></span>Trajectory (TrapezoidalProfile)</span>
<span><span class="swatch" style="background: var(--series-2);"></span>Naive step command</span>
</div>
<p class="chart-caption">0&#176; &#8594; 90&#176;, vMax = 180&#176;/s, aMax = 600&#176;/s&#178;. The step asks for infinite acceleration at t=0 &#8212; physically impossible, so in practice something else (a firmware ramp you don't control, or raw current into a stalled load) decides the real shape for you. The trajectory decides it up front instead.</p>
</div>

The dashed line isn't a real motion — it's a stand-in for "I didn't plan this." Whatever a servo or stepper actually does when you hand it a bare target, it isn't that vertical jump; it's *some* curve the hardware improvises, and you don't get to choose its shape. The solid line is the alternative: a curve computed in advance, from two numbers you already know about your hardware — how fast it can go, and how fast it can change speed.

## Accelerate, cruise, decelerate

Those two numbers are the whole interface. A **trapezoidal profile** — the shape this library is named after — spends a move in three phases: speed up at the motor's maximum acceleration, hold the maximum safe speed as long as there's distance left, then slow down in time to land exactly on target at zero velocity.

<div class="chart-figure">
<p class="chart-title">Velocity and acceleration during that same move</p>
<div class="subplot-grid">
<div class="subplot">
<svg viewBox="0 0 300 250" role="img" aria-label="Velocity vs time: ramps linearly from 0 to 180 degrees per second over 0.3 seconds, holds 180 for 0.2 seconds, then ramps back to 0 over the final 0.3 seconds -- a trapezoid shape">
<text x="40" y="10" font-family="var(--font-mono)" font-size="10.5" font-weight="600" fill="var(--text)" text-anchor="start">Velocity</text>
<line x1="40" y1="210" x2="290" y2="210" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="213" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">0</text>
<line x1="40" y1="162.5" x2="290" y2="162.5" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="165.5" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">50</text>
<line x1="40" y1="115" x2="290" y2="115" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="118" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">100</text>
<line x1="40" y1="67.5" x2="290" y2="67.5" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="70.5" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">150</text>
<line x1="40" y1="20" x2="290" y2="20" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="23" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">200</text>
<line x1="133.75" y1="20" x2="133.75" y2="210" stroke="var(--border-strong)" stroke-width="1" stroke-dasharray="3 3"/>
<line x1="196.25" y1="20" x2="196.25" y2="210" stroke="var(--border-strong)" stroke-width="1" stroke-dasharray="3 3"/>
<path d="M 40.00 210.00 L 133.75 39.00 L 196.25 39.00 L 290.00 210.00" fill="none" stroke="var(--series-1)" stroke-width="2" stroke-linejoin="round"/>
<line x1="40" y1="210" x2="290" y2="210" stroke="var(--border-strong)" stroke-width="1.25"/>
</svg>
<p class="subplot-caption">180&#176;/s max, held for 0.2s.</p>
</div>
<div class="subplot">
<svg viewBox="0 0 300 250" role="img" aria-label="Acceleration vs time: a flat 600 degrees per second squared for 0.3 seconds, then a flat 0 during the 0.2 second cruise, then a flat -600 for the final 0.3 seconds -- a bounded step function, never spiking">
<text x="40" y="10" font-family="var(--font-mono)" font-size="10.5" font-weight="600" fill="var(--text)" text-anchor="start">Acceleration</text>
<line x1="40" y1="196.43" x2="290" y2="196.43" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="199.43" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">-600</text>
<line x1="40" y1="155.71" x2="290" y2="155.71" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="158.71" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">-300</text>
<line x1="40" y1="115" x2="290" y2="115" stroke="var(--border-strong)" stroke-width="1.25"/>
<text x="34" y="118" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">0</text>
<line x1="40" y1="74.29" x2="290" y2="74.29" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="77.29" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">300</text>
<line x1="40" y1="33.57" x2="290" y2="33.57" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="36.57" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">600</text>
<line x1="133.75" y1="20" x2="133.75" y2="210" stroke="var(--border-strong)" stroke-width="1" stroke-dasharray="3 3"/>
<line x1="196.25" y1="20" x2="196.25" y2="210" stroke="var(--border-strong)" stroke-width="1" stroke-dasharray="3 3"/>
<path d="M 40.00 33.57 L 133.75 33.57 L 133.75 115.00 L 196.25 115.00 L 196.25 196.43 L 290.00 196.43" fill="none" stroke="var(--series-2)" stroke-width="2" stroke-linejoin="round"/>
</svg>
<p class="subplot-caption">&#177;600&#176;/s&#178;, never more.</p>
</div>
</div>
<p class="chart-caption">Same move as above, split into its two derivatives. The dashed lines mark the phase boundaries at t=0.3s and t=0.5s. Acceleration is a bounded step, not a spike &#8212; that's the entire difference between this and the naive step command: nothing here ever asks the hardware for more than the two limits you gave it.</p>
</div>

Those two limits are the whole API. In code, planning this exact move is:

```cpp
TrajectoryLimits limits(180.0f, 600.0f);   // vMax (deg/s), aMax (deg/s^2)
TrapezoidalProfile profile;
profile.plan(0.0f, 90.0f, limits);          // q0, qf -- minimum time

// every control tick, t = seconds since plan():
float pos, vel, accel;
bool stillMoving = profile.evaluate(t, pos, vel, accel);
```

`evaluate()` is a pure function of elapsed time — no clock owned internally, no state that changes except what `plan()` set up. Call it once a control loop tick, feed `pos` (or `vel`, for a velocity-mode driver) to the motor, and the same three numbers replay identically whether `t` comes from an Arduino's `millis()` or an EtherCAT master's cycle counter.

## Short moves skip the cruise entirely

Not every move is long enough to reach the cruise phase. A short hop only has room to accelerate partway before it has to start slowing down again to land exactly on target — the velocity profile never flattens into a plateau, it just ramps up and straight back down. The shape degrades gracefully from trapezoid to triangle; nothing about the underlying rule changes, the move is just too short to make full use of the speed limit.

<div class="chart-figure">
<p class="chart-title">A 20&#176; move never reaches the same top speed as a 90&#176; move</p>
<svg viewBox="0 0 680 260" role="img" aria-label="Velocity vs time comparing a 90 degree move that reaches the full 180 degree per second cap (trapezoid shape) against a 20 degree move that peaks at only about 110 degrees per second before decelerating (triangle shape), finishing in 0.365 seconds versus 0.8 seconds">
<g stroke="var(--border)" stroke-width="1">
<line x1="48" y1="220" x2="656" y2="220" />
<line x1="48" y1="170" x2="656" y2="170" />
<line x1="48" y1="120" x2="656" y2="120" />
<line x1="48" y1="70" x2="656" y2="70" />
<line x1="48" y1="20" x2="656" y2="20" />
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">
<text x="40" y="224">0</text>
<text x="40" y="174">50</text>
<text x="40" y="124">100</text>
<text x="40" y="74">150</text>
<text x="40" y="24">200</text>
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">
<text x="48" y="240">0.0s</text>
<text x="200" y="240">0.2s</text>
<text x="352" y="240">0.4s</text>
<text x="504" y="240">0.6s</text>
<text x="656" y="240">0.8s</text>
</g>
<path d="M 48.00 220.00 L 276.00 40.00 L 428.00 40.00 L 656.00 220.00" fill="none" stroke="var(--series-1)" stroke-width="2.5" stroke-linejoin="round"/>
<path d="M 48.00 220.00 L 186.76 110.46 L 325.51 220.00" fill="none" stroke="var(--series-2)" stroke-width="2.5" stroke-linejoin="round"/>
<circle cx="276.00" cy="40.00" r="4" fill="var(--bg-raised)" stroke="var(--series-1)" stroke-width="2.5"/>
<text x="276" y="30" font-family="var(--font-mono)" font-size="11" fill="var(--text)" font-weight="600" text-anchor="middle">180&#176;/s (vMax)</text>
<circle cx="186.76" cy="110.46" r="4" fill="var(--bg-raised)" stroke="var(--series-2)" stroke-width="2.5"/>
<text x="186.76" y="100.46" font-family="var(--font-mono)" font-size="11" fill="var(--text)" font-weight="600" text-anchor="middle">109.5&#176;/s</text>
</svg>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--series-1);"></span>0&#176; &#8594; 90&#176; (reaches vMax, trapezoid)</span>
<span><span class="swatch" style="background: var(--series-2);"></span>0&#176; &#8594; 20&#176; (never reaches vMax, triangle)</span>
</div>
<p class="chart-caption">Same vMax/aMax limits, two different distances. The short move settles by t=0.365s &#8212; it isn't slower per degree, it just runs out of room to keep accelerating before it has to start slowing down again. TrapezoidalProfile detects this internally; the caller never has to special-case it.</p>
</div>

## One smooth joint isn't the whole story

Everything above is one motor. A robot arm has several, and each joint's own move is a different distance — a shoulder sweeping 80°, an elbow sweeping 40°, a wrist sweeping 15°. Plan each of those independently for minimum time and they finish at three different moments: the wrist is done in under a third of a second while the shoulder has only just crossed a third of its own swing. Every individual joint traces a perfectly smooth trapezoid — and the arm's tip still doesn't move smoothly through space, because the joints aren't arriving together.

<div class="chart-figure">
<p class="chart-title">Three joints, planned independently vs. synchronized</p>
<div class="subplot-grid">
<div class="subplot">
<svg viewBox="0 0 300 250" role="img" aria-label="Percent of target reached vs time for three joints planned independently. The wrist finishes at 0.32 seconds, the elbow at 0.52 seconds, and the shoulder at 0.74 seconds -- three different arrival times">
<text x="40" y="10" font-family="var(--font-mono)" font-size="10.5" font-weight="600" fill="var(--text)" text-anchor="start">Unsynchronized</text>
<line x1="40" y1="210" x2="290" y2="210" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="213" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">0%</text>
<line x1="40" y1="115" x2="290" y2="115" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="118" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">50%</text>
<line x1="40" y1="20" x2="290" y2="20" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="23" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">100%</text>
<path d="M 40.00 210.00 L 44.65 209.84 L 49.31 209.37 L 53.96 208.58 L 58.61 207.47 L 63.26 206.05 L 67.92 204.31 L 72.57 202.26 L 77.22 199.89 L 81.88 197.21 L 86.53 194.21 L 91.18 190.89 L 95.83 187.26 L 100.49 183.31 L 105.14 179.04 L 109.79 174.46 L 114.44 169.57 L 119.10 164.35 L 123.75 158.83 L 128.40 152.98 L 133.06 146.82 L 137.71 140.46 L 142.36 134.09 L 147.01 127.73 L 151.67 121.36 L 156.32 115.00 L 160.97 108.64 L 165.62 102.27 L 170.28 95.90 L 174.93 89.54 L 179.58 83.18 L 184.24 77.02 L 188.89 71.17 L 193.54 65.65 L 198.19 60.43 L 202.85 55.54 L 207.50 50.96 L 212.15 46.69 L 216.81 42.74 L 221.46 39.11 L 226.11 35.79 L 230.76 32.79 L 235.42 30.11 L 240.07 27.74 L 244.72 25.69 L 249.38 23.95 L 254.03 22.53 L 258.68 21.42 L 263.33 20.63 L 267.99 20.16 L 272.64 20.00 L 290.00 20.00" fill="none" stroke="var(--series-1)" stroke-width="2"/>
<path d="M 40.00 210.00 L 43.23 209.85 L 46.45 209.39 L 49.68 208.63 L 52.91 207.57 L 56.14 206.20 L 59.37 204.53 L 62.59 202.55 L 65.82 200.27 L 69.05 197.69 L 72.28 194.80 L 75.50 191.61 L 78.73 188.11 L 81.96 184.31 L 85.18 180.21 L 88.41 175.80 L 91.64 171.09 L 94.87 166.07 L 98.09 160.75 L 101.32 155.13 L 104.55 149.20 L 107.78 142.97 L 111.00 136.43 L 114.23 129.59 L 117.46 122.45 L 120.69 115.00 L 123.91 107.55 L 127.14 100.41 L 130.37 93.57 L 133.60 87.03 L 136.82 80.80 L 140.05 74.87 L 143.28 69.25 L 146.51 63.93 L 149.73 58.91 L 152.96 54.20 L 156.19 49.79 L 159.42 45.69 L 162.64 41.89 L 165.87 38.39 L 169.10 35.20 L 172.33 32.31 L 175.55 29.73 L 178.78 27.45 L 182.01 25.47 L 185.24 23.80 L 188.46 22.43 L 191.69 21.37 L 194.92 20.61 L 198.15 20.15 L 201.37 20.00 L 290.00 20.00" fill="none" stroke="var(--series-2)" stroke-width="2"/>
<path d="M 40.00 210.00 L 41.98 209.85 L 43.95 209.39 L 45.93 208.63 L 47.91 207.57 L 49.88 206.20 L 51.86 204.53 L 53.84 202.55 L 55.81 200.27 L 57.79 197.69 L 59.76 194.80 L 61.74 191.61 L 63.72 188.11 L 65.69 184.31 L 67.67 180.21 L 69.65 175.80 L 71.62 171.09 L 73.60 166.07 L 75.58 160.75 L 77.55 155.13 L 79.53 149.20 L 81.50 142.97 L 83.48 136.43 L 85.46 129.59 L 87.43 122.45 L 89.41 115.00 L 91.39 107.55 L 93.36 100.41 L 95.34 93.57 L 97.32 87.03 L 99.29 80.80 L 101.27 74.87 L 103.25 69.25 L 105.22 63.93 L 107.20 58.91 L 109.17 54.20 L 111.15 49.79 L 113.13 45.69 L 115.10 41.89 L 117.08 38.39 L 119.06 35.20 L 121.03 32.31 L 123.01 29.73 L 124.99 27.45 L 126.96 25.47 L 128.94 23.80 L 130.92 22.43 L 132.89 21.37 L 134.87 20.61 L 136.84 20.15 L 138.82 20.00 L 290.00 20.00" fill="none" stroke="var(--accent)" stroke-width="2"/>
<line x1="40" y1="210" x2="290" y2="210" stroke="var(--border-strong)" stroke-width="1.25"/>
</svg>
<p class="subplot-caption">Each joint at its own minimum time.</p>
</div>
<div class="subplot">
<svg viewBox="0 0 300 250" role="img" aria-label="Percent of target reached vs time for the same three joints after TrajectoryGroup re-plans them to a shared duration. All three now reach 100 percent together at 0.74 seconds">
<text x="40" y="10" font-family="var(--font-mono)" font-size="10.5" font-weight="600" fill="var(--text)" text-anchor="start">Synchronized</text>
<line x1="40" y1="210" x2="290" y2="210" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="213" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">0%</text>
<line x1="40" y1="115" x2="290" y2="115" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="118" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">50%</text>
<line x1="40" y1="20" x2="290" y2="20" stroke="var(--border)" stroke-width="1"/>
<text x="34" y="23" font-family="var(--font-mono)" font-size="9" fill="var(--text-faint)" text-anchor="end">100%</text>
<path d="M 40.00 210.00 L 44.65 209.84 L 49.31 209.37 L 53.96 208.58 L 58.61 207.47 L 63.26 206.05 L 67.92 204.31 L 72.57 202.26 L 77.22 199.89 L 81.88 197.21 L 86.53 194.21 L 91.18 190.89 L 95.83 187.26 L 100.49 183.31 L 105.14 179.04 L 109.79 174.46 L 114.44 169.57 L 119.10 164.35 L 123.75 158.83 L 128.40 152.98 L 133.06 146.82 L 137.71 140.46 L 142.36 134.09 L 147.01 127.73 L 151.67 121.36 L 156.32 115.00 L 160.97 108.64 L 165.62 102.27 L 170.28 95.90 L 174.93 89.54 L 179.58 83.18 L 184.24 77.02 L 188.89 71.17 L 193.54 65.65 L 198.19 60.43 L 202.85 55.54 L 207.50 50.96 L 212.15 46.69 L 216.81 42.74 L 221.46 39.11 L 226.11 35.79 L 230.76 32.79 L 235.42 30.11 L 240.07 27.74 L 244.72 25.69 L 249.38 23.95 L 254.03 22.53 L 258.68 21.42 L 263.33 20.63 L 267.99 20.16 L 272.64 20.00 L 290.00 20.00" fill="none" stroke="var(--series-1)" stroke-width="2"/>
<path d="M 40.00 210.00 L 44.65 209.68 L 49.31 208.74 L 53.96 207.16 L 58.61 204.95 L 63.26 202.10 L 67.92 198.63 L 72.57 194.52 L 77.22 190.10 L 81.88 185.69 L 86.53 181.27 L 91.18 176.85 L 95.83 172.43 L 100.49 168.01 L 105.14 163.60 L 109.79 159.18 L 114.44 154.76 L 119.10 150.34 L 123.75 145.92 L 128.40 141.51 L 133.06 137.09 L 137.71 132.67 L 142.36 128.25 L 147.01 123.84 L 151.67 119.42 L 156.32 115.00 L 160.97 110.58 L 165.62 106.16 L 170.28 101.75 L 174.93 97.33 L 179.58 92.91 L 184.24 88.49 L 188.89 84.08 L 193.54 79.66 L 198.19 75.24 L 202.85 70.82 L 207.50 66.40 L 212.15 61.99 L 216.81 57.57 L 221.46 53.15 L 226.11 48.73 L 230.76 44.31 L 235.42 39.90 L 240.07 35.48 L 244.72 31.37 L 249.38 27.90 L 254.03 25.05 L 258.68 22.84 L 263.33 21.26 L 267.99 20.32 L 272.64 20.00 L 290.00 20.00" fill="none" stroke="var(--series-2)" stroke-width="2"/>
<path d="M 40.00 210.00 L 44.65 209.16 L 49.31 206.63 L 53.96 202.76 L 58.61 198.77 L 63.26 194.78 L 67.92 190.79 L 72.57 186.80 L 77.22 182.81 L 81.88 178.82 L 86.53 174.83 L 91.18 170.84 L 95.83 166.86 L 100.49 162.87 L 105.14 158.88 L 109.79 154.89 L 114.44 150.90 L 119.10 146.91 L 123.75 142.92 L 128.40 138.93 L 133.06 134.94 L 137.71 130.96 L 142.36 126.97 L 147.01 122.98 L 151.67 118.99 L 156.32 115.00 L 160.97 111.01 L 165.62 107.02 L 170.28 103.03 L 174.93 99.04 L 179.58 95.06 L 184.24 91.07 L 188.89 87.08 L 193.54 83.09 L 198.19 79.10 L 202.85 75.11 L 207.50 71.12 L 212.15 67.13 L 216.81 63.14 L 221.46 59.16 L 226.11 55.17 L 230.76 51.18 L 235.42 47.19 L 240.07 43.20 L 244.72 39.21 L 249.38 35.22 L 254.03 31.23 L 258.68 27.24 L 263.33 23.37 L 267.99 20.84 L 272.64 20.00 L 290.00 20.00" fill="none" stroke="var(--accent)" stroke-width="2"/>
<line x1="40" y1="210" x2="290" y2="210" stroke="var(--border-strong)" stroke-width="1.25"/>
</svg>
<p class="subplot-caption">All three re-planned to 0.74s.</p>
</div>
</div>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--series-1);"></span>Shoulder (80&#176;)</span>
<span><span class="swatch" style="background: var(--series-2);"></span>Elbow (40&#176;)</span>
<span><span class="swatch" style="background: var(--accent);"></span>Wrist (15&#176;)</span>
</div>
<p class="chart-caption">Position shown as percent of each joint's own target, so three different distances plot on one axis. Unsynchronized, the wrist (0.32s) and elbow (0.52s) both finish well before the shoulder (0.74s) &#8212; each curve is individually smooth, but the set arrives staggered. TrajectoryGroup takes the slowest axis's own minimum-time duration and re-plans every other axis to exactly that duration, so all three cross 100% together.</p>
</div>

Concretely:

```cpp
TrajectoryLimits limits(180.0f, 600.0f);   // same limits, every joint
ITrajectoryProfile* joints[] = { &shoulder, &elbow, &wrist };
float q0[] = { 0.0f, 0.0f, 0.0f };
float qf[] = { 80.0f, 40.0f, 15.0f };
TrajectoryLimits perAxis[] = { limits, limits, limits };

TrajectoryGroup group;
group.plan(joints, q0, qf, perAxis, 3);   // re-plans every axis to the shared duration
```

Matching arrival *time* across joints is what `TrajectoryGroup` does — it's still working entirely in joint space, one independent trapezoid per axis, just stretched to a common finish line. It says nothing about the *shape* of the path the arm's tip actually traces between those two poses; a straight line through space, or a curve around an obstacle, is a related but different problem, sitting one layer up. That's for another article.

## Why this lives in its own tiny library

None of the math above cares what's on the other end of `evaluate()`'s output. `plan()` and `evaluate()` never touch a clock, allocate memory, or assume a platform — the caller supplies elapsed time, and gets back a position, velocity, and acceleration, deterministically, every time. That's what makes the exact same trapezoid in the charts above run identically on an 8-bit Arduino driving a single RC servo and on a Linux box under a hard-real-time EtherCAT master driving a whole arm — and what makes it possible to validate that shape once, on a desktop build with no hardware attached at all, before it ever touches a motor.

Universal-Trajectory-Interface is one piece of the [Universal Interface Stack](/projects/universal-interface-stack/) — the layer that turns "go to 90°" into a plan for getting there, so the layer below it (the motor driver) only ever has to track a curve, never invent one.
