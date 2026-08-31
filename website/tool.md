---
title: "Trajectory Lab"
description: "Try trapezoidal and S-curve motion profiles in the browser, then flash the same code onto your own Arduino and watch a real servo run them — no toolchain, no library installs."
tags: ["Web Serial", "Motion Control", "Robotics"]
status: active
repo: "https://github.com/vishwam-aggarwal/Universal-Trajectory-Interface"
draft: true
---

Tell a motor to go from here to there and it will try to get there instantly. What stops that from tearing the machine apart is a *motion profile* — a curve deciding how position, velocity and acceleration evolve across the move, inside limits the hardware can actually honour. [Universal-Trajectory-Interface](/articles/universal-trajectory-interface/) is a small C++ library that generates those curves. Trajectory Lab is that library, running in your browser.

Drag a jerk limit and watch the corner round off the acceleration trace. Shorten a move until the trapezoid collapses into a triangle because it never gets a chance to reach full speed. Put all three profile types on one set of axes and see what each costs. Then, if you have an Arduino and a servo in a drawer, flash the demo firmware straight from the page and watch the same profile move something physical.

## It isn't a re-implementation

The obvious way to build a page like this is to rewrite the profile equations in JavaScript — at which point the page and the library are two pieces of software that agree only until one of them changes.

Instead, `TrapezoidalProfile.cpp`, `SCurveProfile.cpp` and `JerkPercentProfile.cpp` are compiled to WebAssembly and called in single-precision `float`, exactly as an 8-bit AVR would. What you see plotted is the same arithmetic the microcontroller executes, down to the rounding. Every build is checked sample-by-sample against an independent double-precision transcription of the original MATLAB — the same check that held the library's output on real hardware to within 0.001°.

That's also why the durations the page reports for its preset move — 1.50 s trapezoidal, 1.75 s S-curve, 1.50 s jerk-percent — are exactly the numbers the bundled example sketches print on a real serial port.

## What you'll need

Nothing at all for the **Simulate** tab. It works in any browser and needs no hardware.

The **Connect hardware** tab needs three things:

- **Chrome, Edge or Opera on desktop.** Both flashing and telemetry go through the Web Serial API. Firefox ships it only in Nightly behind a flag; Safari doesn't implement it.
- **An ATmega328P board** — an Uno, or a Nano of either bootloader vintage. All three take the same image; only the upload settings differ. Nano clones are split between two bootloaders and nothing on the port says which, so if flashing won't sync, pick the other Nano option and try again.
- **A hobby servo**, plus its own power supply. Any standard 3-wire servo will do.

## Wiring

<div class="chart-figure">
<p class="chart-title">Three connections, and one of them is the one people skip</p>
<svg viewBox="0 0 680 330" role="img" aria-label="Wiring diagram. An Arduino Uno or Nano connects pin A3 to the servo's signal wire. A separate 4.8 to 6 volt supply connects its positive terminal to the servo's V-plus wire. The Arduino ground, the supply's negative terminal, and the servo ground all join a single common ground line running along the bottom. The servo is never powered from the Arduino's own 5 volt pin.">
  <text x="30" y="86" text-anchor="middle" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)">USB</text>
  <line x1="8" y1="98" x2="60" y2="98" stroke="var(--text-faint)" stroke-width="1.5" stroke-dasharray="4 3"/>
  <rect x="60" y="60" width="180" height="110" rx="2" fill="none" stroke="var(--border-strong)" stroke-width="1.5"/>
  <text x="150" y="105" text-anchor="middle" font-family="var(--font-mono)" font-size="14" fill="var(--text)">Arduino</text>
  <text x="150" y="126" text-anchor="middle" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)">Uno or Nano</text>
  <rect x="440" y="50" width="190" height="110" rx="2" fill="none" stroke="var(--border-strong)" stroke-width="1.5"/>
  <text x="535" y="100" text-anchor="middle" font-family="var(--font-mono)" font-size="14" fill="var(--text)">Hobby servo</text>
  <text x="535" y="121" text-anchor="middle" font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)">any standard 3-wire</text>
  <rect x="200" y="210" width="200" height="62" rx="2" fill="none" stroke="var(--border-strong)" stroke-width="1.5"/>
  <text x="300" y="238" text-anchor="middle" font-family="var(--font-mono)" font-size="13" fill="var(--text)">4.8&ndash;6 V supply</text>
  <text x="300" y="258" text-anchor="middle" font-family="var(--font-mono)" font-size="10.5" fill="var(--text-faint)">not the board&rsquo;s 5 V pin</text>
  <polyline points="240,92 340,92 340,78 440,78" fill="none" stroke="var(--accent)" stroke-width="2.5"/>
  <circle cx="240" cy="92" r="4" fill="var(--accent)"/>
  <circle cx="440" cy="78" r="4" fill="var(--accent)"/>
  <text x="250" y="84" font-family="var(--font-mono)" font-size="12" fill="var(--accent)">A3</text>
  <text x="430" y="70" text-anchor="end" font-family="var(--font-mono)" font-size="12" fill="var(--accent)">signal</text>
  <polyline points="400,232 425,232 425,112 440,112" fill="none" stroke="var(--series-2)" stroke-width="2.5"/>
  <circle cx="400" cy="232" r="4" fill="var(--series-2)"/>
  <circle cx="440" cy="112" r="4" fill="var(--series-2)"/>
  <text x="406" y="224" font-family="var(--font-mono)" font-size="12" fill="var(--series-2)">+</text>
  <text x="430" y="104" text-anchor="end" font-family="var(--font-mono)" font-size="12" fill="var(--series-2)">V+</text>
  <line x1="130" y1="300" x2="560" y2="300" stroke="var(--series-1)" stroke-width="2.5"/>
  <polyline points="150,170 150,300" fill="none" stroke="var(--series-1)" stroke-width="2.5"/>
  <polyline points="300,272 300,300" fill="none" stroke="var(--series-1)" stroke-width="2.5"/>
  <polyline points="520,160 520,300" fill="none" stroke="var(--series-1)" stroke-width="2.5"/>
  <circle cx="150" cy="170" r="4" fill="var(--series-1)"/>
  <circle cx="300" cy="272" r="4" fill="var(--series-1)"/>
  <circle cx="520" cy="160" r="4" fill="var(--series-1)"/>
  <text x="160" y="190" font-family="var(--font-mono)" font-size="12" fill="var(--series-1)">GND</text>
  <text x="308" y="292" font-family="var(--font-mono)" font-size="12" fill="var(--series-1)">&minus;</text>
  <text x="530" y="180" font-family="var(--font-mono)" font-size="12" fill="var(--series-1)">GND</text>
  <text x="345" y="322" text-anchor="middle" font-family="var(--font-mono)" font-size="11.5" fill="var(--text-faint)">all three grounds tied together</text>
</svg>
<div class="chart-legend">
  <span><span class="swatch" style="background: var(--accent);"></span>Signal &mdash; pin A3</span>
  <span><span class="swatch" style="background: var(--series-2);"></span>Servo power &mdash; separate supply</span>
  <span><span class="swatch" style="background: var(--series-1);"></span>Common ground</span>
</div>
<p class="chart-caption">The servo's signal wire goes to pin A3. Its power comes from its own supply, never the board's 5 V pin &mdash; a servo's stall current will brown out a USB-powered board mid-move, which looks exactly like the profile glitching. The ground that supply shares with the Arduino is the connection people leave out, and without it nothing works reliably.</p>
</div>

## Using it

Open the app, go to **Connect hardware**, pick your board, and press **Flash**. The browser asks which serial port to use; the upload takes a few seconds. Then press **Connect**, pick a profile, set a target pulse width, and press **Move**.

The board plans and executes each move itself — the profile maths runs on the Arduino, not in the page — and streams back what it commanded. The chart draws that on top of the same profile computed in your browser, so you're watching an 8-bit microcontroller and a WebAssembly build agree on the same curve.

**Stop** is always available and never queues behind anything else.

If you'd rather not use the browser at all, the repo ships five example sketches; four need nothing but a board.

## Why the charts are in microseconds

The hardware charts are labelled in microseconds of pulse width rather than degrees, and that's deliberate.

Without an encoder there's no way to know what a pulse width does to *your* servo, and the usual assumption is wrong. "1000–2000 µs is 180°" is a convention, not a specification — the servo on this library's bench rig moves 0.0888 °/µs, so that range spans 89.7°, roughly half what everyone assumes.

That matters more than it sounds. Commanding as though the range were 180° produces a curve with the right shape at the wrong amplitude, which reads like the servo failing to track rather than like a calibration error. It cost real time to diagnose once already, so this demo doesn't repeat it. There's a field for your own measured °/µs if you've characterised your servo — with [Servo Calibrator](/tools/servo-calibrator/), for instance — and the page labels the result as your figure rather than as a measurement.

The full protocol reference and the library's own documentation are in the [project README](https://github.com/vishwam-aggarwal/Universal-Trajectory-Interface).
