---
title: "The Corner Your Motor Can Feel"
description: "A trapezoidal profile changes acceleration instantly — a corner the hardware has to absorb. S-curves round it off, and there are two ways to pay for that: with time, or with acceleration."
pubDate: 2026-08-30
tags: ["Robotics", "Embedded", "Motion Control"]
draft: true
---

[Last time](/articles/universal-trajectory-interface/) I argued that a target angle isn't a motion plan, and that a trapezoidal velocity profile — accelerate, cruise, decelerate — is the smallest thing that counts as one. It is. It bounds velocity, it bounds acceleration, and it's three lines of algebra you can run on an 8-bit chip.

It also has four sharp corners, and your hardware can feel every one of them.

## Bounded is not the same as smooth

Look at what a trapezoid actually asks for at the instant the move begins. Velocity is zero, and one control tick later it's climbing at the full acceleration limit. Acceleration went from 0 to 180°/s² *instantly* — not quickly, instantly. Same again when the cruise phase starts, and twice more on the way down.

The rate of change of acceleration has a name: **jerk**. A trapezoidal profile's jerk is zero everywhere except at four instants, where it is infinite.

No physical system does that. What actually happens is that the motor tries, the drivetrain flexes, and the difference comes out as a step input to every spring in the machine — the belt, the gear backlash, the arm's own bending stiffness. A step input is the broadest possible excitation you can hand a resonant structure: it contains energy at every frequency, including whichever one your arm happens to ring at. That's the knock you hear when a gantry starts, the shimmer at the end of a long arm, the settling time that's longer than the move deserved.

The fix is to stop asking for the impossible: put a limit on jerk too, so acceleration ramps instead of jumping. That's an **S-curve**.

## Seven segments instead of three

Bounding jerk splits each of the trapezoid's two acceleration phases into three: ramp acceleration up, hold it, ramp it back down. Three segments on the way up, one cruise, three on the way down — seven in total.

The name comes from what that does to **velocity**. A trapezoid ramps velocity in a straight line — full acceleration from the first instant, constant the whole way up. An S-curve has to ease into that acceleration and ease back out of it, so its velocity ramp leaves zero slowly, straightens out through the middle, and flattens gently into the cruise. That sigmoid is the S:

<div class="chart-figure">
<p class="chart-title">Where the S is: velocity through the acceleration phase</p>
<svg viewBox="0 0 680 240" role="img" aria-label="Velocity versus time during the acceleration phase of the same move. The trapezoidal profile is a perfectly straight diagonal line from zero to 90 degrees per second, reaching it at 0.5 seconds. Both S-curve profiles instead trace a sigmoid: they leave zero almost flat, steepen through the middle, then flatten gently as they approach 90 degrees per second. The jerk-percent curve reaches cruise at 0.5 seconds like the trapezoid, the jerk-limited one not until 0.75 seconds.">
<g stroke="var(--border)" stroke-width="1">
<line x1="56" y1="39.7" x2="656" y2="39.7" />
<line x1="56" y1="68.7" x2="656" y2="68.7" />
<line x1="56" y1="117.2" x2="656" y2="117.2" />
<line x1="56" y1="165.6" x2="656" y2="165.6" />
</g>
<line x1="56" y1="214" x2="656" y2="214" stroke="var(--border-strong)" stroke-width="1" />
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">
<text x="48" y="43">90</text>
<text x="48" y="72">75</text>
<text x="48" y="121">50</text>
<text x="48" y="169">25</text>
<text x="48" y="218">0</text>
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">
<text x="56" y="232">0</text>
<text x="232.5" y="232">0.25s</text>
<text x="408.9" y="232">0.5s</text>
<text x="585.4" y="232">0.75s</text>
</g>
<polyline fill="none" stroke="var(--accent)" stroke-width="2.5" points="56.0,214.0 408.9,39.7 656.0,39.7" />
<polyline fill="none" stroke="var(--series-1)" stroke-width="2.5" points="56.0,214.0 68.0,213.8 80.1,213.2 92.1,212.2 104.1,210.8 116.2,208.9 128.2,206.7 140.2,204.1 152.3,201.0 164.3,197.6 176.3,193.7 188.4,189.5 200.4,184.8 212.4,179.8 224.4,174.3 236.5,168.4 248.5,162.5 260.5,156.6 272.6,150.6 284.6,144.7 296.6,138.7 308.7,132.8 320.7,126.8 332.7,120.9 344.8,115.0 356.8,109.0 368.8,103.1 380.9,97.1 392.9,91.2 404.9,85.2 417.0,79.4 429.0,73.9 441.0,68.9 453.1,64.2 465.1,59.9 477.1,56.1 489.2,52.6 501.2,49.6 513.2,47.0 525.3,44.7 537.3,42.9 549.3,41.5 561.3,40.5 573.4,39.9 585.4,39.7 656.0,39.7" />
<polyline fill="none" stroke="var(--series-2)" stroke-width="2.5" points="56.0,214.0 64.0,213.8 72.0,213.2 80.1,212.2 88.1,210.7 96.1,208.9 104.1,206.7 112.1,204.0 120.2,201.0 128.2,197.5 136.2,193.6 144.2,189.4 152.3,184.7 160.3,179.6 168.3,174.1 176.3,168.2 184.3,162.3 192.4,156.4 200.4,150.5 208.4,144.6 216.4,138.7 224.4,132.8 232.5,126.8 240.5,120.9 248.5,115.0 256.5,109.1 264.6,103.2 272.6,97.3 280.6,91.4 288.6,85.5 296.6,79.6 304.7,74.1 312.7,69.0 320.7,64.3 328.7,60.0 336.7,56.2 344.8,52.7 352.8,49.7 360.8,47.0 368.8,44.8 376.9,42.9 384.9,41.5 392.9,40.5 400.9,39.9 408.9,39.7 656.0,39.7" />
</svg>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--accent);"></span>Trapezoidal &#8212; a straight ramp</span>
<span><span class="swatch" style="background: var(--series-1);"></span>S-curve (jerk limit)</span>
<span><span class="swatch" style="background: var(--series-2);"></span>S-curve (jerk percent)</span>
</div>
<p class="chart-caption">Velocity in &#176;/s, showing only the rise. The straight line is what a trapezoid does; both S-curves trace the sigmoid the profile is named for. The two S-curves have the same shape &#8212; the jerk-percent one is simply compressed into the trapezoid's 0.5&#8239;s instead of taking 0.75&#8239;s, which is the whole trade this article is about.</p>
</div>

The velocity curve is where the name lives, but the *reason* is easier to see one derivative up, in acceleration.

<div class="chart-figure">
<p class="chart-title">Acceleration through the same 90&#176; move</p>
<svg viewBox="0 0 680 260" role="img" aria-label="Acceleration versus time for three profiles making the same 90 degree move. The trapezoidal profile is a rectangular pulse that jumps instantly to plus 180 and later to minus 180. The S-curve profile ramps smoothly up to plus 180 and back, but its move ends at 1.75 seconds instead of 1.5. The jerk-percent profile also ramps smoothly and ends at 1.5 seconds like the trapezoid, but reaches a higher peak of plus and minus 269.">
<g stroke="var(--border)" stroke-width="1">
<line x1="56" y1="26" x2="656" y2="26" />
<line x1="56" y1="78" x2="656" y2="78" />
<line x1="56" y1="182" x2="656" y2="182" />
<line x1="56" y1="234" x2="656" y2="234" />
</g>
<line x1="56" y1="130" x2="656" y2="130" stroke="var(--border-strong)" stroke-width="1" />
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">
<text x="48" y="30">300</text>
<text x="48" y="82">150</text>
<text x="48" y="134">0</text>
<text x="48" y="186">-150</text>
<text x="48" y="238">-300</text>
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">
<text x="56" y="252">0</text>
<text x="222.7" y="252">0.5s</text>
<text x="389.3" y="252">1.0s</text>
<text x="556" y="252">1.5s</text>
</g>
<polyline fill="none" stroke="var(--accent)" stroke-width="2.5" points="56.0,130.0 56.0,67.6 222.7,67.6 222.7,130.0 389.3,130.0 389.3,192.4 556.0,192.4 556.0,130.0 656.0,130.0" />
<polyline fill="none" stroke="var(--series-1)" stroke-width="2.5" points="56.0,130.0 139.3,67.6 222.7,67.6 306.0,130.0 389.3,130.0 472.7,192.4 556.0,192.4 639.3,130.0 656.0,130.0" />
<polyline fill="none" stroke="var(--series-2)" stroke-width="2.5" points="56.0,130.0 111.0,36.9 167.7,36.9 222.7,130.0 389.3,130.0 444.3,223.1 501.0,223.1 556.0,130.0 656.0,130.0" />
</svg>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--accent);"></span>Trapezoidal</span>
<span><span class="swatch" style="background: var(--series-1);"></span>S-curve (jerk limit)</span>
<span><span class="swatch" style="background: var(--series-2);"></span>S-curve (jerk percent)</span>
</div>
<p class="chart-caption">The same move (0 &#8594; 90&#176;, vMax 90&#176;/s, nominal aMax 180&#176;/s&#178;) under all three profiles. The trapezoidal trace has vertical edges &#8212; those are the infinite-jerk corners. Both S-curves replace them with finite slopes; the slope <em>is</em> the jerk limit. Note the two S-curves solve it differently: one runs longer, the other runs taller.</p>
</div>

That's the whole idea. The vertical edges become slopes, and the steepness of each slope is exactly the jerk limit you asked for.

But look closely at the two S-curve traces, because they are not the same shape, and the difference is the actual subject of this article.

## Smoothness has to be paid for

Here's the thing nobody mentions when they tell you to use an S-curve: **rounding those corners removes area from underneath the acceleration curve.** Area under acceleration is velocity. If you ramp acceleration up over a quarter second instead of stepping it, you've given away some of the speed you would otherwise have gained.

So something has to give. Either the move takes longer, or the acceleration has to go higher to make up the loss. There is no third option, and that's the fork in the road.

### Paying with time

The obvious approach: keep every limit exactly as it was, hand the profile a jerk limit as well, and accept that the move now takes longer.

<div class="chart-figure">
<p class="chart-title">The same velocity curves, full move, with finish times</p>
<svg viewBox="0 0 680 260" role="img" aria-label="Velocity versus time for the same three profiles. All three rise to the 90 degrees per second cruise speed and return to zero. The trapezoidal and jerk-percent profiles both finish at 1.5 seconds, while the jerk-limited S-curve finishes later, at 1.75 seconds.">
<g stroke="var(--border)" stroke-width="1">
<line x1="56" y1="30" x2="656" y2="30" />
<line x1="56" y1="76" x2="656" y2="76" />
<line x1="56" y1="122" x2="656" y2="122" />
<line x1="56" y1="168" x2="656" y2="168" />
<line x1="56" y1="214" x2="656" y2="214" />
</g>
<line x1="556" y1="30" x2="556" y2="214" stroke="var(--border-strong)" stroke-width="1" stroke-dasharray="3 3" />
<line x1="639.3" y1="30" x2="639.3" y2="214" stroke="var(--border-strong)" stroke-width="1" stroke-dasharray="3 3" />
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">
<text x="48" y="34">100</text>
<text x="48" y="80">75</text>
<text x="48" y="126">50</text>
<text x="48" y="172">25</text>
<text x="48" y="218">0</text>
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">
<text x="56" y="232">0</text>
<text x="222.7" y="232">0.5s</text>
<text x="389.3" y="232">1.0s</text>
<text x="556" y="232">1.5s</text>
</g>
<g font-family="var(--font-mono)" font-size="10" fill="var(--text-faint)">
<text x="551" y="248" text-anchor="middle">1.50</text>
<text x="644" y="248" text-anchor="middle">1.75</text>
</g>
<polyline fill="none" stroke="var(--accent)" stroke-width="2.5" points="56.0,214.0 66.4,203.7 76.8,193.3 87.2,182.9 97.7,172.6 108.1,162.2 118.5,151.9 128.9,141.6 139.3,131.2 149.8,120.8 160.2,110.5 170.6,100.2 181.0,89.8 191.4,79.4 201.8,69.1 212.2,58.8 222.7,48.4 285.2,48.4 347.7,48.4 389.3,48.4 399.8,58.8 410.2,69.1 420.6,79.4 431.0,89.8 441.4,100.2 451.8,110.5 462.2,120.8 472.7,131.2 483.1,141.6 493.5,151.9 503.9,162.2 514.3,172.6 524.8,182.9 535.2,193.3 545.6,203.7 556.0,214.0 656.0,214.0" />
<polyline fill="none" stroke="var(--series-1)" stroke-width="2.5" points="56.0,214.0 68.2,213.1 80.3,210.5 92.5,206.1 104.6,199.9 116.8,192.0 128.9,182.3 141.1,170.9 153.2,158.8 165.4,146.7 177.5,134.7 189.7,122.6 201.8,110.5 214.0,98.4 226.1,86.4 238.3,75.7 250.4,66.8 262.6,59.6 274.8,54.2 286.9,50.6 299.1,48.7 311.2,48.4 347.7,48.4 384.1,48.4 396.3,48.7 408.4,50.6 420.6,54.2 432.7,59.6 444.9,66.8 457.0,75.7 469.2,86.4 481.3,98.4 493.5,110.5 505.7,122.6 517.8,134.7 530.0,146.7 542.1,158.8 554.3,170.9 566.4,182.3 578.6,192.0 590.7,199.9 602.9,206.1 615.0,210.5 627.2,213.1 639.3,214.0 656.0,214.0" />
<polyline fill="none" stroke="var(--series-2)" stroke-width="2.5" points="56.0,214.0 66.4,212.5 76.8,208.1 87.2,200.8 97.7,190.6 108.1,177.4 118.5,162.1 128.9,146.6 139.3,131.2 149.8,115.8 160.2,100.3 170.6,85.0 181.0,71.8 191.4,61.6 201.8,54.3 212.2,49.9 222.7,48.4 285.2,48.4 347.7,48.4 389.3,48.4 399.8,49.9 410.2,54.3 420.6,61.6 431.0,71.8 441.4,85.0 451.8,100.3 462.2,115.8 472.7,131.2 483.1,146.6 493.5,162.1 503.9,177.4 514.3,190.6 524.8,200.8 535.2,208.1 545.6,212.5 556.0,214.0 656.0,214.0" />
</svg>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--accent);"></span>Trapezoidal &#8212; 1.50s</span>
<span><span class="swatch" style="background: var(--series-1);"></span>Jerk limit &#8212; 1.75s</span>
<span><span class="swatch" style="background: var(--series-2);"></span>Jerk percent &#8212; 1.50s</span>
</div>
<p class="chart-caption">All three reach the same cruise speed and stop at the same place. The jerk-limited S-curve takes 250&#8239;ms longer for it &#8212; a 17% penalty on this move &#8212; because it never exceeds the original acceleration limit and simply spends longer getting up to speed.</p>
</div>

Seventeen percent is a real cost. On a pick-and-place doing thousands of cycles an hour it's the difference between hitting a rate target and missing it. And it's the cost that makes people quietly turn S-curves off.

### Paying with acceleration

The second approach starts from the opposite question. Instead of *"here's my jerk limit, how long does the move take?"*, it asks *"here's how long the move should take — how much smoothing can I fit inside that?"*

You give it a **jerk percentage** and a **nominal** acceleration limit. The percentage says what fraction of the acceleration phase is spent ramping rather than holding steady: 0% is a plain trapezoid, 100% means the acceleration is ramping the entire time and never holds a constant value at all. The profile then keeps the acceleration phase exactly as long as the trapezoid's was, and raises the acceleration plateau just enough to put back the area the ramps took away.

The whole derivation is one line of bookkeeping. The acceleration phase has to deliver the same velocity change as before &mdash; the same area under the acceleration curve &mdash; in the same time <math><mrow><msub><mi>T</mi><mi>A</mi></msub></mrow></math>. Rounding the corners removes two triangles from that area, so the plateau has to rise to compensate. Writing the ramp fraction as <math><mrow><mi>p</mi></mrow></math>, each ramp lasts <math><mrow><mi>p</mi><msub><mi>T</mi><mi>A</mi></msub><mo>/</mo><mn>2</mn></mrow></math> and the areas balance when:

<math display="block" class="formula"><mrow><msup><mi>A</mi><mo>&#x2032;</mo></msup><mo>&#x2062;</mo><mo>(</mo><msub><mi>T</mi><mi>A</mi></msub><mo>&#x2212;</mo><msub><mi>t</mi><mi>j</mi></msub><mo>)</mo><mo>=</mo><mi>A</mi><mo>&#x22C5;</mo><msub><mi>T</mi><mi>A</mi></msub><mo>,</mo><mspace width="1.5em"/><msub><mi>t</mi><mi>j</mi></msub><mo>=</mo><mfrac><mrow><mi>p</mi><msub><mi>T</mi><mi>A</mi></msub></mrow><mn>2</mn></mfrac></mrow></math>

which rearranges into the two lines the code actually runs &mdash; the raised plateau, and the jerk that produces those ramps:

<math display="block" class="formula"><mrow><msup><mi>A</mi><mo>&#x2032;</mo></msup><mo>=</mo><mfrac><mrow><mn>2</mn><mi>A</mi></mrow><mrow><mn>2</mn><mo>&#x2212;</mo><mi>p</mi></mrow></mfrac><mspace width="3em"/><mi>J</mi><mo>=</mo><mfrac><mrow><mn>2</mn><msup><mi>A</mi><mo>&#x2032;</mo></msup></mrow><mrow><msub><mi>T</mi><mi>A</mi></msub><mo>&#x22C5;</mo><mi>p</mi></mrow></mfrac></mrow></math>

That's it. Feed <math><mrow><msup><mi>A</mi><mo>&#x2032;</mo></msup></mrow></math> and <math><mrow><mi>J</mi></mrow></math> into exactly the same seven-segment engine and the move comes out smooth *and* finishes on the trapezoid's original schedule. In the charts above, the jerk-percent trace reaches its cruise speed at precisely the same instant the trapezoid does, and both cross zero at 1.50&#8239;s.

The price is on the vertical axis. Peak acceleration went from 180 to **269°/s²** — about 1.49&times;. This is the part to internalise, because it inverts an assumption that holds everywhere else in the library:

**`aMax` is not a ceiling here. It's a nominal figure the profile deliberately exceeds.**

How far it exceeds it depends entirely on the percentage:

| Jerk percent | Peak acceleration |
|---|---|
| 10% | 1.05&times; nominal |
| 33% | 1.20&times; nominal |
| 50% | 1.33&times; nominal |
| **66%** &mdash; the default | **1.49&times; nominal** |
| 90% | 1.82&times; nominal |
| 100% | **2.00&times; nominal** |

At 100% you need double the acceleration you nominally asked for. So if the number you're passing in is your actuator's genuine limit, you have to derate it first — divide by <math><mrow><mn>2</mn><mo>/</mo><mo>(</mo><mn>2</mn><mo>&#x2212;</mo><mi>p</mi><mo>)</mo></mrow></math> — or the profile will confidently command torque the hardware cannot produce, and you'll get the very tracking error you were trying to avoid.

## Why 66%?

This parametrisation is common in industry, and 66% is a figure that comes up repeatedly as the recommended default — it's what I found on an MEI controller, and it's the default in the library.

I've heard the rationale explained this way: at roughly two-thirds, the acceleration pulse stops looking like a trapezoid and starts looking like **a half wave of a sine**. Which would make the whole scheme a piecewise-linear approximation of sinusoidal acceleration smoothing — a cheap way to get most of the benefit of a genuinely smooth accel profile using nothing but straight lines.

I can't vouch for that being the actual historical reasoning. But the geometry does hold up:

<div class="chart-figure">
<p class="chart-title">A 66% pulse against a half sine of equal area</p>
<svg viewBox="0 0 680 250" role="img" aria-label="The acceleration pulse at 66 percent jerk plotted against a half sine wave carrying the same area. The two curves track each other closely across the whole acceleration phase, with the trapezoid slightly under the sine near the edges and slightly over it in the middle. The trapezoid peaks at 1.49 times nominal acceleration and the sine at 1.57 times.">
<g stroke="var(--border)" stroke-width="1">
<line x1="56" y1="64" x2="656" y2="64" />
<line x1="56" y1="114" x2="656" y2="114" />
<line x1="56" y1="164" x2="656" y2="164" />
<line x1="56" y1="214" x2="656" y2="214" />
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">
<text x="48" y="68">1.5&#215;</text>
<text x="48" y="118">1.0&#215;</text>
<text x="48" y="168">0.5&#215;</text>
<text x="48" y="218">0</text>
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">
<text x="56" y="232">start of accel phase</text>
<text x="656" y="232">end</text>
</g>
<polyline fill="none" stroke="var(--series-1)" stroke-width="2.5" points="56.0,214.0 61.0,209.9 71.0,201.7 81.0,193.5 91.0,185.4 101.0,177.3 111.0,169.4 121.0,161.6 131.0,153.9 141.0,146.4 151.0,139.0 161.0,131.9 171.0,125.0 181.0,118.4 191.0,112.0 201.0,105.9 211.0,100.1 221.0,94.6 231.0,89.4 241.0,84.5 251.0,80.1 261.0,76.0 271.0,72.2 281.0,68.9 291.0,65.9 301.0,63.4 311.0,61.3 321.0,59.6 331.0,58.3 341.0,57.4 351.0,57.0 361.0,57.0 371.0,57.4 381.0,58.3 391.0,59.6 401.0,61.3 411.0,63.4 421.0,65.9 431.0,68.9 441.0,72.2 451.0,76.0 461.0,80.1 471.0,84.5 481.0,89.4 491.0,94.6 501.0,100.1 511.0,105.9 521.0,112.0 531.0,118.4 541.0,125.0 551.0,131.9 561.0,139.0 571.0,146.4 581.0,153.9 591.0,161.6 601.0,169.4 611.0,177.3 621.0,185.4 631.0,193.5 641.0,201.7 651.0,209.9 656.0,214.0" />
<polyline fill="none" stroke="var(--series-2)" stroke-width="2.5" points="56.0,214.0 254.0,64.7 458.0,64.7 656.0,214.0" />
</svg>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--series-1);"></span>Half sine</span>
<span><span class="swatch" style="background: var(--series-2);"></span>Jerk percent, 66%</span>
</div>
<p class="chart-caption">Both carry identical area, so both produce the same velocity change over the same acceleration phase. The half sine peaks at <math><mrow><mi>&#x3C0;</mi><mo>/</mo><mn>2</mn></mrow></math> &#8776; 1.571&times; nominal acceleration; the 66% pulse peaks at 1.493&times;, about 95% of it.</p>
</div>

Close enough that the description is fair, at least geometrically. If you actually fit a trapezoidal pulse to a half sine by least squares, the best match lands at 68% — and the error curve around there is shallow enough that 66% is indistinguishable from optimal in any way a machine would notice.

Whether that's *why* the number is 66, I genuinely don't know. It's a good default either way.

## So which one?

They're both in the library, and they answer different questions.

Reach for the **jerk limit** version when the jerk figure is a real physical constraint — a maximum rate of torque change the drivetrain tolerates, a spec someone handed you, a resonance you're deliberately staying under. You're saying "never exceed this," and you're willing to spend time to honour it.

Reach for the **jerk percent** version when the cycle time is the constraint and you have acceleration headroom to spend. You're saying "this move has to take exactly as long as it used to — smooth it out inside that budget." Which is usually the situation on a machine that already works and just rattles more than you'd like.

The honest summary:

| | `SCurveProfile` | `JerkPercentProfile` |
|---|---|---|
| **You specify** | An absolute jerk limit, `jMax` | A percentage of the acceleration phase |
| **Move duration** | Longer than the trapezoid | Identical to the trapezoid |
| **Smoothness costs you** | Time | Acceleration headroom |
| **`aMax` behaves as** | A hard ceiling | A nominal figure it exceeds |
| **Reach for it when** | Jerk is a real physical limit | Cycle time is fixed and you have headroom |

Both drop into the same interface as the original trapezoid, so switching between all three is a matter of changing which object you planned with — the code that calls `evaluate()` every control tick doesn't change, and neither does multi-axis synchronisation. You can even mix them across axes of the same machine, if one joint has headroom and another doesn't.

```cpp
SCurveProfile      wrist;              // jerk limit: never exceed jMax
JerkPercentProfile shoulder(66.0f);    // jerk percent: keep the schedule

TrajectoryLimits jerkLimited(90.0f, 180.0f, 720.0f);   // vMax, aMax, jMax
TrajectoryLimits nominal(90.0f, 180.0f);               // jMax unused here

wrist.plan(0.0f, 90.0f, jerkLimited);
shoulder.plan(0.0f, 90.0f, nominal);   // peak accel will be 269, not 180
```

## What the hardware actually says

Everything above is geometry. Here is what happened on a real joint: an Arduino
Nano driving a hobby servo, with an AS5600 magnetic encoder on the output shaft
reading the horn's actual angle back at 200&nbsp;Hz.

**The profiles are correct on the target.** The Nano's own output, sampled as
it ran, matches an independent double-precision implementation of the same
equations to within **0.001&deg;** &mdash; float32 rounding and nothing else,
for all three profiles. The maths survives the trip from MATLAB, through C++,
onto an 8-bit chip doing float arithmetic with no FPU.

And here is the move itself, as the encoder saw it:

<div class="chart-figure">
<p class="chart-title">Measured: the same 80&#176; move, all three profiles</p>
<svg viewBox="0 0 680 232" role="img" aria-label="Encoder-measured position versus time for the same 80 degree move under all three profiles. All three start at 4 degrees and arrive at 84 degrees. The trapezoidal and jerk-percent traces leave the start almost immediately and finish together at about 1.4 seconds; the jerk-limited S-curve eases away from the start more gradually and finishes later, at about 1.65 seconds. All three arrive smoothly with no overshoot or oscillation.">
<g stroke="var(--border)" stroke-width="1">
<line x1="56" y1="50.4" x2="656" y2="50.4" />
<line x1="56" y1="91.3" x2="656" y2="91.3" />
<line x1="56" y1="132.2" x2="656" y2="132.2" />
<line x1="56" y1="173.1" x2="656" y2="173.1" />
</g>
<line x1="56" y1="214" x2="656" y2="214" stroke="var(--border-strong)" stroke-width="1" />
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="end">
<text x="48" y="54">80&#176;</text>
<text x="48" y="95">60&#176;</text>
<text x="48" y="136">40&#176;</text>
<text x="48" y="177">20&#176;</text>
<text x="48" y="218">0&#176;</text>
</g>
<g font-family="var(--font-mono)" font-size="11" fill="var(--text-faint)" text-anchor="middle">
<text x="56" y="230">0</text>
<text x="222.7" y="230">0.5s</text>
<text x="389.3" y="230">1.0s</text>
<text x="556" y="230">1.5s</text>
</g>
<polyline fill="none" stroke="var(--accent)" stroke-width="2.2" points="56.0,205.8 64.3,205.8 72.7,205.8 81.0,206.0 89.3,205.8 97.7,205.8 106.0,205.6 114.3,204.9 122.7,202.9 131.0,201.0 139.3,199.5 147.7,197.7 156.0,195.2 164.3,193.1 172.7,190.7 181.0,187.3 189.3,184.3 197.7,181.2 206.0,177.6 214.3,174.2 222.7,170.1 231.0,165.9 239.3,161.4 247.7,156.6 256.0,151.7 264.3,147.2 272.7,143.1 281.0,138.6 289.3,134.3 297.7,130.2 306.0,125.7 314.3,120.8 322.7,116.3 331.0,111.8 339.3,107.2 347.7,102.7 356.0,98.4 364.3,93.9 372.7,89.4 381.0,84.5 389.3,80.4 397.7,76.4 406.0,73.0 414.3,69.1 422.7,65.7 431.0,62.6 439.3,59.9 447.7,57.2 456.0,54.7 464.3,52.2 472.7,50.4 481.0,48.2 489.3,47.0 497.7,45.9 506.0,45.0 514.3,44.1 522.7,43.0 531.0,42.5 539.3,42.5 547.7,42.3 556.0,42.3 564.3,42.3 572.7,42.3 581.0,42.5 589.3,42.5 597.7,42.3 606.0,42.3 614.3,42.3 622.7,42.3 631.0,42.3 639.3,42.3 647.7,42.3 656.0,42.3" />
<polyline fill="none" stroke="var(--series-2)" stroke-width="2.2" points="56.0,205.8 64.3,205.8 72.7,205.8 81.0,205.8 89.3,205.8 97.7,205.8 106.0,205.8 114.3,205.6 122.7,205.8 131.0,205.3 139.3,203.8 147.7,201.0 156.0,199.0 164.3,196.5 172.7,194.0 181.0,191.3 189.3,187.5 197.7,183.5 206.0,180.1 214.3,175.8 222.7,171.5 231.0,166.8 239.3,162.2 247.7,157.1 256.0,152.6 264.3,148.3 272.7,144.0 281.0,139.3 289.3,134.8 297.7,130.5 306.0,126.0 314.3,120.8 322.7,116.2 331.0,111.5 339.3,107.2 347.7,102.5 356.0,98.4 364.3,94.2 372.7,89.9 381.0,85.3 389.3,80.8 397.7,76.4 406.0,72.1 414.3,68.0 422.7,64.4 431.0,61.4 439.3,58.1 447.7,55.2 456.0,52.4 464.3,50.2 472.7,48.2 481.0,45.9 489.3,45.0 497.7,44.1 506.0,43.6 514.3,43.0 522.7,43.0 531.0,43.0 539.3,43.2 547.7,43.0 556.0,43.0 564.3,43.0 572.7,43.0 581.0,43.0 589.3,43.2 597.7,43.0 606.0,43.2 614.3,43.2 622.7,43.0 631.0,43.0 639.3,43.0 647.7,42.3 656.0,42.3" />
<polyline fill="none" stroke="var(--series-1)" stroke-width="2.2" points="56.0,205.8 64.3,205.8 72.7,205.8 81.0,205.8 89.3,205.8 97.7,205.8 106.0,205.8 114.3,205.8 122.7,205.8 131.0,205.8 139.3,205.6 147.7,204.7 156.0,203.3 164.3,201.2 172.7,199.7 181.0,197.9 189.3,195.6 197.7,193.4 206.0,191.6 214.3,189.1 222.7,185.9 231.0,183.0 239.3,179.9 247.7,176.4 256.0,172.6 264.3,168.6 272.7,164.7 281.0,160.4 289.3,155.9 297.7,151.4 306.0,146.9 314.3,142.4 322.7,137.4 331.0,132.7 339.3,128.7 347.7,124.4 356.0,119.9 364.3,115.1 372.7,110.6 381.0,106.3 389.3,101.6 397.7,97.5 406.0,93.3 414.3,89.0 422.7,84.5 431.0,80.6 439.3,76.8 447.7,73.2 456.0,69.4 464.3,65.8 472.7,63.0 481.0,60.1 489.3,57.6 497.7,55.2 506.0,52.9 514.3,51.1 522.7,49.5 531.0,47.9 539.3,46.6 547.7,45.5 556.0,44.8 564.3,43.9 572.7,43.2 581.0,42.7 589.3,42.5 597.7,42.5 606.0,42.5 614.3,42.7 622.7,42.7 631.0,42.7 639.3,42.7 647.7,42.7 656.0,42.7" />
</svg>
<div class="chart-legend">
<span><span class="swatch" style="background: var(--accent);"></span>Trapezoidal</span>
<span><span class="swatch" style="background: var(--series-1);"></span>S-curve (jerk limit)</span>
<span><span class="swatch" style="background: var(--series-2);"></span>S-curve (jerk percent)</span>
</div>
<p class="chart-caption">AS5600 readings at 200&#8239;Hz &#8212; the horn's real angle, not the command. Both S-curves ease away from the start instead of stepping into motion, and all three arrive without overshoot. The jerk-limited trace finishes last, as it should: it is the one that bought its smoothness with time.</p>
</div>

One caveat about reading that plot: a hobby servo has its own internal
controller with real dead time, so it smooths the commanded motion further and
lags it by a few degrees rather than tracing the designed trajectory exactly
&mdash; on a stepper, or a well-tuned BLDC with a proper position loop,
tracking is far closer than this.

Worth saying plainly: **on this rig nothing rang.** All three profiles arrive
monotonically, with no overshoot beyond the servo's own 0.45&deg; deadband. A
hobby servo is a closed loop with a well-damped controller of its own; the
step-input-excites-resonance story is real, but it belongs to stiff drivetrains
and open-loop steppers, not to something already filtering your command for
you. If you came here expecting a chart of a trapezoid ringing and an S-curve
not ringing, this rig could not produce one, and I would rather say so than
stage it.

Every number above is reproducible: `examples/HardwareValidation` and the two
scripts in `tools/` are what produced them.

## Try both of them yourself

The choice in the last two sections is a trade, and trades are easier to judge with your hands on the sliders. [**Trajectory Lab**](/tools/trajectory-lab/) runs this library in a browser tab, with all three profiles on one set of axes for the same move.

Drop `jMax` and watch the S-curve's acceleration corners round off while its duration grows. Push `jerkPercent` from 10% toward 100% and watch the jerk-percent profile keep the trapezoid's duration exactly while its acceleration plateau climbs past the `aMax` you asked for &mdash; the page reports the derived figure and the 2/(2&minus;p) multiplier next to the chart, so the headroom you're spending is on screen rather than implied.

The curves are not a re-derivation. `SCurveProfile.cpp` and `JerkPercentProfile.cpp` are compiled to WebAssembly and called in `float`, the same arithmetic the Nano above was running, and every build is checked sample-by-sample against the same double-precision transcription of the MATLAB that produced the reference traces in this article.

A second tab flashes a demo sketch onto your own Arduino over USB and plots what the board reports against the browser's own computation of the same move &mdash; the same comparison as the hardware section above, on your servo instead of mine. It wants Chrome or Edge on desktop, an ATmega328P board, and a servo; there's no encoder in that setup, so it plots pulse width rather than degrees, for the reason the calibration story above makes obvious.

## Get the library

Universal-Trajectory-Interface is free, public, and MIT-licensed:
[github.com/vishwam-aggarwal/Universal-Trajectory-Interface](https://github.com/vishwam-aggarwal/Universal-Trajectory-Interface).
Both profiles ship with runnable examples — `SCurveTrajectoryDemo` and
`JerkPercentTrajectoryDemo` — which deliberately run the same move with the
same limits as the original `SimpleTrajectoryDemo`, so you can flash all
three and watch the numbers in the table above come out of a real serial
port. Neither needs any hardware attached. `HardwareValidation` is the one
that does: it wants a servo and an AS5600, and it is what produced the
measured plot above.

It's still one piece of the [Universal Interface Stack](/projects/universal-interface-stack/) — the layer that decides what curve everything below it should be tracking. Adding jerk to that curve doesn't change the layering at all. It just means the curve no longer asks the hardware to do something impossible four times per move.
