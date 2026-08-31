// Checks the WebAssembly build against the reference math.
//
//   node wasm/verify_parity.mjs
//
// The web app's whole claim is that it runs the real library rather than a
// JavaScript re-derivation. This is what backs that claim: it plans a sweep
// of moves through website/app/uti.js (the compiled src/*.cpp) and compares
// every sample against tools/reference_profiles.py -- the independent
// double-precision transcription of the same MATLAB, which is the same
// oracle the hardware rig was held to at 0.001 deg.
//
// So the comparison is browser-build vs. reference math. Comparing the WASM
// against the C++ tests would prove far less, since they are the same
// source; comparing it against itself would prove nothing at all.
//
// Requires: a built website/app/uti.js (see wasm/build.sh) and python3 on
// PATH. reference_profiles.py is pure stdlib, so there is nothing to pip
// install.

import { createRequire } from 'node:module';
import { spawnSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.join(HERE, '..');

// Defaults to the committed bundle -- the one the website actually serves.
// CI also points this at a freshly built copy, so both the artifact that
// ships and the artifact current src/ produces are held to the reference.
const BUNDLE = process.argv[2]
  ? path.resolve(process.argv[2])
  : path.join(REPO, 'website', 'app', 'uti.js');

// Mirrors the Kind enum in wasm/uti_wasm.cpp and PROFILE_KIND in
// website/app.html.
const KIND = { trap: 0, scurve: 1, jerkpct: 2 };

// float32 carries ~7 significant decimal digits, and the reference is
// float64, so a relative floor is the right shape for this comparison.
// The absolute term covers values near zero, where a relative tolerance
// degenerates.
const RTOL = 5e-5;
const ATOL = 1e-4;

// How far to step when testing whether an acceleration mismatch is just a
// sample landing on a discontinuity (see checkAccel below).
const DISCONTINUITY_PROBE = 1e-4;

function close(actual, expected, scale) {
  const tol = ATOL * Math.max(1, scale) + RTOL * Math.abs(expected);
  return Math.abs(actual - expected) <= tol;
}

function runReference() {
  const py = process.env.PYTHON || 'python';
  const res = spawnSync(py, [path.join(HERE, 'gen_reference.py')], {
    encoding: 'utf-8',
    maxBuffer: 64 * 1024 * 1024,
  });
  if (res.error) {
    throw new Error(`could not run ${py}: ${res.error.message}`);
  }
  if (res.status !== 0) {
    throw new Error(`gen_reference.py failed:\n${res.stderr}`);
  }
  return JSON.parse(res.stdout);
}

async function loadModule() {
  if (!existsSync(BUNDLE)) {
    throw new Error(
      `${path.relative(REPO, BUNDLE)} not found -- run wasm/build.sh first.`
    );
  }
  const require = createRequire(import.meta.url);
  const factory = require(BUNDLE);
  return await factory();
}

function main0(mod, cases) {
  const uti_plan = mod.cwrap('uti_plan', 'number',
    ['number', 'number', 'number', 'number', 'number', 'number', 'number', 'number']);
  const uti_planned = mod.cwrap('uti_planned', 'number', ['number']);
  const uti_duration = mod.cwrap('uti_duration', 'number', ['number']);
  const uti_derived_aMax = mod.cwrap('uti_derived_aMax', 'number', ['number']);
  const uti_derived_jMax = mod.cwrap('uti_derived_jMax', 'number', ['number']);
  const uti_sample = mod.cwrap('uti_sample', 'number',
    ['number', 'number', 'number', 'number']);

  let checks = 0;
  let failures = 0;
  // Samples that landed on an acceleration step and were resolved by the
  // discontinuity probe. Reported rather than hidden -- a sudden jump in
  // this number means something moved.
  let straddles = 0;

  const fail = (msg) => {
    failures += 1;
    console.log(`  FAIL  ${msg}`);
  };

  for (const c of cases) {
    const kind = KIND[c.kind];
    const ok = uti_plan(kind, c.q0, c.qf, c.vMax, c.aMax, c.jMax,
                        c.jerkPercent, 0);

    checks += 1;
    if (!ok || !uti_planned(kind)) {
      fail(`${c.id}: plan() returned false`);
      continue;
    }

    checks += 1;
    if (!close(uti_duration(kind), c.duration, Math.abs(c.duration))) {
      fail(`${c.id}: duration ${uti_duration(kind)} != ${c.duration}`);
    }

    if (c.kind === 'jerkpct') {
      checks += 2;
      if (!close(uti_derived_aMax(kind), c.derivedAMax, Math.abs(c.derivedAMax))) {
        fail(`${c.id}: derived aMax ${uti_derived_aMax(kind)} != ${c.derivedAMax}`);
      }
      if (!close(uti_derived_jMax(kind), c.derivedJMax, Math.abs(c.derivedJMax))) {
        fail(`${c.id}: derived jMax ${uti_derived_jMax(kind)} != ${c.derivedJMax}`);
      }
    }

    const n = c.samples.length;
    const ptr = uti_sample(kind, 0, c.tEnd, n);
    if (ptr === 0) {
      fail(`${c.id}: uti_sample returned null`);
      continue;
    }
    const buf = new Float32Array(mod.HEAPF32.buffer, ptr, n * 4);

    // Scale tolerances by the size of the move, so a case spanning 90
    // units isn't held to the same absolute error as one spanning 0.05.
    const posScale = Math.max(Math.abs(c.q0), Math.abs(c.qf), 1);
    const velScale = Math.max(Math.abs(c.vMax), 1);
    const accScale = Math.max(Math.abs(c.aMax), c.derivedAMax || 0, 1);

    let caseFailed = false;
    for (let i = 0; i < n && !caseFailed; i += 1) {
      const [rt, rpos, rvel, racc] = c.samples[i];
      const pos = buf[i * 4 + 1];
      const vel = buf[i * 4 + 2];
      const acc = buf[i * 4 + 3];

      checks += 3;

      if (!close(pos, rpos, posScale)) {
        fail(`${c.id}: pos at t=${rt.toFixed(6)}  ${pos} != ${rpos}`);
        caseFailed = true;
      } else if (!close(vel, rvel, velScale)) {
        fail(`${c.id}: vel at t=${rt.toFixed(6)}  ${vel} != ${rvel}`);
        caseFailed = true;
      } else if (!close(acc, racc, accScale)) {
        // Acceleration is genuinely discontinuous for a trapezoid (it
        // steps at each segment boundary). A sample instant that lands on
        // a boundary can therefore fall on either side depending on
        // float32-vs-float64 rounding of the boundary time itself, and
        // disagree by the full step height without anything being wrong.
        //
        // Probe just either side: if the WASM value matches the reference
        // a hair before or after, this is a straddle, not a defect.
        const before = accelAt(c, rt - DISCONTINUITY_PROBE);
        const after = accelAt(c, rt + DISCONTINUITY_PROBE);
        if (close(acc, before, accScale) || close(acc, after, accScale)) {
          straddles += 1;
        } else {
          fail(`${c.id}: accel at t=${rt.toFixed(6)}  ${acc} != ${racc}`);
          caseFailed = true;
        }
      }
    }

    if (!caseFailed) {
      console.log(`  ok    ${c.id.padEnd(18)} ${n} samples, ` +
                  `duration ${c.duration.toFixed(6)}`);
    }
  }

  return { checks, failures, straddles };
}

// Reference acceleration at an arbitrary instant, interpolated from the
// sampled trace. Only used by the discontinuity probe, where all that
// matters is which side of a step we land on -- so nearest-sample is both
// sufficient and the right choice (interpolating across a step would
// invent a value that the reference never produces).
function accelAt(c, t) {
  const n = c.samples.length;
  const dt = c.tEnd / (n - 1);
  let idx = Math.round(t / dt);
  if (idx < 0) idx = 0;
  if (idx > n - 1) idx = n - 1;
  return c.samples[idx][3];
}

async function main() {
  console.log('WASM parity check -- website/app/uti.js vs ' +
              'tools/reference_profiles.py\n');

  const cases = runReference();
  const mod = await loadModule();

  console.log(`loaded ${path.relative(REPO, BUNDLE)}, ` +
              `${cases.length} reference cases\n`);

  const { checks, failures, straddles } = main0(mod, cases);

  console.log();
  if (straddles > 0) {
    console.log(`${straddles} sample(s) landed on an acceleration step and ` +
                `were resolved by probing either side.`);
  }
  console.log(`${checks} checks, ${failures} failure(s).`);

  if (failures > 0) {
    process.exitCode = 1;
  } else {
    console.log('\nPASS -- the browser build agrees with the reference math.');
  }
}

main().catch((err) => {
  console.error(`\nERROR: ${err.message}`);
  process.exitCode = 1;
});
