# WebAssembly build

Compiles the three scalar profiles to WebAssembly so the [Trajectory
Lab](../website/app.html) web app runs the **actual library** rather than a
JavaScript re-derivation of the same equations.

That distinction is the reason this directory exists. There are already
three transcriptions of this math in play — the MATLAB it was ported from,
`src/`, and `tools/reference_profiles.py` — and a fourth, hand-written in
JavaScript, would be one more thing to keep in sync and one more place for
the browser to quietly disagree with the hardware. Compiling `src/` instead
means the curve on the page is the same arithmetic an ATmega328P executes,
in the same `float` precision.

## Files

| | |
|---|---|
| `uti_wasm.cpp` | A flat `extern "C"` shim over `TrapezoidalProfile`, `SCurveProfile` and `JerkPercentProfile`. No embind — nothing here needs to marshal anything richer than floats. Sampling writes into one static buffer that JavaScript reads as a `Float32Array` view, so the path stays allocation-free. |
| `build.sh` | The single `emcc` invocation. |
| `gen_reference.py` | Emits expected traces from `tools/reference_profiles.py` as JSON. Spawned at test time, never committed, so it cannot go stale. |
| `verify_parity.mjs` | Compares the built bundle against those traces. |

## Building

```sh
bash wasm/build.sh                     # -> website/app/uti.js
bash wasm/build.sh build-wasm/uti.js   # -> somewhere else
node wasm/verify_parity.mjs            # check it against the reference math
```

Needs [emsdk](https://emscripten.org/docs/getting_started/downloads.html) on
`PATH`:

```sh
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk && ./emsdk install latest && ./emsdk activate latest
source ./emsdk_env.sh
```

**`website/app/uti.js` is a committed generated artifact.** After changing
anything in `src/` or `uti_wasm.cpp`, rebuild it, re-run the parity check,
and commit the result — the website pulls that file at build time, so a
stale commit ships stale math.

`-sSINGLE_FILE=1` base64-inlines the `.wasm` into the `.js`, so there is one
artifact rather than two. That also keeps it clear of the GitHub Contents
API's 1 MB ceiling, which is how the website fetches it.

## If you can't run emsdk locally

The `wasm` job in [`.github/workflows/build.yml`](../.github/workflows/build.yml)
builds the bundle on every push and uploads it as the `uti-wasm-bundle`
artifact. Download that and commit it:

```sh
gh run download <run-id> -n uti-wasm-bundle -D build-wasm
cp build-wasm/uti.js website/app/uti.js
```

This is not hypothetical — the machine this was developed on runs a Windows
Application Control policy that blocks emsdk's `clang.exe` and `emcc.exe`
outright (`An Application Control policy has blocked this file`), so CI was
the only way to produce the artifact. `tests/test_wasm_shim.cpp` exists
partly because of that: it compiles `uti_wasm.cpp` with an ordinary host
compiler and exercises the whole `extern "C"` surface, so the shim's logic
is covered by the normal desktop CI on both Linux and Windows, and emsdk is
only ever needed to *package* code that is already known good.

## Not built through CMake

`CMakeLists.txt` unconditionally builds a `device` library from the
`extern/Universal-Device-Interface` submodule, because `TrajectoryGroup` and
`CartesianMove` derive from `IDevice`. Neither is part of this bundle —
`TrapezoidalProfile`, `SCurveProfile` and `JerkPercentProfile` include only
`<math.h>` — so `build.sh` calls `emcc` directly and keeps the submodule out
of the WASM build entirely.

## The JavaScript surface

`build.sh` produces a `createUTI()` factory (Emscripten `MODULARIZE`), usable
from a `<script>` tag or from Node. Kinds are `0` trapezoidal, `1` s-curve,
`2` jerk-percent.

```js
const uti = await createUTI();
const plan   = uti.cwrap('uti_plan', 'number', Array(8).fill('number'));
const sample = uti.cwrap('uti_sample', 'number', Array(4).fill('number'));

// q0, qf, vMax, aMax, jMax, jerkPercent, targetDuration
plan(1 /* s-curve */, 0, 90, 90, 180, 720, 0, 0);

const ptr = sample(1, 0, 1.75, 500);
const rows = new Float32Array(uti.HEAPF32.buffer, ptr, 500 * 4);
// interleaved [t, pos, vel, accel]
```

Two things to know about `uti_sample`: it returns a view into a buffer that
the *next* call overwrites, so copy out before sampling another kind; and
sampling past `uti_duration()` is fine and intended — `evaluate()` clamps,
holding at `qf`, which is how the app draws a settled tail.

`uti_derived_aMax()` is worth reading whenever kind 2 is planned. It reports
what `JerkPercentProfile` actually commanded, which is `2/(2-p)` times the
nominal `aMax` — 1.49× at the default 66%, 2× at 100%. That class is the one
place in this library where `aMax` is a nominal figure rather than a ceiling.
