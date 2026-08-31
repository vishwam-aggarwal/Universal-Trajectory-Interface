#!/usr/bin/env bash
#
# Builds the browser bundle for the Trajectory Lab web app.
#
# Compiles the three dependency-free scalar profiles from src/ plus the
# extern "C" shim in uti_wasm.cpp, and writes a single generated artifact
# to website/app/uti.js.
#
# That artifact is COMMITTED. It is what the website pulls at build time
# (see website/app.html and the note in wasm/README.md), so after changing
# anything in src/ or uti_wasm.cpp you must re-run this and commit the
# result, then re-run wasm/verify_parity.mjs.
#
# Requires emsdk on PATH:
#   git clone https://github.com/emscripten-core/emsdk.git
#   cd emsdk && ./emsdk install latest && ./emsdk activate latest
#   source ./emsdk_env.sh          # or emsdk_env.bat on Windows
#
# Note this deliberately does NOT go through CMakeLists.txt. That file
# builds an unconditional `device` library from the
# extern/Universal-Device-Interface submodule, which the simulator has no
# use for -- only TrajectoryGroup.h and CartesianMove.h include <IDevice.h>,
# and neither is part of this build. Invoking emcc directly keeps the WASM
# build free of the submodule entirely.

set -euo pipefail

cd "$(dirname "$0")/.."

# Defaults to the committed location. CI overrides it so it can build a
# fresh bundle without clobbering the committed one it is comparing against.
OUT="${1:-website/app/uti.js}"

mkdir -p "$(dirname "${OUT}")"

# Keep this list in sync with the extern "C" block in uti_wasm.cpp.
# Emscripten wants the leading underscore on each.
EXPORTS="_uti_plan,_uti_planned,_uti_duration,_uti_derived_aMax,_uti_derived_jMax,_uti_max_samples,_uti_sample"

# The emsdk ships emcc, emcc.bat and (on Windows) emcc.exe side by side in
# one directory, all of them on PATH once emsdk_env is sourced -- but which
# are actually runnable depends on the platform. On Linux only the
# extensionless script is executable; from a POSIX shell on Windows that
# same script has no exec bit and it is emcc.exe that works.
#
# So mere existence is not a good enough test: picking emcc.bat on Linux
# fails with "Permission denied" and exit 126. Probe each candidate by
# actually running it. Override with EMCC=... if your install differs.
EMCC="${EMCC:-}"
if [ -z "${EMCC}" ]; then
  for candidate in emcc emcc.exe emcc.bat; do
    if command -v "${candidate}" >/dev/null 2>&1 &&
       "${candidate}" --version >/dev/null 2>&1; then
      EMCC="${candidate}"
      break
    fi
  done
fi
if [ -z "${EMCC}" ]; then
  echo "error: no runnable emcc on PATH -- source your emsdk_env script first." >&2
  echo "       (if emcc is present but refuses to run, see wasm/README.md)" >&2
  exit 1
fi

"${EMCC}" \
  src/TrapezoidalProfile.cpp \
  src/SCurveProfile.cpp \
  src/JerkPercentProfile.cpp \
  wasm/uti_wasm.cpp \
  -I src \
  -std=c++11 \
  -Os \
  -sSINGLE_FILE=1 \
  -sMODULARIZE=1 \
  -sEXPORT_NAME=createUTI \
  -sENVIRONMENT=web,node \
  -sFILESYSTEM=0 \
  -sALLOW_MEMORY_GROWTH=0 \
  -sEXPORTED_FUNCTIONS="${EXPORTS}" \
  -sEXPORTED_RUNTIME_METHODS=cwrap,HEAPF32 \
  -o "${OUT}"

echo "Wrote ${OUT} ($(wc -c < "${OUT}") bytes)"
echo
echo "-sSINGLE_FILE=1 base64-inlines the .wasm into that one .js, so there is"
echo "no separate binary to ship. Next: node wasm/verify_parity.mjs"
