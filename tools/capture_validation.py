#!/usr/bin/env python3
"""Drive examples/HardwareValidation and save what it streams to a CSV.

    python tools/capture_validation.py --port COM4 --out run.csv

Opens the port, waits for the sketch's "# READY", sends 'g', and records
everything until "# END". Comment lines (starting with '#') are echoed to the
terminal and stored in a sidecar .meta.txt, so the run's own record of its
limits and derived values travels with the data.

Only dependency is pyserial.
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")


def pick_port(explicit):
    if explicit:
        return explicit
    ports = list(list_ports.comports())
    if not ports:
        sys.exit("No serial ports found. Is the board plugged in?")
    if len(ports) == 1:
        print(f"Using the only port present: {ports[0].device} ({ports[0].description})")
        return ports[0].device
    print("Several ports present -- pick one with --port:")
    for p in ports:
        print(f"  {p.device}  {p.description}")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (auto-detected if only one)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="validation.csv", help="CSV to write")
    ap.add_argument("--timeout", type=float, default=90.0,
                    help="give up this many seconds after the run starts")
    args = ap.parse_args()

    port = pick_port(args.port)
    meta_path = args.out.rsplit(".", 1)[0] + ".meta.txt"

    print(f"Opening {port} at {args.baud}...")
    with serial.Serial(port, args.baud, timeout=1.0) as ser:
        # The sketch prints its banner once, at boot. Clear whatever is already
        # buffered and then deliberately reset the board, so the banner we read
        # is a fresh one rather than one we might have half-missed. (Just
        # sleeping and flushing loses the banner entirely on a board that reset
        # when the port was opened -- which is most classic Arduinos.)
        ser.reset_input_buffer()
        ser.dtr = False
        time.sleep(0.12)
        ser.dtr = True

        deadline = time.time() + 12.0
        ready = False
        banner = []
        while time.time() < deadline:
            line = ser.readline().decode("utf-8", "replace").strip()
            if not line:
                continue
            print("  " + line)
            banner.append(line)
            if line.startswith("# READY"):
                ready = True
                break

        if not ready:
            # Not necessarily fatal: a board that ignores DTR (native USB, or a
            # board already running) never re-prints its banner. The sketch sits
            # in loop() waiting for 'g' either way, so try anyway rather than
            # making the user power-cycle.
            print("\nNo '# READY' seen -- the board may not reset on DTR.\n"
                  "Trying anyway; if nothing arrives, check the sketch and baud rate.",
                  file=sys.stderr)

        print("\nSending 'g' -- THE SERVO IS ABOUT TO MOVE (3 sweeps of 90 deg).")
        ser.write(b"g")
        ser.flush()

        rows, meta = [], list(banner)
        header = None
        started = time.time()
        ended = False
        while time.time() - started < args.timeout:
            line = ser.readline().decode("utf-8", "replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                print("  " + line)
                meta.append(line)
                if line.startswith("# END"):
                    ended = True
                    break
                continue
            if header is None and line.startswith("profile,"):
                header = line
                continue
            rows.append(line)

        if not ended:
            print(f"\nWARNING: timed out after {args.timeout}s without '# END'. "
                  "Saving what arrived.", file=sys.stderr)

    if not rows:
        sys.exit("No data rows captured.")

    with open(args.out, "w", encoding="utf-8", newline="") as f:
        f.write((header or "profile,t_ms,cmd_deg,meas_deg,cmd_vel") + "\n")
        f.write("\n".join(rows) + "\n")
    with open(meta_path, "w", encoding="utf-8") as f:
        f.write("\n".join(meta) + "\n")

    counts = {}
    for r in rows:
        counts[r.split(",", 1)[0]] = counts.get(r.split(",", 1)[0], 0) + 1
    print(f"\nWrote {len(rows)} rows to {args.out}")
    for k, v in counts.items():
        print(f"  {k}: {v} samples")
    print(f"Run metadata in {meta_path}")
    print(f"\nNext:  python tools/plot_validation.py {args.out}")


if __name__ == "__main__":
    main()
