#!/usr/bin/env python3
"""
Pi A — Minimal joystick → network sender (NDJSON) to Pi B.

Usage:
  python3 src/joy_net_send.py --host 172.28.214.226 --port 5055 --mode 0
  python3 src/joy_net_send.py --host 172.28.214.226 --mode 7j
  # If you want to fake joystick data (no ESP32 required), use --fake
  python3 src/joy_net_send.py --host 172.28.214.226 --mode 0 --fake
"""

import argparse, json, math, socket, sys, time
from pathlib import Path

# import shim so running from repo root works
THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

# Use the same helper as your UI/probe if present
try:
    from joystick_serial import frames as joy_frames
except Exception as e:
    joy_frames = None

DEF_HOST = "127.0.0.1"
DEF_PORT = 5055

def fake_frames():
    """Generator that yields smooth [-1,1] circles (when no ESP32 is attached)."""
    t0 = time.time()
    while True:
        t = time.time() - t0
        # circle on left stick, stationary right stick
        lx = math.sin(t*0.6)
        ly = math.cos(t*0.6)
        rx = 0.0
        ry = 0.0
        frame = {"lx": lx, "ly": ly, "lsw": 1, "rx": rx, "ry": ry, "rsw": 1}
        yield frame

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEF_HOST, help="Pi B host/ip")
    ap.add_argument("--port", type=int, default=DEF_PORT, help="Pi B port (default 5055)")
    ap.add_argument("--mode", default=None, help="Send a mode first (e.g., 0, 2, 7j). Optional.")
    ap.add_argument("--fake", action="store_true", help="Send synthetic joystick frames instead of real ESP32")
    args = ap.parse_args()

    # Pick a frame source
    if args.fake:
        frame_src = fake_frames()
        print("[JOY-SEND] Using synthetic frames (--fake)")
    else:
        if joy_frames is None:
            print("[ERR] joystick_serial not importable and --fake not set.")
            print("      Either run with --fake or ensure src/joystick_serial.py exists.")
            sys.exit(1)
        # Use the helper generator; no status callback needed here
        frame_src = joy_frames()
        print("[JOY-SEND] Using ESP32 joystick frames")

    # Connect to Pi B
    addr = (args.host, args.port)
    print(f"[NET] Connecting to {addr} ...")
    s = socket.create_connection(addr, timeout=3.0)
    f = s.makefile("w", encoding="utf-8", newline="\n")
    print("[NET] Connected.")

    # Optionally send a mode first (so Pi B switches runner)
    if args.mode is not None:
        mode_msg = {"type": "mode", "key": str(args.mode)}
        f.write(json.dumps(mode_msg, separators=(",",":")) + "\n")
        f.flush()
        print(f"[NET] Sent mode -> {args.mode}")

    # Stream joystick frames
    count = 0
    last_log = time.time()
    try:
        for fr in frame_src:
            fr_out = {
                "type": "joy",
                "lx": float(fr.get("lx", 0.0)),
                "ly": float(fr.get("ly", 0.0)),
                "rx": float(fr.get("rx", 0.0)),
                "ry": float(fr.get("ry", 0.0)),
                "lsw": int(fr.get("lsw", 1)),
                "rsw": int(fr.get("rsw", 1)),
            }
            f.write(json.dumps(fr_out, separators=(",",":")) + "\n")
            count += 1

            # light throttling to be polite if your serial is faster than needed
            # (your ESP32 is ~100 Hz already; this just mirrors source pace)
            # If fake, aim ~60 Hz:
            if args.fake:
                time.sleep(1/60.0)

            # periodic log
            now = time.time()
            if now - last_log >= 2.0:
                rate = count / (now - last_log)
                print(f"[JOY-SEND] pushed ~{rate:0.1f} fps; last lx={fr_out['lx']:+.2f} ly={fr_out['ly']:+.2f} "
                      f"rx={fr_out['rx']:+.2f} ry={fr_out['ry']:+.2f} lsw={fr_out['lsw']} rsw={fr_out['rsw']}")
                count = 0
                last_log = now

            # flush frequently enough so lines reach the server promptly
            f.flush()
    except KeyboardInterrupt:
        print("\n[JOY-SEND] Stopping.")
    finally:
        try:
            f.close()
            s.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
