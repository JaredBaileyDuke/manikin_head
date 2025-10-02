#!/usr/bin/env python3
"""
Pi A — Minimal joystick probe for ESP32 CSV stream.
Reads /dev/ttyACM*/ttyUSB* and prints raw + normalized values.
"""

import sys, time, serial, argparse

DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1"]
BAUD = 115200
ADC_MAX = 4095

# Axis config
SWAP_LEFT_AXES  = True   # swap J1 x<->y
SWAP_RIGHT_AXES = True   # swap J2 x<->y
INVERT_LX = False
INVERT_LY = True   # you mentioned Y felt inverted
INVERT_RX = False
INVERT_RY = True
DEADZONE = 0.04

def open_port(port_list):
    for p in port_list:
        try:
            ser = serial.Serial(p, BAUD, timeout=0.2)
            time.sleep(0.2)
            ser.reset_input_buffer()
            return ser, p
        except Exception:
            pass
    return None, None

def norm(v: int) -> float:
    return (float(v) / ADC_MAX) * 2.0 - 1.0

def dz(x: float, thr: float = DEADZONE) -> float:
    return 0.0 if abs(x) < thr else x

def main():
    ap = argparse.ArgumentParser(description="ESP32 joystick probe")
    ap.add_argument("--port", help="Serial port (overrides auto-detect)")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--no-swap", action="store_true",
                    help="Disable axis swapping on both sticks")
    args = ap.parse_args()

    ports = [args.port] if args.port else DEFAULT_PORTS
    ser, used = open_port(ports)
    if not ser:
        print("No joystick serial found. Try:")
        print("  ls /dev/ttyACM* /dev/ttyUSB*")
        print("Then rerun: python3 joystick_probe.py --port /dev/ttyACM0")
        sys.exit(1)

    print(f"[OK] Connected: {used} @ {args.baud} baud")
    print("Expecting lines: j1x,j1y,j1sw,j2x,j2y,j2sw  (Ctrl+C to quit)\n")

    swap_left  = False if args.no_swap else SWAP_LEFT_AXES
    swap_right = False if args.no_swap else SWAP_RIGHT_AXES

    print("   J1 raw           J1 norm        |    J2 raw           J2 norm")
    print(" x    y  sw      x      y         |  x    y  sw      x      y")
    print("-"*70)

    try:
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            parts = line.split(",")
            if len(parts) != 6:
                continue
            try:
                j1x, j1y, j1sw, j2x, j2y, j2sw = map(int, parts)
            except ValueError:
                continue

            j1xn, j1yn = norm(j1x), norm(j1y)
            j2xn, j2yn = norm(j2x), norm(j2y)

            if swap_left:
                j1xn, j1yn = j1yn, j1xn
            if swap_right:
                j2xn, j2yn = j2yn, j2xn

            if INVERT_LX: j1xn = -j1xn
            if INVERT_LY: j1yn = -j1yn
            if INVERT_RX: j2xn = -j2xn
            if INVERT_RY: j2yn = -j2yn

            j1xn, j1yn = dz(j1xn), dz(j1yn)
            j2xn, j2yn = dz(j2xn), dz(j2yn)

            print(f"{j1x:4d} {j1y:4d}  {j1sw}   {j1xn:+0.2f} {j1yn:+0.2f}    |  "
                  f"{j2x:4d} {j2y:4d}  {j2sw}   {j2xn:+0.2f} {j2yn:+0.2f}")
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        try:
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
