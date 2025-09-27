#!/usr/bin/env python3
import serial
import sys
import time

PORTS_TO_TRY = ["/dev/ttyUSB0", "/dev/ttyACM0"]
BAUD = 115200

def open_port():
    for p in PORTS_TO_TRY:
        try:
            s = serial.Serial(p, BAUD, timeout=1)
            print(f"[OK] Connected on {p}")
            # Clear any partial line
            time.sleep(0.2)
            s.reset_input_buffer()
            return s
        except Exception:
            pass
    print("[ERR] Could not open serial port. Try: ls /dev/ttyUSB* /dev/ttyACM*  (and check dmesg | tail)")
    sys.exit(1)

def parse_line(line):
    # Expect: j1x,j1y,j1sw,j2x,j2y,j2sw
    parts = line.strip().split(",")
    if len(parts) != 6:
        return None
    try:
        j1x, j1y, j1sw, j2x, j2y, j2sw = map(int, parts)
        return j1x, j1y, j1sw, j2x, j2y, j2sw
    except ValueError:
        return None

def normalize_adc(v, max_adc=4095):
    # Map 0..max_adc → -1..+1
    return (v / max_adc) * 2.0 - 1.0

def main():
    ser = open_port()
    try:
        # Header
        print("Reading... (Ctrl+C to quit)")
        while True:
            line = ser.readline().decode(errors="ignore")
            if not line:
                continue
            vals = parse_line(line)
            if not vals:
                continue

            j1x, j1y, j1sw, j2x, j2y, j2sw = vals

            # Normalize to -1..+1 for ESP32 (0..4095 ADC)
            j1x_n = normalize_adc(j1x)
            j1y_n = normalize_adc(j1y)
            j2x_n = normalize_adc(j2x)
            j2y_n = normalize_adc(j2y)

            # Simple deadzone display (±0.04)
            def dz(x, thr=0.04):
                return 0.0 if abs(x) < thr else x

            # Pretty one-line status
            print(
                f"J1 raw(x={j1x:4d}, y={j1y:4d}, sw={j1sw})  "
                f"norm(x={dz(j1x_n):+0.2f}, y={dz(j1y_n):+0.2f})  |  "
                f"J2 raw(x={j2x:4d}, y={j2y:4d}, sw={j2sw})  "
                f"norm(x={dz(j2x_n):+0.2f}, y={dz(j2y_n):+0.2f})"
            )
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()