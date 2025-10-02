#!/usr/bin/env python3
# Joystick reader for ESP32 CSV → normalized frames for network/UI.
# Yields dicts: {"lx","ly","rx","ry","lsw","rsw"} with values in [-1,1] and 0/1 buttons.

import os, time, serial
from typing import Dict, Generator, Optional, Iterable

# ---------- Config ----------
DEFAULT_PORTS = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyACM1", "/dev/ttyUSB1"]
BAUD = int(os.getenv("JOY_BAUD", "115200"))
ADC_MAX = int(os.getenv("JOY_ADC_MAX", "4095"))

# Feel
DEADZONE = float(os.getenv("JOY_DEADZONE", "0.04"))
LPF_ALPHA = float(os.getenv("JOY_LPF_ALPHA", "0.25"))

# Axes (adjust if your sticks feel swapped/inverted)
SWAP_LEFT_AXES  = os.getenv("JOY_SWAP_LEFT",  "1") == "1"   # swap J1 x<->y
SWAP_RIGHT_AXES = os.getenv("JOY_SWAP_RIGHT", "1") == "1"   # swap J2 x<->y
INVERT_LX = os.getenv("JOY_INV_LX", "0") == "1"
INVERT_LY = os.getenv("JOY_INV_LY", "1") == "1"             # commonly inverted
INVERT_RX = os.getenv("JOY_INV_RX", "0") == "1"
INVERT_RY = os.getenv("JOY_INV_RY", "1") == "1"

# Retry timing
RETRY_WAIT = 0.8

def _ports_from_env() -> Iterable[str]:
    env = os.getenv("JOY_PORTS", "").strip()
    if env:
        return [p for p in env.split(":") if p]
    return DEFAULT_PORTS

def _open_port() -> Optional[serial.Serial]:
    ports = _ports_from_env()
    for p in ports:
        try:
            ser = serial.Serial(p, BAUD, timeout=0.2)
            time.sleep(0.2)
            ser.reset_input_buffer()
            return ser
        except Exception:
            pass
    return None

def _norm(v: int) -> float:
    return (float(v) / ADC_MAX) * 2.0 - 1.0

def _dz(x: float, thr: float = DEADZONE) -> float:
    return 0.0 if abs(x) < thr else x

def _lpf(prev: float, new: float, alpha: float = LPF_ALPHA) -> float:
    return prev + alpha * (new - prev)

def frames(status_cb=None) -> Generator[Dict, None, None]:
    """
    Yields normalized frames:
      {"lx","ly","rx","ry","lsw","rsw"}   # lx..ry in [-1,1]; lsw/rsw: 0=pressed,1=released
    status_cb(state, detail) called with: "streaming" | "not_found" | "retrying" | "error"
    """
    lx = ly = rx = ry = 0.0
    while True:
        ser = _open_port()
        if not ser:
            if status_cb: status_cb("not_found", "no serial (/dev/ttyACM*/ttyUSB*)")
            time.sleep(RETRY_WAIT)
            if status_cb: status_cb("retrying", "rechecking ports")
            continue

        if status_cb: status_cb("streaming", getattr(ser, "port", ""))
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

                # normalize
                j1xn, j1yn = _norm(j1x), _norm(j1y)
                j2xn, j2yn = _norm(j2x), _norm(j2y)

                # optional swap
                if SWAP_LEFT_AXES:
                    j1xn, j1yn = j1yn, j1xn
                if SWAP_RIGHT_AXES:
                    j2xn, j2yn = j2yn, j2xn

                # inversion
                if INVERT_LX: j1xn = -j1xn
                if INVERT_LY: j1yn = -j1yn
                if INVERT_RX: j2xn = -j2xn
                if INVERT_RY: j2yn = -j2yn

                # deadzone + smoothing
                j1xn, j1yn = _dz(j1xn), _dz(j1yn)
                j2xn, j2yn = _dz(j2xn), _dz(j2yn)
                lx = _lpf(lx, j1xn)
                ly = _lpf(ly, j1yn)
                rx = _lpf(rx, j2xn)
                ry = _lpf(ry, j2yn)

                yield {"lx": lx, "ly": ly, "rx": rx, "ry": ry, "lsw": 0 if j1sw == 0 else 1, "rsw": 0 if j2sw == 0 else 1}
        except KeyboardInterrupt:
            if status_cb: status_cb("error", "KeyboardInterrupt")
            break
        except Exception as e:
            if status_cb: status_cb("error", f"{type(e).__name__}: {e}")
            time.sleep(RETRY_WAIT)
        finally:
            try: ser.close()
            except Exception: pass
