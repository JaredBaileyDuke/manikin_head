#!/usr/bin/env python3
"""
Pi B — Servo controller + TCP server.

Listens for NDJSON commands from Pi A (over Wi-Fi):
  - {"type":"mode","key":"2j"}  # select mode (0..9, optionally with 'j')
  - {"type":"joy","lx":..,"ly":..,"lsw":0/1,"rx":..,"ry":..,"rsw":0/1}  # joystick frames

Mode semantics:
  0  : live joystick control (both sticks) from network
  1..9 : scripted patterns
  1j..9j : left stick live on top of that feature (right stick ignored)

Run:
  python3 controller_server.py
"""

from time import sleep, time
from math import sin, pi
import threading
import socket
import json
import sys

# ====== NETWORK CONFIG ======
HOST = "0.0.0.0"
PORT = 5055
# ============================

# ---- Debug logging for joystick frames ----
# Print every frame (very chatty).
JOY_ECHO_EVERY_FRAME = False
# Or print a compact summary every N seconds (recommended).
JOY_LOG_EVERY = 2.0  # seconds; set 0 to disable periodic summaries

# Internals for logging
_last_log = 0.0
_frame_count = 0
_last_vals = {"lx": 0.0, "ly": 0.0, "rx": 0.0, "ry": 0.0, "lsw": 1, "rsw": 1}

# =========================
# Hardware setup
# =========================
try:
    from adafruit_servokit import ServoKit
except Exception as e:
    print("[ERR] adafruit_servokit missing; install with:")
    print("      pip install adafruit-circuitpython-servokit")
    raise

kit = ServoKit(channels=16)

# Eyelids
left_top       = kit.servo[2]
left_bottom    = kit.servo[3]
right_top      = kit.servo[6]
right_bottom   = kit.servo[7]

# Eyeballs
left_eye_lr    = kit.servo[0]  # left-right
left_eye_ud    = kit.servo[1]  # up-down
right_eye_lr   = kit.servo[4]  # left-right
right_eye_ud   = kit.servo[5]  # up-down

for s in (left_bottom, left_top, right_bottom, right_top,
          left_eye_lr, left_eye_ud, right_eye_lr, right_eye_ud):
    s.set_pulse_width_range(500, 2500)

# =========================
# Calibrated positions
# =========================
LEFT_TOP_OPEN       = 110
LEFT_TOP_CLOSED     = 30
LEFT_BOTTOM_OPEN    = 35
LEFT_BOTTOM_CLOSED  = 130

RIGHT_TOP_OPEN      = 100
RIGHT_TOP_CLOSED    = 30
RIGHT_BOTTOM_OPEN   = 45
RIGHT_BOTTOM_CLOSED = 135

LEFT_EYE_CENTER_LR  = 125
LEFT_EYE_LEFT       = 70
LEFT_EYE_RIGHT      = 180
LEFT_EYE_CENTER_UD  = 110
LEFT_EYE_UP         = 140
LEFT_EYE_DOWN       = 90

RIGHT_EYE_CENTER_LR = 110
RIGHT_EYE_LEFT      = 60
RIGHT_EYE_RIGHT     = 160
RIGHT_EYE_CENTER_UD = 125
RIGHT_EYE_UP        = 160
RIGHT_EYE_DOWN      = 110

# Dysconjugate (inward) LR hold positions
LEFT_EYE_INWARD_LR  = 150
RIGHT_EYE_INWARD_LR = 80

# Blink timing
BLINK_PERIOD_S = 1.0
CLOSE_TIME_S   = 0.15
OPEN_TIME_S    = BLINK_PERIOD_S - CLOSE_TIME_S

# Tremor
TREMOR_AMPLITUDE_DEG = 10.0
TREMOR_FREQ_HZ       = 12.0
TREMOR_STEP_S        = 0.03

# Nystagmus
NYSTAGMUS_AMPLITUDE_DEG = 10.0
NYSTAGMUS_FREQ_HZ       = 2.5
NYSTAGMUS_STEP_S        = 0.02
NYSTAGMUS_HOLD_S        = 2.5

SLEEP_QUANTUM = 0.05

# =========================
# Utilities
# =========================
def pausable_sleep(total_sec: float, stop_event: threading.Event, quantum: float = SLEEP_QUANTUM):
    deadline = time() + total_sec
    while time() < deadline and not stop_event.is_set():
        sleep(min(quantum, max(0, deadline - time())))

def _clamp_angle(a): return max(0, min(180, a))
def _clamp_to_range(a, amin, amax): return max(amin, min(amax, a))

def _biased_base_and_amp(base, amin, amax, desired_amp):
    amp = min(desired_amp, (amax - amin) / 2)
    if base - amin < amp:
        base = amin + amp
    elif amax - base < amp:
        base = amax - amp
    return base, amp

def _set_lids_hw(lb, lt, rb, rt):
    left_bottom.angle  = lb
    left_top.angle     = lt
    right_bottom.angle = rb
    right_top.angle    = rt

def _set_eyes_hw(l_lr, l_ud, r_lr, r_ud):
    left_eye_lr.angle  = l_lr
    left_eye_ud.angle  = l_ud
    right_eye_lr.angle = r_lr
    right_eye_ud.angle = r_ud

def park_eyes():
    _set_lids_hw(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN, RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
    _set_eyes_hw(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
                 RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)

def _map_norm_to_servo(val, center, amin, amax):
    """Map normalized [-1,+1] to servo [amin,amax] around center with clamping."""
    if val >= 0:
        return _clamp_to_range(center + val * (amax - center), amin, amax)
    else:
        return _clamp_to_range(center + val * (center - amin), amin, amax)

# =========================
# Shared network state
# =========================
class NetJoyBoth:
    """For mode 0 (both sticks)."""
    def __init__(self):
        self.lock = threading.Lock()
        self.rx = self.ry = 0.0
        self.lx = self.ly = 0.0
        self.lsw = 1
        self.rsw = 1
        self.active = False

NETB = NetJoyBoth()

class NetJoyLeft:
    """For j-modes (left stick only)."""
    def __init__(self):
        self.lock = threading.Lock()
        self.lx = self.ly = 0.0
        self.lsw = 1
        self.active = False

NETL = NetJoyLeft()

class ModeBus:
    """Holds the currently requested mode key (e.g., '2j')."""
    def __init__(self):
        self.lock = threading.Lock()
        self.key = None

MODEBUS = ModeBus()

# =========================
# Network server
# =========================
def network_server(stop_event):
    """Accepts a single client at a time; updates NET* and MODEBUS."""
    global _frame_count, _last_log, _last_vals

    print(f"[NET] Listening on {HOST}:{PORT}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((HOST, PORT))
        srv.listen(1)
        srv.settimeout(1.0)

        while not stop_event.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            print(f"[NET] Client connected from {addr}")
            with conn:
                f = conn.makefile("r", encoding="utf-8", newline="\n")
                with NETB.lock, NETL.lock:
                    NETB.active = True
                    NETL.active = True
                try:
                    for line in f:
                        if stop_event.is_set():
                            break
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            msg = json.loads(line)
                        except json.JSONDecodeError:
                            continue

                        mtype = msg.get("type", "")

                        if mtype == "mode":
                            key = str(msg.get("key", "")).lower().strip()
                            with MODEBUS.lock:
                                MODEBUS.key = key
                            print(f"[MODE] -> {key}")

                        elif mtype == "joy":
                            lx = float(msg.get("lx", 0.0))
                            ly = float(msg.get("ly", 0.0))
                            rx = float(msg.get("rx", 0.0))
                            ry = float(msg.get("ry", 0.0))
                            lsw = int(msg.get("lsw", 1))
                            rsw = int(msg.get("rsw", 1))

                            with NETB.lock:
                                NETB.lx, NETB.ly, NETB.rx, NETB.ry = lx, ly, rx, ry
                                NETB.lsw, NETB.rsw = lsw, rsw
                            with NETL.lock:
                                NETL.lx, NETL.ly, NETL.lsw = lx, ly, lsw

                            # ---- logging ----
                            _frame_count += 1
                            _last_vals.update(dict(lx=lx, ly=ly, rx=rx, ry=ry, lsw=lsw, rsw=rsw))
                            if JOY_ECHO_EVERY_FRAME:
                                print(f"[JOY] lx={lx:+.2f} ly={ly:+.2f}  rx={rx:+.2f} ry={ry:+.2f}  lsw={lsw} rsw={rsw}")
                            if JOY_LOG_EVERY:
                                now = time()
                                if now - _last_log >= JOY_LOG_EVERY:
                                    _last_log = now
                                    print(f"[JOY] { _frame_count } frames; "
                                          f"lx={_last_vals['lx']:+.2f} ly={_last_vals['ly']:+.2f} "
                                          f"rx={_last_vals['rx']:+.2f} ry={_last_vals['ry']:+.2f} "
                                          f"lsw={_last_vals['lsw']} rsw={_last_vals['rsw']}")
                finally:
                    with NETB.lock, NETL.lock:
                        NETB.active = False
                        NETL.active = False
            print("[NET] Client disconnected")

# =========================
# Motion primitives
# =========================
def sleep_with_tremor(duration, stop_event, tremor_side=None):
    if stop_event.is_set(): return
    if tremor_side not in ('left', 'right'):
        pausable_sleep(duration, stop_event)
        return

    servo = left_bottom if tremor_side == 'left' else right_bottom
    base = servo.angle if servo.angle is not None else (
        LEFT_BOTTOM_OPEN if tremor_side == 'left' else RIGHT_BOTTOM_OPEN
    )

    t = 0.0
    while t < duration and not stop_event.is_set():
        offset = TREMOR_AMPLITUDE_DEG * sin(2 * pi * TREMOR_FREQ_HZ * t)
        servo.angle = _clamp_angle(base + offset)
        dt = min(TREMOR_STEP_S, duration - t)
        pausable_sleep(dt, stop_event, quantum=dt)
        t += dt
    servo.angle = _clamp_angle(base)

def blink(stop_event, stroke_side=None, tremor_side=None):
    if stop_event.is_set(): return

    if stroke_side is None:
        _set_lids_hw(LEFT_BOTTOM_CLOSED, LEFT_TOP_CLOSED, RIGHT_BOTTOM_CLOSED, RIGHT_TOP_CLOSED)
        sleep_with_tremor(CLOSE_TIME_S, stop_event, tremor_side)
        if stop_event.is_set(): return
        _set_lids_hw(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN, RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
        sleep_with_tremor(OPEN_TIME_S, stop_event, tremor_side)

    elif stroke_side == 'left':
        left_top.angle     = LEFT_TOP_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_CLOSED
        right_top.angle    = RIGHT_TOP_CLOSED
        sleep_with_tremor(CLOSE_TIME_S, stop_event, tremor_side)
        if stop_event.is_set(): return
        left_top.angle     = LEFT_TOP_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_OPEN
        right_bottom.angle = RIGHT_BOTTOM_OPEN
        right_top.angle    = RIGHT_TOP_OPEN
        sleep_with_tremor(OPEN_TIME_S, stop_event, tremor_side)

    elif stroke_side == 'right':
        right_top.angle    = RIGHT_TOP_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_CLOSED
        left_top.angle     = LEFT_TOP_CLOSED
        sleep_with_tremor(CLOSE_TIME_S, stop_event, tremor_side)
        if stop_event.is_set(): return
        right_top.angle    = RIGHT_TOP_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_OPEN
        left_bottom.angle  = LEFT_BOTTOM_OPEN
        left_top.angle     = LEFT_TOP_OPEN
        sleep_with_tremor(OPEN_TIME_S, stop_event, tremor_side)

def hold_with_nystagmus(duration, stop_event, orientation='horizontal'):
    l_lr_base = left_eye_lr.angle if left_eye_lr.angle is not None else LEFT_EYE_CENTER_LR
    r_lr_base = right_eye_lr.angle if right_eye_lr.angle is not None else RIGHT_EYE_CENTER_LR
    l_ud_base = left_eye_ud.angle if left_eye_ud.angle is not None else LEFT_EYE_CENTER_UD
    r_ud_base = right_eye_ud.angle if right_eye_ud.angle is not None else RIGHT_EYE_CENTER_UD

    if orientation == 'horizontal':
        l_lr_base, l_lr_amp = _biased_base_and_amp(l_lr_base, LEFT_EYE_LEFT,  LEFT_EYE_RIGHT,  NYSTAGMUS_AMPLITUDE_DEG)
        r_lr_base, r_lr_amp = _biased_base_and_amp(r_lr_base, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT, NYSTAGMUS_AMPLITUDE_DEG)
        l_ud_amp = 0.0; r_ud_amp = 0.0
    else:
        l_ud_base, l_ud_amp = _biased_base_and_amp(l_ud_base, LEFT_EYE_DOWN,  LEFT_EYE_UP,  NYSTAGMUS_AMPLITUDE_DEG)
        r_ud_base, r_ud_amp = _biased_base_and_amp(r_ud_base, RIGHT_EYE_DOWN, RIGHT_EYE_UP, NYSTAGMUS_AMPLITUDE_DEG)
        l_lr_amp = 0.0; r_lr_amp = 0.0

    t = 0.0
    while t < duration and not stop_event.is_set():
        phase = sin(2 * pi * NYSTAGMUS_FREQ_HZ * t)
        l_lr = _clamp_to_range(l_lr_base + l_lr_amp * phase, LEFT_EYE_LEFT,  LEFT_EYE_RIGHT)
        r_lr = _clamp_to_range(r_lr_base + r_lr_amp * phase, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT)
        l_ud = _clamp_to_range(l_ud_base + l_ud_amp * phase, LEFT_EYE_DOWN,  LEFT_EYE_UP)
        r_ud = _clamp_to_range(r_ud_base + r_ud_amp * phase, RIGHT_EYE_DOWN, RIGHT_EYE_UP)
        _set_eyes_hw(l_lr, l_ud, r_lr, r_ud)
        dt = min(NYSTAGMUS_STEP_S, duration - t)
        pausable_sleep(dt, stop_event, quantum=dt)
        t += dt

def blink_with_nystagmus(stop_event, orientation='horizontal'):
    _set_lids_hw(LEFT_BOTTOM_CLOSED, LEFT_TOP_CLOSED, RIGHT_BOTTOM_CLOSED, RIGHT_TOP_CLOSED)
    hold_with_nystagmus(CLOSE_TIME_S, stop_event, orientation)
    if stop_event.is_set(): return
    _set_lids_hw(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN, RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
    hold_with_nystagmus(OPEN_TIME_S, stop_event, orientation)

def run_nystagmus_sequence(stop_event, orientation='horizontal'):
    if stop_event.is_set(): return
    _set_lids_hw(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN, RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)

    if stop_event.is_set(): return
    _set_eyes_hw(LEFT_EYE_LEFT, LEFT_EYE_CENTER_UD, RIGHT_EYE_LEFT, RIGHT_EYE_CENTER_UD)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    if stop_event.is_set(): return
    _set_eyes_hw(LEFT_EYE_RIGHT, LEFT_EYE_CENTER_UD, RIGHT_EYE_RIGHT, RIGHT_EYE_CENTER_UD)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    if stop_event.is_set(): return
    blink_with_nystagmus(stop_event, orientation)

    if stop_event.is_set(): return
    _set_eyes_hw(LEFT_EYE_CENTER_LR, LEFT_EYE_UP, RIGHT_EYE_CENTER_LR, RIGHT_EYE_UP)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    if stop_event.is_set(): return
    _set_eyes_hw(LEFT_EYE_CENTER_LR, LEFT_EYE_DOWN, RIGHT_EYE_CENTER_LR, RIGHT_EYE_DOWN)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    if stop_event.is_set(): return
    blink_with_nystagmus(stop_event, orientation)

def run_sequence(stop_event, *, stroke_side=None, tremor_side=None, dysconj_side=None):
    if stop_event.is_set(): return

    if stroke_side is None:
        _set_lids_hw(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN, RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
        l_lr_base = LEFT_EYE_INWARD_LR if dysconj_side == 'left' else LEFT_EYE_CENTER_LR
        r_lr_base = RIGHT_EYE_INWARD_LR if dysconj_side == 'right' else RIGHT_EYE_CENTER_LR
        _set_eyes_hw(l_lr_base, LEFT_EYE_CENTER_UD, r_lr_base, RIGHT_EYE_CENTER_UD)
    elif stroke_side == 'left':
        _set_lids_hw(LEFT_BOTTOM_OPEN, LEFT_TOP_CLOSED, RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
        _set_eyes_hw(LEFT_EYE_CENTER_LR, LEFT_EYE_DOWN, RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)
    elif stroke_side == 'right':
        _set_lids_hw(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN, RIGHT_BOTTOM_OPEN, RIGHT_TOP_CLOSED)
        _set_eyes_hw(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD, RIGHT_EYE_CENTER_LR, RIGHT_EYE_DOWN)

    sleep_with_tremor(0.2, stop_event, tremor_side)
    if stop_event.is_set(): return

    def L_LR(target):
        if stroke_side == 'left': return LEFT_EYE_CENTER_LR
        if dysconj_side == 'left': return LEFT_EYE_INWARD_LR
        return target
    def R_LR(target):
        if stroke_side == 'right': return RIGHT_EYE_CENTER_LR
        if dysconj_side == 'right': return RIGHT_EYE_INWARD_LR
        return target
    def L_UD(target): return LEFT_EYE_DOWN if stroke_side == 'left' else target
    def R_UD(target): return RIGHT_EYE_DOWN if stroke_side == 'right' else target

    _set_eyes_hw(L_LR(LEFT_EYE_LEFT),  L_UD(LEFT_EYE_CENTER_UD),
                 R_LR(RIGHT_EYE_LEFT), R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    if stop_event.is_set(): return

    _set_eyes_hw(L_LR(LEFT_EYE_RIGHT),  L_UD(LEFT_EYE_CENTER_UD),
                 R_LR(RIGHT_EYE_RIGHT), R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    if stop_event.is_set(): return

    _set_eyes_hw(LEFT_EYE_CENTER_LR, L_UD(LEFT_EYE_CENTER_UD),
                 RIGHT_EYE_CENTER_LR, R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    if stop_event.is_set(): return

    blink(stop_event, stroke_side=stroke_side, tremor_side=tremor_side)
    if stop_event.is_set(): return

    _set_eyes_hw(L_LR(LEFT_EYE_CENTER_LR), L_UD(LEFT_EYE_UP),
                 R_LR(RIGHT_EYE_CENTER_LR), R_UD(RIGHT_EYE_UP))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    if stop_event.is_set(): return

    _set_eyes_hw(L_LR(LEFT_EYE_CENTER_LR), L_UD(LEFT_EYE_DOWN),
                 R_LR(RIGHT_EYE_CENTER_LR), R_UD(RIGHT_EYE_DOWN))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    if stop_event.is_set(): return

    _set_eyes_hw(LEFT_EYE_CENTER_LR, L_UD(LEFT_EYE_CENTER_UD),
                 RIGHT_EYE_CENTER_LR, R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    blink(stop_event, stroke_side=stroke_side, tremor_side=tremor_side)

# =========================
# Mode runners
# =========================
def run_normal(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event)

def run_horizontal_nystagmus(stop_event):
    while not stop_event.is_set():
        run_nystagmus_sequence(stop_event, orientation='horizontal')

def run_vertical_nystagmus(stop_event):
    while not stop_event.is_set():
        run_nystagmus_sequence(stop_event, orientation='vertical')

def run_left_stroke(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, stroke_side='left')

def run_left_tremor(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, tremor_side='left')

def run_left_dysconj(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, dysconj_side='left')

def run_right_stroke(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, stroke_side='right')

def run_right_tremor(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, tremor_side='right')

def run_right_dysconj(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, dysconj_side='right')

# ========== NETWORK JOYSTICK MODES ==========
def run_joystick_network(stop_event):
    """Mode 0: both sticks live from network."""
    park_eyes()
    l_blink_t = 0.0
    r_blink_t = 0.0
    last_tick = time()
    DO_CYCLE = True

    while not stop_event.is_set():
        now = time()
        dt = max(0.0, now - last_tick)
        last_tick = now

        with NETB.lock:
            active = NETB.active
            lx, ly, rx, ry = NETB.lx, NETB.ly, NETB.rx, NETB.ry
            lsw, rsw = NETB.lsw, NETB.rsw

        if not active:
            pausable_sleep(0.01, stop_event)
            continue

        l_lr = _map_norm_to_servo(lx, LEFT_EYE_CENTER_LR,  LEFT_EYE_LEFT,  LEFT_EYE_RIGHT)
        l_ud = _map_norm_to_servo(ly, LEFT_EYE_CENTER_UD,  LEFT_EYE_DOWN,  LEFT_EYE_UP)
        r_lr = _map_norm_to_servo(rx, RIGHT_EYE_CENTER_LR, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT)
        r_ud = _map_norm_to_servo(ry, RIGHT_EYE_CENTER_UD, RIGHT_EYE_DOWN, RIGHT_EYE_UP)
        _set_eyes_hw(l_lr, l_ud, r_lr, r_ud)

        left_pressed  = (lsw == 0)
        right_pressed = (rsw == 0)
        if DO_CYCLE:
            l_blink_t = (l_blink_t + dt) if left_pressed  else 0.0
            r_blink_t = (r_blink_t + dt) if right_pressed else 0.0

            def side(pressed, t_accum, b_open, b_closed, t_open, t_closed):
                if not pressed:
                    return b_open, t_open
                if (t_accum % BLINK_PERIOD_S) < CLOSE_TIME_S:
                    return b_closed, t_closed
                return b_open, t_open

            lb, lt = side(left_pressed,  l_blink_t, LEFT_BOTTOM_OPEN, LEFT_BOTTOM_CLOSED,
                                               LEFT_TOP_OPEN,    LEFT_TOP_CLOSED)
            rb, rt = side(right_pressed, r_blink_t, RIGHT_BOTTOM_OPEN, RIGHT_BOTTOM_CLOSED,
                                               RIGHT_TOP_OPEN,    RIGHT_TOP_CLOSED)
        else:
            lb = LEFT_BOTTOM_CLOSED if left_pressed else LEFT_BOTTOM_OPEN
            lt = LEFT_TOP_CLOSED    if left_pressed else LEFT_TOP_OPEN
            rb = RIGHT_BOTTOM_CLOSED if right_pressed else RIGHT_BOTTOM_OPEN
            rt = RIGHT_TOP_CLOSED    if right_pressed else RIGHT_TOP_OPEN

        _set_lids_hw(lb, lt, rb, rt)
        pausable_sleep(SLEEP_QUANTUM, stop_event)

def run_leftstick_realtime_net(stop_event, *, feature="none"):
    """1j..9j: left stick live from network + feature overlay."""
    park_eyes()
    t0 = time()
    l_blink_t = 0.0
    last_tick = time()

    while not stop_event.is_set():
        now = time()
        dt = max(0.0, now - last_tick)
        last_tick = now
        t = now - t0

        with NETL.lock:
            active = NETL.active
            lx, ly, lsw = NETL.lx, NETL.ly, NETL.lsw

        if not active:
            pausable_sleep(0.01, stop_event)
            continue

        l_lr = _map_norm_to_servo(lx, LEFT_EYE_CENTER_LR,  LEFT_EYE_LEFT,  LEFT_EYE_RIGHT)
        l_ud = _map_norm_to_servo(ly, LEFT_EYE_CENTER_UD,  LEFT_EYE_DOWN,  LEFT_EYE_UP)
        r_lr = _map_norm_to_servo(lx, RIGHT_EYE_CENTER_LR, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT)
        r_ud = _map_norm_to_servo(ly, RIGHT_EYE_CENTER_UD, RIGHT_EYE_DOWN, RIGHT_EYE_UP)

        # Feature overlays (stroke/dysconj/nystag/tremor)
        left_top_cmd     = LEFT_TOP_OPEN
        right_top_cmd    = RIGHT_TOP_OPEN
        left_bottom_cmd  = LEFT_BOTTOM_OPEN
        right_bottom_cmd = RIGHT_BOTTOM_OPEN

        if feature == "stroke_left":
            l_ud = LEFT_EYE_DOWN
            left_top_cmd = LEFT_TOP_CLOSED
        elif feature == "stroke_right":
            r_ud = RIGHT_EYE_DOWN
            right_top_cmd = RIGHT_TOP_CLOSED

        if feature == "dysconj_left":
            l_lr = LEFT_EYE_INWARD_LR
        elif feature == "dysconj_right":
            r_lr = RIGHT_EYE_INWARD_LR

        # Nystagmus
        def add_nystag_lr(lrL, lrR, t):
            phase = sin(2 * pi * NYSTAGMUS_FREQ_HZ * t)
            llr_base, llr_amp = _biased_base_and_amp(lrL, LEFT_EYE_LEFT,  LEFT_EYE_RIGHT,  NYSTAGMUS_AMPLITUDE_DEG)
            rlr_base, rlr_amp = _biased_base_and_amp(lrR, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT, NYSTAGMUS_AMPLITUDE_DEG)
            return (
                _clamp_to_range(llr_base + llr_amp * phase, LEFT_EYE_LEFT, LEFT_EYE_RIGHT),
                _clamp_to_range(rlr_base + rlr_amp * phase, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT)
            )

        def add_nystag_ud(udL, udR, t):
            phase = sin(2 * pi * NYSTAGMUS_FREQ_HZ * t)
            lud_base, lud_amp = _biased_base_and_amp(udL, LEFT_EYE_DOWN,  LEFT_EYE_UP,  NYSTAGMUS_AMPLITUDE_DEG)
            rud_base, rud_amp = _biased_base_and_amp(udR, RIGHT_EYE_DOWN, RIGHT_EYE_UP, NYSTAGMUS_AMPLITUDE_DEG)
            return (
                _clamp_to_range(lud_base + lud_amp * phase, LEFT_EYE_DOWN, LEFT_EYE_UP),
                _clamp_to_range(rud_base + rud_amp * phase, RIGHT_EYE_DOWN, RIGHT_EYE_UP)
            )

        if feature == "nystag_h":
            l_lr, r_lr = add_nystag_lr(l_lr, r_lr, t)
        elif feature == "nystag_v":
            l_ud, r_ud = add_nystag_ud(l_ud, r_ud, t)

        # Blink while left button held
        left_pressed = (lsw == 0)
        if left_pressed:
            l_blink_t += dt
            closed = (l_blink_t % BLINK_PERIOD_S) < CLOSE_TIME_S
        else:
            l_blink_t = 0.0
            closed = False

        if closed:
            lb = LEFT_BOTTOM_CLOSED; lt = LEFT_TOP_CLOSED
            rb = RIGHT_BOTTOM_CLOSED; rt = RIGHT_TOP_CLOSED
        else:
            lb = left_bottom_cmd;  lt = left_top_cmd
            rb = right_bottom_cmd; rt = right_top_cmd
            if feature == "tremor_left":
                trem = TREMOR_AMPLITUDE_DEG * sin(2 * pi * TREMOR_FREQ_HZ * t)
                lb = _clamp_angle(LEFT_BOTTOM_OPEN + trem)
            elif feature == "tremor_right":
                trem = TREMOR_AMPLITUDE_DEG * sin(2 * pi * TREMOR_FREQ_HZ * t)
                rb = _clamp_angle(RIGHT_BOTTOM_OPEN + trem)

        _set_eyes_hw(l_lr, l_ud, r_lr, r_ud)
        _set_lids_hw(lb, lt, rb, rt)
        pausable_sleep(SLEEP_QUANTUM, stop_event)

# =========================
# Pattern runner (network-aware)
# =========================
MODES = {
    "0": ("Joystick control (network, both sticks)", run_joystick_network),
    "1": ("Normal", run_normal),
    "2": ("Horizontal nystagmus", run_horizontal_nystagmus),
    "3": ("Vertical nystagmus", run_vertical_nystagmus),
    "4": ("Left stroke", run_left_stroke),
    "5": ("Left tremor", run_left_tremor),
    "6": ("Left dysconj", run_left_dysconj),
    "7": ("Right stroke", run_right_stroke),
    "8": ("Right tremor", run_right_tremor),
    "9": ("Right dysconj", run_right_dysconj),
}

J_MODES = {
    "1": ("Normal (left-stick realtime)",            "none"),
    "2": ("Horizontal nystagmus (left-stick)",       "nystag_h"),
    "3": ("Vertical nystagmus (left-stick)",         "nystag_v"),
    "4": ("Left stroke (left-stick)",                "stroke_left"),
    "5": ("Left eyelid tremor (left-stick)",         "tremor_left"),
    "6": ("Left dysconjugate (left-stick)",          "dysconj_left"),
    "7": ("Right stroke (left-stick)",               "stroke_right"),
    "8": ("Right eyelid tremor (left-stick)",        "tremor_right"),
    "9": ("Right dysconjugate (left-stick)",         "dysconj_right"),
}

class PatternRunner:
    def __init__(self):
        self.thread = None
        self.stop_event = threading.Event()
        self.current_key = None

    def start_mode(self, key: str):
        base_key = key.rstrip('j')
        use_j = key.endswith('j')

        if base_key not in MODES:
            print("Unknown mode:", key)
            return

        # stop old
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

        self.stop_event = threading.Event()

        if use_j and base_key != "0":
            name, feature = J_MODES[base_key]
            print(f"→ Switched to: {name} [NET left joystick]")
            target = lambda ev: run_leftstick_realtime_net(ev, feature=feature)
        else:
            name, func = MODES[base_key]
            print(f"→ Switched to: {name}")
            target = func

        self.thread = threading.Thread(target=target, args=(self.stop_event,), daemon=True)
        self.thread.start()
        self.current_key = key

    def stop(self):
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.thread = None
        self.current_key = None

# =========================
# Main
# =========================
def main():
    stop_all = threading.Event()
    net_thread = threading.Thread(target=network_server, args=(stop_all,), daemon=True)
    net_thread.start()

    park_eyes()
    runner = PatternRunner()

    print("[CTRL] Ready. Waiting for 'mode' from network…  (Ctrl+C to exit)")
    last_seen = None
    try:
        while True:
            with MODEBUS.lock:
                key = MODEBUS.key
            if key and key != last_seen:
                last_seen = key
                runner.start_mode(key)
            sleep(0.05)
    except KeyboardInterrupt:
        print("\n[CTRL] Shutting down…")
    finally:
        runner.stop()
        stop_all.set()
        park_eyes()

if __name__ == "__main__":
    main()
