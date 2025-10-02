#!/usr/bin/env python3
from time import sleep, time
from math import sin, pi
import threading
import sys
import serial  # pip install pyserial
from adafruit_servokit import ServoKit

# =========================
# Setup
# =========================
kit = ServoKit(channels=16)

# Eyelids
left_top       = kit.servo[2]
left_bottom    = kit.servo[3]
right_top      = kit.servo[6]
right_bottom   = kit.servo[7]

# Eyeballs
# Left eye
left_eye_lr    = kit.servo[0]  # left-right
left_eye_ud    = kit.servo[1]  # up-down
# Right eye
right_eye_lr   = kit.servo[4]  # left-right
right_eye_ud   = kit.servo[5]  # up-down

# Match your servos' pulse range
for s in (
    left_bottom, left_top, right_bottom, right_top,
    left_eye_lr, left_eye_ud, right_eye_lr, right_eye_ud
):
    s.set_pulse_width_range(500, 2500)

# =========================
# Calibrated positions
# =========================
# Left eyelids
LEFT_TOP_OPEN       = 110
LEFT_TOP_CLOSED     = 30
LEFT_BOTTOM_OPEN    = 35
LEFT_BOTTOM_CLOSED  = 130

# Right eyelids
RIGHT_TOP_OPEN      = 100
RIGHT_TOP_CLOSED    = 30
RIGHT_BOTTOM_OPEN   = 45
RIGHT_BOTTOM_CLOSED = 135

# Left eyeball (servo 0 = left-right, servo 1 = up-down)
LEFT_EYE_CENTER_LR  = 125
LEFT_EYE_LEFT       = 70     # min LR
LEFT_EYE_RIGHT      = 180    # max LR
LEFT_EYE_CENTER_UD  = 110
LEFT_EYE_UP         = 140    # max UD
LEFT_EYE_DOWN       = 90     # min UD

# Right eyeball (servo 4 = left-right, servo 5 = up-down)
RIGHT_EYE_CENTER_LR = 110
RIGHT_EYE_LEFT      = 60     # min LR
RIGHT_EYE_RIGHT     = 160    # max LR
RIGHT_EYE_CENTER_UD = 125
RIGHT_EYE_UP        = 160    # max UD
RIGHT_EYE_DOWN      = 110    # min UD

# Dysconjugate (inward) LR hold positions — tweak to taste
LEFT_EYE_INWARD_LR  = 150
RIGHT_EYE_INWARD_LR = 80

# Blink timing
BLINK_PERIOD_S = 1.0
CLOSE_TIME_S   = 0.15
OPEN_TIME_S    = BLINK_PERIOD_S - CLOSE_TIME_S

# Tremor settings (bottom eyelid only)
TREMOR_AMPLITUDE_DEG = 10.0
TREMOR_FREQ_HZ       = 12.0
TREMOR_STEP_S        = 0.03

# Nystagmus settings
NYSTAGMUS_AMPLITUDE_DEG = 10.0
NYSTAGMUS_FREQ_HZ       = 2.5
NYSTAGMUS_STEP_S        = 0.02
NYSTAGMUS_HOLD_S        = 2.5

# Sleep quantum for responsiveness
SLEEP_QUANTUM = 0.05  # 50 ms

# =========================
# Joystick config
# =========================
PORTS_TO_TRY = ["/dev/ttyUSB0", "/dev/ttyACM0"]
BAUD = 115200
ADC_MAX = 4095

# Feel (deadzone & smoothing)
DZ_J1 = 0.04     # joystick 1 (left)
DZ_J2 = 0.04     # joystick 2 (right)
LPF_ALPHA = 0.25 # 0..1 (higher = snappier)

# Axis mapping (based on your earlier observation)
SWAP_LEFT_AXES  = True   # swap J1 x/y before mapping
SWAP_RIGHT_AXES = True   # swap J2 x/y before mapping
INVERT_LX = False        # invert J1 X after swap
INVERT_LY = False        # invert J1 Y after swap
INVERT_RX = False        # invert J2 X after swap
INVERT_RY = False        # invert J2 Y after swap

# Debug/prints (set to a number of seconds to enable, or None/0 to disable)
PRINT_STATUS = None  # e.g., 0.25 to print angles/buttons every 250 ms

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

# --- Hardware writers (do not apply overlay) ---
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

# =========================
# Joystick Overlay (shared state)
# =========================
class JoystickOverlay:
    """
    Holds smoothed joystick values in [-1,1] and button states.
    Kept for compatibility with scripted modes; 'j' modes do not use this.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.enabled = False
        self.lx = 0.0; self.ly = 0.0; self.rx = 0.0; self.ry = 0.0
        self.left_pressed = False
        self.right_pressed = False

JO = JoystickOverlay()

# How strongly the stick biases the gaze (legacy overlay)
JOYSTICK_GAIN = 0.8

def _apply_overlay_to_eye(target, center, amin, amax, stick_value):
    if stick_value == 0.0:
        return _clamp_to_range(target, amin, amax)
    if stick_value > 0:
        extra = stick_value * JOYSTICK_GAIN * (amax - center)
        return _clamp_to_range(target + extra, amin, amax)
    else:
        extra = stick_value * JOYSTICK_GAIN * (center - amin)
        return _clamp_to_range(target + extra, amin, amax)

def set_lids(lb, lt, rb, rt):
    with JO.lock:
        if JO.enabled and JO.left_pressed:
            lb, lt = LEFT_BOTTOM_CLOSED, LEFT_TOP_CLOSED
        if JO.enabled and JO.right_pressed:
            rb, rt = RIGHT_BOTTOM_CLOSED, RIGHT_TOP_CLOSED
    _set_lids_hw(lb, lt, rb, rt)

def set_eyes(l_lr, l_ud, r_lr, r_ud):
    with JO.lock:
        if JO.enabled:
            l_lr = _apply_overlay_to_eye(l_lr, LEFT_EYE_CENTER_LR, LEFT_EYE_LEFT,  LEFT_EYE_RIGHT, JO.lx)
            l_ud = _apply_overlay_to_eye(l_ud, LEFT_EYE_CENTER_UD, LEFT_EYE_DOWN,  LEFT_EYE_UP,   JO.ly)
            r_lr = _apply_overlay_to_eye(r_lr, RIGHT_EYE_CENTER_LR, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT, JO.rx)
            r_ud = _apply_overlay_to_eye(r_ud, RIGHT_EYE_CENTER_UD, RIGHT_EYE_DOWN, RIGHT_EYE_UP,    JO.ry)
    _set_eyes_hw(l_lr, l_ud, r_lr, r_ud)

def park_eyes():
    set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
             RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)

# =========================
# Left-joystick state (for j modes)
# =========================
class LeftJoystickState:
    def __init__(self):
        self.lock = threading.Lock()
        self.lx = 0.0
        self.ly = 0.0
        self.left_pressed = False
        self.active = False

LJ = LeftJoystickState()

def left_joystick_reader(stop_event):
    """
    Reads ONLY the left joystick (J1) and updates LJ.*.
    Ignores the right stick and its button entirely.
    """
    ser = js_open_port()
    if ser is None:
        with LJ.lock:
            LJ.active = False
        return

    def dz(x, thr): return 0.0 if abs(x) < thr else x
    lx = ly = 0.0

    with LJ.lock:
        LJ.active = True

    try:
        while not stop_event.is_set():
            line = ser.readline().decode(errors="ignore")
            if not line:
                pausable_sleep(0.005, stop_event)
                continue
            vals = js_parse_line(line)
            if not vals:
                continue

            j1x, j1y, j1sw, *_ = vals
            j1x_n = js_norm(j1x); j1y_n = js_norm(j1y)
            j1x_n = dz(j1x_n, DZ_J1); j1y_n = dz(j1y_n, DZ_J1)

            # Swap/invert per tuning
            if SWAP_LEFT_AXES:
                j1x_n, j1y_n = j1y_n, j1x_n
            if INVERT_LX: j1x_n = -j1x_n
            if INVERT_LY: j1y_n = -j1y_n

            lx = _lpf(lx, j1x_n)
            ly = _lpf(ly, j1y_n)

            with LJ.lock:
                LJ.lx = lx
                LJ.ly = ly
                LJ.left_pressed = (j1sw == 0)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass
        with LJ.lock:
            LJ.active = False

def _map_left_stick_to_both_eyes(lx, ly):
    l_lr = _map_norm_to_servo(lx, LEFT_EYE_CENTER_LR,  LEFT_EYE_LEFT,  LEFT_EYE_RIGHT)
    l_ud = _map_norm_to_servo(ly, LEFT_EYE_CENTER_UD,  LEFT_EYE_DOWN,  LEFT_EYE_UP)
    r_lr = _map_norm_to_servo(lx, RIGHT_EYE_CENTER_LR, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT)
    r_ud = _map_norm_to_servo(ly, RIGHT_EYE_CENTER_UD, RIGHT_EYE_DOWN, RIGHT_EYE_UP)
    return l_lr, l_ud, r_lr, r_ud

# =========================
# Motion Primitives (pausable)
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
        set_lids(LEFT_BOTTOM_CLOSED, LEFT_TOP_CLOSED,
                 RIGHT_BOTTOM_CLOSED, RIGHT_TOP_CLOSED)
        sleep_with_tremor(CLOSE_TIME_S, stop_event, tremor_side)
        if stop_event.is_set(): return
        set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
                 RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
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

# --- Nystagmus helpers (pausable) ---
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
        set_eyes(l_lr, l_ud, r_lr, r_ud)
        dt = min(NYSTAGMUS_STEP_S, duration - t)
        pausable_sleep(dt, stop_event, quantum=dt)
        t += dt

def blink_with_nystagmus(stop_event, orientation='horizontal'):
    set_lids(LEFT_BOTTOM_CLOSED, LEFT_TOP_CLOSED,
             RIGHT_BOTTOM_CLOSED, RIGHT_TOP_CLOSED)
    hold_with_nystagmus(CLOSE_TIME_S, stop_event, orientation)
    if stop_event.is_set(): return
    set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
             RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
    hold_with_nystagmus(OPEN_TIME_S, stop_event, orientation)

# =========================
# Sequences (pausable)
# =========================
def run_nystagmus_sequence(stop_event, orientation='horizontal'):
    if stop_event.is_set(): return
    set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
             RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)

    if stop_event.is_set(): return
    set_eyes(LEFT_EYE_LEFT, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_LEFT, RIGHT_EYE_CENTER_UD)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    if stop_event.is_set(): return
    set_eyes(LEFT_EYE_RIGHT, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_RIGHT, RIGHT_EYE_CENTER_UD)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    if stop_event.is_set(): return
    blink_with_nystagmus(stop_event, orientation)

    if stop_event.is_set(): return
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_UP,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_UP)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    if stop_event.is_set(): return
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_DOWN,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_DOWN)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    if stop_event.is_set(): return
    blink_with_nystagmus(stop_event, orientation)

def run_sequence(stop_event, *, stroke_side=None, tremor_side=None, dysconj_side=None):
    if stop_event.is_set(): return

    if stroke_side is None:
        set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
                 RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
        l_lr_base = LEFT_EYE_INWARD_LR if dysconj_side == 'left' else LEFT_EYE_CENTER_LR
        r_lr_base = RIGHT_EYE_INWARD_LR if dysconj_side == 'right' else RIGHT_EYE_CENTER_LR
        set_eyes(l_lr_base, LEFT_EYE_CENTER_UD,
                 r_lr_base, RIGHT_EYE_CENTER_UD)
    elif stroke_side == 'left':
        set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_CLOSED,
                 RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
        set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_DOWN,
                 RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)
    elif stroke_side == 'right':
        set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
                 RIGHT_BOTTOM_OPEN, RIGHT_TOP_CLOSED)
        set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
                 RIGHT_EYE_CENTER_LR, RIGHT_EYE_DOWN)
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

    set_eyes(L_LR(LEFT_EYE_LEFT),  L_UD(LEFT_EYE_CENTER_UD),
             R_LR(RIGHT_EYE_LEFT), R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    if stop_event.is_set(): return

    set_eyes(L_LR(LEFT_EYE_RIGHT),  L_UD(LEFT_EYE_CENTER_UD),
             R_LR(RIGHT_EYE_RIGHT), R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    if stop_event.is_set(): return

    set_eyes(LEFT_EYE_CENTER_LR, L_UD(LEFT_EYE_CENTER_UD),
             RIGHT_EYE_CENTER_LR, R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    if stop_event.is_set(): return

    blink(stop_event, stroke_side=stroke_side, tremor_side=tremor_side)
    if stop_event.is_set(): return

    set_eyes(L_LR(LEFT_EYE_CENTER_LR), L_UD(LEFT_EYE_UP),
             R_LR(RIGHT_EYE_CENTER_LR), R_UD(RIGHT_EYE_UP))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    if stop_event.is_set(): return

    set_eyes(L_LR(LEFT_EYE_CENTER_LR), L_UD(LEFT_EYE_DOWN),
             R_LR(RIGHT_EYE_CENTER_LR), R_UD(RIGHT_EYE_DOWN))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    if stop_event.is_set(): return

    set_eyes(LEFT_EYE_CENTER_LR, L_UD(LEFT_EYE_CENTER_UD),
             RIGHT_EYE_CENTER_LR, R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, stop_event, tremor_side)
    if stop_event.is_set(): return

    blink(stop_event, stroke_side=stroke_side, tremor_side=tremor_side)

# =========================
# Mode wrappers (loop until stopped)
# =========================
def run_normal(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, stroke_side=None, tremor_side=None, dysconj_side=None)

def run_horizontal_nystagmus(stop_event):
    while not stop_event.is_set():
        run_nystagmus_sequence(stop_event, orientation='horizontal')

def run_vertical_nystagmus(stop_event):
    while not stop_event.is_set():
        run_nystagmus_sequence(stop_event, orientation='vertical')

def run_left_stroke(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, stroke_side='left', tremor_side=None, dysconj_side=None)

def run_left_tremor(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, stroke_side=None, tremor_side='left', dysconj_side=None)

def run_left_dysconj(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, stroke_side=None, tremor_side=None, dysconj_side='left')

def run_right_stroke(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, stroke_side='right', tremor_side=None, dysconj_side=None)

def run_right_tremor(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, stroke_side=None, tremor_side='right', dysconj_side=None)

def run_right_dysconj(stop_event):
    while not stop_event.is_set():
        run_sequence(stop_event, stroke_side=None, tremor_side=None, dysconj_side='right')

# =========================
# Joystick helpers (reader + modes)
# =========================
def js_open_port():
    for p in PORTS_TO_TRY:
        try:
            s = serial.Serial(p, BAUD, timeout=0.05)
            sleep(0.2)
            s.reset_input_buffer()
            return s
        except Exception:
            pass
    print("[ERR] Could not open joystick serial port. Try: ls /dev/ttyUSB* /dev/ttyACM*  (and check dmesg | tail)")
    return None

def js_parse_line(line):
    parts = line.strip().split(",")
    if len(parts) != 6:
        return None
    try:
        j1x, j1y, j1sw, j2x, j2y, j2sw = map(int, parts)
        return j1x, j1y, j1sw, j2x, j2y, j2sw
    except ValueError:
        return None

def js_norm(v, max_adc=ADC_MAX):
    return (v / max_adc) * 2.0 - 1.0  # 0..max -> -1..+1

def _lpf(prev, new, alpha=LPF_ALPHA):
    return prev + alpha * (new - prev)

def _map_norm_to_servo(val, center, amin, amax):
    if val >= 0:
        return _clamp_to_range(center + val * (amax - center), amin, amax)
    else:
        return _clamp_to_range(center + val * (center - amin), amin, amax)

def run_joystick(stop_event):
    """
    Dedicated joystick-only mode (original behavior, both sticks).
    """
    ser = js_open_port()
    if ser is None:
        while not stop_event.is_set():
            pausable_sleep(0.2, stop_event)
        return

    DO_CYCLING_BLINK = True  # while holding button: cycle close/open

    l_x = l_y = 0.0
    r_x = r_y = 0.0
    l_blink_t = 0.0
    r_blink_t = 0.0
    last_tick = time()
    last_print = 0.0

    park_eyes()

    def dz(x, thr): return 0.0 if abs(x) < thr else x

    try:
        while not stop_event.is_set():
            now = time()
            dt = max(0.0, now - last_tick)
            last_tick = now

            line = ser.readline().decode(errors="ignore")
            if not line:
                pausable_sleep(0.005, stop_event)
                continue

            vals = js_parse_line(line)
            if not vals:
                continue

            j1x, j1y, j1sw, j2x, j2y, j2sw = vals

            # Normalize −1..+1
            j1x_n = js_norm(j1x); j1y_n = js_norm(j1y)
            j2x_n = js_norm(j2x); j2y_n = js_norm(j2y)

            # Deadzone
            j1x_n = dz(j1x_n, DZ_J1); j1y_n = dz(j1y_n, DZ_J1)
            j2x_n = dz(j2x_n, DZ_J2); j2y_n = dz(j2y_n, DZ_J2)

            # Axis swap
            if SWAP_LEFT_AXES:
                j1x_n, j1y_n = j1y_n, j1x_n
            if SWAP_RIGHT_AXES:
                j2x_n, j2y_n = j2y_n, j2x_n

            # Per-axis inversion AFTER swap
            if INVERT_LX: j1x_n = -j1x_n
            if INVERT_LY: j1y_n = -j1y_n
            if INVERT_RX: j2x_n = -j2x_n
            if INVERT_RY: j2y_n = -j2y_n

            # Low-pass filter
            l_x = _lpf(l_x, j1x_n)
            l_y = _lpf(l_y, j1y_n)
            r_x = _lpf(r_x, j2x_n)
            r_y = _lpf(r_y, j2y_n)

            # Map to servo angles with each eye's limits
            l_lr = _map_norm_to_servo(l_x, LEFT_EYE_CENTER_LR,  LEFT_EYE_LEFT,  LEFT_EYE_RIGHT)
            l_ud = _map_norm_to_servo(l_y, LEFT_EYE_CENTER_UD,  LEFT_EYE_DOWN,  LEFT_EYE_UP)
            r_lr = _map_norm_to_servo(r_x, RIGHT_EYE_CENTER_LR, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT)
            r_ud = _map_norm_to_servo(r_y, RIGHT_EYE_CENTER_UD, RIGHT_EYE_DOWN, RIGHT_EYE_UP)

            _set_eyes_hw(l_lr, l_ud, r_lr, r_ud)  # direct control in this mode

            # Buttons (active-low)
            left_pressed  = (j1sw == 0)
            right_pressed = (j2sw == 0)

            # Blink behavior
            if DO_CYCLING_BLINK:
                l_blink_t = (l_blink_t + dt) if left_pressed else 0.0
                r_blink_t = (r_blink_t + dt) if right_pressed else 0.0

                def side_lids(pressed, phase_t, b_open, b_closed, t_open, t_closed):
                    if not pressed:
                        return b_open, t_open
                    t_in_cycle = phase_t % BLINK_PERIOD_S
                    if t_in_cycle < CLOSE_TIME_S:
                        return b_closed, t_closed
                    else:
                        return b_open,  t_open

                lb, lt = side_lids(left_pressed,  l_blink_t,
                                    LEFT_BOTTOM_OPEN, LEFT_BOTTOM_CLOSED,
                                    LEFT_TOP_OPEN,    LEFT_TOP_CLOSED)
                rb, rt = side_lids(right_pressed, r_blink_t,
                                    RIGHT_BOTTOM_OPEN, RIGHT_BOTTOM_CLOSED,
                                    RIGHT_TOP_OPEN,    RIGHT_TOP_CLOSED)
            else:
                lb = LEFT_BOTTOM_CLOSED if left_pressed else LEFT_BOTTOM_OPEN
                lt = LEFT_TOP_CLOSED    if left_pressed else LEFT_TOP_OPEN
                rb = RIGHT_BOTTOM_CLOSED if right_pressed else RIGHT_BOTTOM_OPEN
                rt = RIGHT_TOP_CLOSED    if right_pressed else RIGHT_TOP_OPEN

            _set_lids_hw(lb, lt, rb, rt)

            if PRINT_STATUS and (now - last_print >= PRINT_STATUS):
                last_print = now
                print(
                    f"L eye: LR={l_lr:6.1f} UD={l_ud:6.1f} | "
                    f"R eye: LR={r_lr:6.1f} UD={r_ud:6.1f} | "
                    f"Lbtn={int(left_pressed)} Rbtn={int(right_pressed)}"
                )

    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass

def joystick_overlay_thread(stop_event):
    """
    Legacy background reader for overlay-enabled scripted modes.
    'j' realtime modes do NOT use this overlay path.
    """
    ser = js_open_port()
    if ser is None:
        with JO.lock:
            JO.enabled = False
        return

    l_x = l_y = 0.0
    r_x = r_y = 0.0

    def dz(x, thr): return 0.0 if abs(x) < thr else x

    try:
        while not stop_event.is_set():
            line = ser.readline().decode(errors="ignore")
            if not line:
                pausable_sleep(0.005, stop_event)
                continue

            vals = js_parse_line(line)
            if not vals:
                continue

            j1x, j1y, j1sw, j2x, j2y, j2sw = vals

            j1x_n = js_norm(j1x); j1y_n = js_norm(j1y)
            j2x_n = js_norm(j2x); j2y_n = js_norm(j2y)

            # deadzone
            j1x_n = dz(j1x_n, DZ_J1); j1y_n = dz(j1y_n, DZ_J1)
            j2x_n = dz(j2x_n, DZ_J2); j2y_n = dz(j2y_n, DZ_J2)

            # swap
            if SWAP_LEFT_AXES:
                j1x_n, j1y_n = j1y_n, j1x_n
            if SWAP_RIGHT_AXES:
                j2x_n, j2y_n = j2y_n, j2x_n

            # invert
            if INVERT_LX: j1x_n = -j1x_n
            if INVERT_LY: j1y_n = -j1y_n
            if INVERT_RX: j2x_n = -j2x_n
            if INVERT_RY: j2y_n = -j2y_n

            # smooth
            l_x = _lpf(l_x, j1x_n)
            l_y = _lpf(l_y, j1y_n)
            r_x = _lpf(r_x, j2x_n)
            r_y = _lpf(r_y, j2y_n)

            with JO.lock:
                JO.lx, JO.ly, JO.rx, JO.ry = l_x, l_y, r_x, r_y
                JO.left_pressed  = (j1sw == 0)
                JO.right_pressed = (j2sw == 0)
                JO.enabled = True
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass

# =========================
# Left-stick realtime modes (for 1j..9j)
# =========================
def run_leftstick_realtime(stop_event, *, feature="none"):
    """
    Left joystick drives BOTH eyes and BOTH lids in realtime (right stick ignored).
    'feature' adds the mode-specific effect on top of live control.
    """
    # Start centered & open
    park_eyes()

    # Phase/time for oscillatory effects
    t0 = time()
    l_blink_t = 0.0

    # Helper: read a consistent snapshot from LJ
    def read_LJ():
        with LJ.lock:
            return LJ.lx, LJ.ly, LJ.left_pressed, LJ.active

    # Small helpers for features
    def add_nystag(lr, ud, orientation, t):
        phase = sin(2 * pi * NYSTAGMUS_FREQ_HZ * t)
        if orientation == "horizontal":
            llr_base, llr_amp = _biased_base_and_amp(lr[0], LEFT_EYE_LEFT,  LEFT_EYE_RIGHT,  NYSTAGMUS_AMPLITUDE_DEG)
            rlr_base, rlr_amp = _biased_base_and_amp(lr[1], RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT, NYSTAGMUS_AMPLITUDE_DEG)
            l_lr = _clamp_to_range(llr_base + llr_amp * phase, LEFT_EYE_LEFT, LEFT_EYE_RIGHT)
            r_lr = _clamp_to_range(rlr_base + rlr_amp * phase, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT)
            return (l_lr, r_lr), ud
        else:  # vertical
            lud_base, lud_amp = _biased_base_and_amp(ud[0], LEFT_EYE_DOWN,  LEFT_EYE_UP,  NYSTAGMUS_AMPLITUDE_DEG)
            rud_base, rud_amp = _biased_base_and_amp(ud[1], RIGHT_EYE_DOWN, RIGHT_EYE_UP, NYSTAGMUS_AMPLITUDE_DEG)
            l_ud = _clamp_to_range(lud_base + lud_amp * phase, LEFT_EYE_DOWN, LEFT_EYE_UP)
            r_ud = _clamp_to_range(rud_base + rud_amp * phase, RIGHT_EYE_DOWN, RIGHT_EYE_UP)
            return lr, (l_ud, r_ud)

    last_tick = time()

    try:
        while not stop_event.is_set():
            now = time()
            dt = max(0.0, now - last_tick)
            last_tick = now
            t = now - t0

            # Require left-joystick reader to be active
            lx, ly, left_pressed, lj_active = read_LJ()
            if not lj_active:
                pausable_sleep(0.01, stop_event)
                continue

            # Map left stick [-1,1] -> both eyes' angles
            l_lr, l_ud, r_lr, r_ud = _map_left_stick_to_both_eyes(lx, ly)

            # Feature overlays
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

            if feature == "nystag_h":
                (l_lr, r_lr), (l_ud, r_ud) = add_nystag((l_lr, r_lr), (l_ud, r_ud), "horizontal", t)
            elif feature == "nystag_v":
                (l_lr, r_lr), (l_ud, r_ud) = add_nystag((l_lr, r_lr), (l_ud, r_ud), "vertical", t)

            # Blink from left button (active-low) — closes BOTH lids while held
            if left_pressed:
                l_blink_t += dt
                t_in = (l_blink_t % BLINK_PERIOD_S)
                closed = (t_in < CLOSE_TIME_S)
            else:
                l_blink_t = 0.0
                closed = False

            if closed:
                lb = LEFT_BOTTOM_CLOSED; lt = LEFT_TOP_CLOSED
                rb = RIGHT_BOTTOM_CLOSED; rt = RIGHT_TOP_CLOSED
            else:
                lb = left_bottom_cmd;  lt = left_top_cmd
                rb = right_bottom_cmd; rt = right_top_cmd

            # Tremor overlays only when not closed
            if not closed:
                if feature == "tremor_left":
                    trem = TREMOR_AMPLITUDE_DEG * sin(2 * pi * TREMOR_FREQ_HZ * t)
                    lb = _clamp_angle(LEFT_BOTTOM_OPEN + trem)
                elif feature == "tremor_right":
                    trem = TREMOR_AMPLITUDE_DEG * sin(2 * pi * TREMOR_FREQ_HZ * t)
                    rb = _clamp_angle(RIGHT_BOTTOM_OPEN + trem)

            set_eyes(l_lr, l_ud, r_lr, r_ud)
            _set_lids_hw(lb, lt, rb, rt)

            pausable_sleep(SLEEP_QUANTUM, stop_event)

    except KeyboardInterrupt:
        pass

def run_leftstick_wrapper(name, feature, stop_event):
    print(f"→ Switched to: {name} [LEFT joystick realtime]")
    reader_thread = threading.Thread(target=left_joystick_reader, args=(stop_event,), daemon=True)
    reader_thread.start()
    try:
        run_leftstick_realtime(stop_event, feature=feature)
    finally:
        pass

# =========================
# Interactive runner
# =========================
def run_mode_wrapper(name, func, stop_event, use_joystick_overlay=False):
    print(f"→ Switched to: {name}" + (" [joystick overlay]" if use_joystick_overlay else ""))
    overlay_thread = None
    if use_joystick_overlay:
        overlay_thread = threading.Thread(target=joystick_overlay_thread, args=(stop_event,), daemon=True)
        overlay_thread.start()
    try:
        func(stop_event)
    finally:
        if use_joystick_overlay:
            with JO.lock:
                JO.enabled = False  # stop applying overlay immediately

MODES = {
    "0": ("Joystick control (direct)", run_joystick),  # legacy direct mode
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

# For number+j: left-stick realtime control + feature
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

HELP_TEXT = """\
Choose a mode:
  0  - joystick control (direct, both sticks, no pattern)
  1  - normal
  2  - horizontal nystagmus
  3  - vertical nystagmus
  4  - left side stroke
  5  - left side tremor
  6  - left side dysconj
  7  - right side stroke
  8  - right side tremor
  9  - right side dysconj
  q  - quit

Tip: add 'j' to any number to enable LEFT-joystick realtime control for both eyes + lids,
with only that number's feature layered on top (no scripted look/blink loop).
Examples: 1j (plain left-stick), 2j (left-stick + horizontal nystagmus), 8j (left-stick + right bottom eyelid tremor)
"""

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

        # Stop current (fast)
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

        # Start new thread/event
        self.stop_event = threading.Event()

        if use_j and base_key != "0":
            if base_key not in J_MODES:
                print("Unsupported j-mode:", key)
                return
            name, feature = J_MODES[base_key]
            self.thread = threading.Thread(
                target=run_leftstick_wrapper,
                args=(name, feature, self.stop_event),
                daemon=True
            )
        else:
            name, func = MODES[base_key]
            self.thread = threading.Thread(
                target=run_mode_wrapper,
                args=(name, func, self.stop_event, False),
                daemon=True
            )

        self.thread.start()
        self.current_key = key

    def stop(self):
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.thread = None
        self.current_key = None

def main():
    park_eyes()
    sleep(0.5)

    runner = PatternRunner()
    print(HELP_TEXT)

    while True:
        try:
            sel = input("Enter mode (0-9, append 'j' for LEFT-stick realtime; q to quit): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            sel = "q"

        if sel == "q":
            print("Shutting down…")
            runner.stop()
            break

        if sel.rstrip('j') in MODES:
            runner.start_mode(sel)
        else:
            print("Invalid choice.")
            print(HELP_TEXT)

    park_eyes()

if __name__ == "__main__":
    main()
