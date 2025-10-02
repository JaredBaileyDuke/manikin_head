#!/usr/bin/env python3
from time import sleep, time
from math import sin, pi
import threading
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
# "Inward" = toward the nose: left eye turns RIGHT; right eye turns LEFT.
LEFT_EYE_INWARD_LR  = 150
RIGHT_EYE_INWARD_LR = 80

# Blink timing
BLINK_PERIOD_S = 1.0
CLOSE_TIME_S   = 0.15
OPEN_TIME_S    = BLINK_PERIOD_S - CLOSE_TIME_S

# Tremor settings (bottom eyelid only)
TREMOR_AMPLITUDE_DEG = 10.0   # ±10°
TREMOR_FREQ_HZ       = 12.0
TREMOR_STEP_S        = 0.03

# Nystagmus settings (both eyes together)
NYSTAGMUS_AMPLITUDE_DEG = 10.0   # target ±20°
NYSTAGMUS_FREQ_HZ       = 2.5    # cycles per second
NYSTAGMUS_STEP_S        = 0.02   # update step
NYSTAGMUS_HOLD_S        = 2.5    # longer hold at each gaze direction

# Sleep quantum for responsiveness
SLEEP_QUANTUM = 0.05  # 50 ms

# =========================
# Utilities
# =========================
def pausable_sleep(total_sec: float, stop_event: threading.Event, quantum: float = SLEEP_QUANTUM):
    """Sleep up to total_sec but wake early if stop_event is set."""
    deadline = time() + total_sec
    while time() < deadline and not stop_event.is_set():
        sleep(min(quantum, max(0, deadline - time())))

def _clamp_angle(a):
    return max(0, min(180, a))

def _clamp_to_range(a, amin, amax):
    return max(amin, min(amax, a))

def _biased_base_and_amp(base, amin, amax, desired_amp):
    """
    Ensure non-zero oscillation even when the base is near/exactly at a limit by
    biasing the base inward enough to allow the amplitude.
    """
    amp = min(desired_amp, (amax - amin) / 2)
    if base - amin < amp:
        base = amin + amp
    elif amax - base < amp:
        base = amax - amp
    return base, amp

def set_lids(lb, lt, rb, rt):
    left_bottom.angle  = lb
    left_top.angle     = lt
    right_bottom.angle = rb
    right_top.angle    = rt

def set_eyes(l_lr, l_ud, r_lr, r_ud):
    left_eye_lr.angle  = l_lr
    left_eye_ud.angle  = l_ud
    right_eye_lr.angle = r_lr
    right_eye_ud.angle = r_ud

# =========================
# Motion Primitives (now pausable)
# =========================
def sleep_with_tremor(duration, stop_event, tremor_side=None):
    """Sleep while optionally adding eyelid tremor to the chosen bottom lid."""
    if stop_event.is_set():
        return

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
        pausable_sleep(dt, stop_event, quantum=dt)  # keep phase smooth
        t += dt
    servo.angle = _clamp_angle(base)

def blink(stop_event, stroke_side=None, tremor_side=None):
    """Blink; tremor (if any) continues during close/open waits."""
    if stop_event.is_set():
        return

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
    """
    Hold current gaze while applying nystagmus oscillation for 'duration' seconds.
    Works even at extreme gazes by biasing the base inward to allow swing.
    orientation: 'horizontal' or 'vertical'
    """
    # Current base angles
    l_lr_base = left_eye_lr.angle if left_eye_lr.angle is not None else LEFT_EYE_CENTER_LR
    r_lr_base = right_eye_lr.angle if right_eye_lr.angle is not None else RIGHT_EYE_CENTER_LR
    l_ud_base = left_eye_ud.angle if left_eye_ud.angle is not None else LEFT_EYE_CENTER_UD
    r_ud_base = right_eye_ud.angle if right_eye_ud.angle is not None else RIGHT_EYE_CENTER_UD

    if orientation == 'horizontal':
        l_lr_base, l_lr_amp = _biased_base_and_amp(l_lr_base, LEFT_EYE_LEFT,  LEFT_EYE_RIGHT,  NYSTAGMUS_AMPLITUDE_DEG)
        r_lr_base, r_lr_amp = _biased_base_and_amp(r_lr_base, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT, NYSTAGMUS_AMPLITUDE_DEG)
        l_ud_amp = 0.0
        r_ud_amp = 0.0
    else:
        l_ud_base, l_ud_amp = _biased_base_and_amp(l_ud_base, LEFT_EYE_DOWN,  LEFT_EYE_UP,  NYSTAGMUS_AMPLITUDE_DEG)
        r_ud_base, r_ud_amp = _biased_base_and_amp(r_ud_base, RIGHT_EYE_DOWN, RIGHT_EYE_UP, NYSTAGMUS_AMPLITUDE_DEG)
        l_lr_amp = 0.0
        r_lr_amp = 0.0

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
# Sequences (now pausable + cooperative)
# =========================
def run_nystagmus_sequence(stop_event, orientation='horizontal'):
    """One full nystagmus pass: L -> R -> blink -> Up -> Down -> blink."""
    if stop_event.is_set(): return

    # Lids open
    set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
             RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)

    # 1) Look LEFT (LR to min, UD center)
    if stop_event.is_set(): return
    set_eyes(LEFT_EYE_LEFT, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_LEFT, RIGHT_EYE_CENTER_UD)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    # 2) Look RIGHT (LR to max, UD center)
    if stop_event.is_set(): return
    set_eyes(LEFT_EYE_RIGHT, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_RIGHT, RIGHT_EYE_CENTER_UD)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    # 3) Blink (nystagmus continues)
    if stop_event.is_set(): return
    blink_with_nystagmus(stop_event, orientation)

    # 4) Look UP (UD to max, LR center)
    if stop_event.is_set(): return
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_UP,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_UP)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    # 5) Look DOWN (UD to min, LR center)
    if stop_event.is_set(): return
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_DOWN,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_DOWN)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, stop_event, orientation)

    # 6) Blink (nystagmus continues)
    if stop_event.is_set(): return
    blink_with_nystagmus(stop_event, orientation)

def run_sequence(stop_event, *, stroke_side=None, tremor_side=None, dysconj_side=None):
    """One full 'normal' movement sequence (with optional stroke/tremor/dysconj)."""
    if stop_event.is_set(): return

    # Baseline posture
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

    # Helpers for constraints in special modes
    def L_LR(target):
        if stroke_side == 'left':
            return LEFT_EYE_CENTER_LR
        if dysconj_side == 'left':
            return LEFT_EYE_INWARD_LR
        return target

    def R_LR(target):
        if stroke_side == 'right':
            return RIGHT_EYE_CENTER_LR
        if dysconj_side == 'right':
            return RIGHT_EYE_INWARD_LR
        return target

    def L_UD(target):
        return LEFT_EYE_DOWN if stroke_side == 'left' else target

    def R_UD(target):
        return RIGHT_EYE_DOWN if stroke_side == 'right' else target

    # Left/Right moves
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

    # Blink (no extra wait; tremor runs during close/open)
    blink(stop_event, stroke_side=stroke_side, tremor_side=tremor_side)
    if stop_event.is_set(): return

    # Up/Down moves
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

    # Blink (no extra wait; tremor runs during close/open)
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
# Interactive runner
# =========================
MODES = {
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

HELP_TEXT = """\
Choose a mode:
  1 - normal
  2 - horizontal nystagmus
  3 - vertical nystagmus
  4 - left side stroke
  5 - left side tremor
  6 - left side dysconj
  7 - right side stroke
  8 - right side tremor
  9 - right side dysconj
  q - quit
"""

class PatternRunner:
    def __init__(self):
        self.thread = None
        self.stop_event = threading.Event()
        self.current_key = None

    def start_mode(self, key: str):
        if key not in MODES:
            print("Unknown mode:", key)
            return
        name, func = MODES[key]

        # Stop current (fast)
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

        # Start new
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=func, args=(self.stop_event,), daemon=True)
        self.thread.start()
        self.current_key = key
        print(f"→ Switched to: {name}")

    def stop(self):
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        self.thread = None
        self.current_key = None

def park_eyes():
    """Neutral, lids open."""
    set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
             RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)

def main():
    # Start positions
    park_eyes()
    sleep(0.5)

    runner = PatternRunner()
    print(HELP_TEXT)

    while True:
        try:
            sel = input("Enter mode (1-9, q to quit): ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            sel = "q"

        if sel == "q":
            print("Shutting down…")
            runner.stop()
            break

        if sel in MODES:
            runner.start_mode(sel)
        else:
            print("Invalid choice.")
            print(HELP_TEXT)

    # Park before exit
    park_eyes()

if __name__ == "__main__":
    main()
