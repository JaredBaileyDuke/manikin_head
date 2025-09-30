from time import sleep
from math import sin, pi
from adafruit_servokit import ServoKit

# --- Setup ---
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

# --- Calibrated positions ---
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
LEFT_EYE_LEFT       = 70
LEFT_EYE_RIGHT      = 180
LEFT_EYE_CENTER_UD  = 110
LEFT_EYE_UP         = 140
LEFT_EYE_DOWN       = 90

# Right eyeball (servo 4 = left-right, servo 5 = up-down)
RIGHT_EYE_CENTER_LR = 110
RIGHT_EYE_LEFT      = 60
RIGHT_EYE_RIGHT     = 160
RIGHT_EYE_CENTER_UD = 125
RIGHT_EYE_UP        = 160
RIGHT_EYE_DOWN      = 110

# Blink timing
BLINK_PERIOD_S = 1.0
CLOSE_TIME_S   = 0.15
OPEN_TIME_S    = BLINK_PERIOD_S - CLOSE_TIME_S

# Tremor settings
TREMOR_AMPLITUDE_DEG = 10.0   # larger tremor amplitude
TREMOR_FREQ_HZ       = 12.0
TREMOR_STEP_S        = 0.03

# --- Helpers ---
def _clamp_angle(a):
    return max(0, min(180, a))

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

def sleep_with_tremor(duration, tremor_side=None):
    """Sleep while optionally adding eyelid tremor."""
    if tremor_side not in ('left', 'right'):
        sleep(duration)
        return

    servo = left_bottom if tremor_side == 'left' else right_bottom
    base = servo.angle if servo.angle is not None else (
        LEFT_BOTTOM_OPEN if tremor_side == 'left' else RIGHT_BOTTOM_OPEN
    )

    t = 0.0
    while t < duration:
        offset = TREMOR_AMPLITUDE_DEG * sin(2 * pi * TREMOR_FREQ_HZ * t)
        servo.angle = _clamp_angle(base + offset)
        dt = min(TREMOR_STEP_S, duration - t)
        sleep(dt)
        t += dt
    servo.angle = _clamp_angle(base)

# Replace your blink() with this version
def blink(stroke_side=None, tremor_side=None):
    """Blink, with stroke options. Tremor (if set) continues during blink waits."""
    if stroke_side is None:
        # close
        set_lids(LEFT_BOTTOM_CLOSED, LEFT_TOP_CLOSED,
                 RIGHT_BOTTOM_CLOSED, RIGHT_TOP_CLOSED)
        sleep_with_tremor(CLOSE_TIME_S, tremor_side)
        # open
        set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
                 RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
        sleep_with_tremor(OPEN_TIME_S, tremor_side)

    elif stroke_side == 'left':
        # close (LT stays closed)
        left_top.angle     = LEFT_TOP_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_CLOSED
        right_top.angle    = RIGHT_TOP_CLOSED
        sleep_with_tremor(CLOSE_TIME_S, tremor_side)
        # open (LT stays closed)
        left_top.angle     = LEFT_TOP_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_OPEN
        right_bottom.angle = RIGHT_BOTTOM_OPEN
        right_top.angle    = RIGHT_TOP_OPEN
        sleep_with_tremor(OPEN_TIME_S, tremor_side)

    elif stroke_side == 'right':
        # close (RT stays closed)
        right_top.angle    = RIGHT_TOP_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_CLOSED
        left_top.angle     = LEFT_TOP_CLOSED
        sleep_with_tremor(CLOSE_TIME_S, tremor_side)
        # open (RT stays closed)
        right_top.angle    = RIGHT_TOP_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_OPEN
        left_bottom.angle  = LEFT_BOTTOM_OPEN
        left_top.angle     = LEFT_TOP_OPEN
        sleep_with_tremor(OPEN_TIME_S, tremor_side)


def run_sequence(stroke_side=None, tremor_side=None):
    """Run one full sequence of movements."""
    # Baseline
    if stroke_side is None:
        set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
                 RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
        set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
                 RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)
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
    sleep_with_tremor(0.2, tremor_side)

    # Helpers
    def L_LR(target): return LEFT_EYE_CENTER_LR if stroke_side == 'left' else target
    def R_LR(target): return RIGHT_EYE_CENTER_LR if stroke_side == 'right' else target
    def L_UD(target): return LEFT_EYE_DOWN if stroke_side == 'left' else target
    def R_UD(target): return RIGHT_EYE_DOWN if stroke_side == 'right' else target

    # --- Left/Right moves ---
    set_eyes(L_LR(LEFT_EYE_LEFT), L_UD(LEFT_EYE_CENTER_UD),
             R_LR(RIGHT_EYE_LEFT), R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, tremor_side)

    set_eyes(L_LR(LEFT_EYE_RIGHT), L_UD(LEFT_EYE_CENTER_UD),
             R_LR(RIGHT_EYE_RIGHT), R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, tremor_side)

    set_eyes(LEFT_EYE_CENTER_LR, L_UD(LEFT_EYE_CENTER_UD),
             RIGHT_EYE_CENTER_LR, R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, tremor_side)

    # Blink (no extra wait after)
    blink(stroke_side=stroke_side, tremor_side=tremor_side)

    # --- Up/Down moves ---
    set_eyes(L_LR(LEFT_EYE_CENTER_LR), L_UD(LEFT_EYE_UP),
             R_LR(RIGHT_EYE_CENTER_LR), R_UD(RIGHT_EYE_UP))
    sleep_with_tremor(1.0, tremor_side)

    set_eyes(L_LR(LEFT_EYE_CENTER_LR), L_UD(LEFT_EYE_DOWN),
             R_LR(RIGHT_EYE_CENTER_LR), R_UD(RIGHT_EYE_DOWN))
    sleep_with_tremor(1.0, tremor_side)

    set_eyes(L_LR(LEFT_EYE_CENTER_LR), L_UD(LEFT_EYE_CENTER_UD),
             R_LR(RIGHT_EYE_CENTER_LR), R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, tremor_side)

    # Blink (no extra wait after)
    blink(stroke_side=stroke_side, tremor_side=tremor_side)

# --- Start positions ---
set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
         RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
         RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)
sleep(0.5)

# --- Loop: normal → stroke left → left tremor → stroke right → right tremor ---
while True:
    run_sequence(stroke_side=None,   tremor_side=None)    # normal
    run_sequence(stroke_side='left', tremor_side=None)    # left stroke
    run_sequence(stroke_side=None,   tremor_side='left')  # left eyelid tremor
    run_sequence(stroke_side='right',tremor_side=None)    # right stroke
    run_sequence(stroke_side=None,   tremor_side='right') # right eyelid tremor
