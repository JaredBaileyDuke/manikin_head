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

# --- Helpers ---
def _clamp_angle(a):
    return max(0, min(180, a))

def _clamp_to_range(a, amin, amax):
    return max(amin, min(amax, a))

def _biased_base_and_amp(base, amin, amax, desired_amp):
    """
    Ensure non-zero oscillation even when the base is near/exactly at a limit by
    biasing the base inward enough to allow the amplitude.
    """
    # Never allow amp larger than half the range
    amp = min(desired_amp, (amax - amin) / 2)
    # If too close to min, push base up; if too close to max, pull base down
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

def sleep_with_tremor(duration, tremor_side=None):
    """Sleep while optionally adding eyelid tremor to the chosen bottom lid."""
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

def blink(stroke_side=None, tremor_side=None):
    """Blink; tremor (if any) continues during close/open waits."""
    if stroke_side is None:
        set_lids(LEFT_BOTTOM_CLOSED, LEFT_TOP_CLOSED,
                 RIGHT_BOTTOM_CLOSED, RIGHT_TOP_CLOSED)
        sleep_with_tremor(CLOSE_TIME_S, tremor_side)
        set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
                 RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
        sleep_with_tremor(OPEN_TIME_S, tremor_side)

    elif stroke_side == 'left':
        left_top.angle     = LEFT_TOP_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_CLOSED
        right_top.angle    = RIGHT_TOP_CLOSED
        sleep_with_tremor(CLOSE_TIME_S, tremor_side)
        left_top.angle     = LEFT_TOP_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_OPEN
        right_bottom.angle = RIGHT_BOTTOM_OPEN
        right_top.angle    = RIGHT_TOP_OPEN
        sleep_with_tremor(OPEN_TIME_S, tremor_side)

    elif stroke_side == 'right':
        right_top.angle    = RIGHT_TOP_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_CLOSED
        left_top.angle     = LEFT_TOP_CLOSED
        sleep_with_tremor(CLOSE_TIME_S, tremor_side)
        right_top.angle    = RIGHT_TOP_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_OPEN
        left_bottom.angle  = LEFT_BOTTOM_OPEN
        left_top.angle     = LEFT_TOP_OPEN
        sleep_with_tremor(OPEN_TIME_S, tremor_side)

# --- Nystagmus helpers ---
def hold_with_nystagmus(duration, orientation='horizontal'):
    """
    Hold current gaze while applying nystagmus oscillation for 'duration' seconds.
    Works even at extreme gazes by biasing the base inward to allow swing.
    Orientation: 'horizontal' or 'vertical'
    """
    # Current base angles
    l_lr_base = left_eye_lr.angle if left_eye_lr.angle is not None else LEFT_EYE_CENTER_LR
    r_lr_base = right_eye_lr.angle if right_eye_lr.angle is not None else RIGHT_EYE_CENTER_LR
    l_ud_base = left_eye_ud.angle if left_eye_ud.angle is not None else LEFT_EYE_CENTER_UD
    r_ud_base = right_eye_ud.angle if right_eye_ud.angle is not None else RIGHT_EYE_CENTER_UD

    # Compute per-eye base and amplitude so we never hit limits AND we still move at edges
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
    while t < duration:
        phase = sin(2 * pi * NYSTAGMUS_FREQ_HZ * t)
        l_lr = _clamp_to_range(l_lr_base + l_lr_amp * phase, LEFT_EYE_LEFT,  LEFT_EYE_RIGHT)
        r_lr = _clamp_to_range(r_lr_base + r_lr_amp * phase, RIGHT_EYE_LEFT, RIGHT_EYE_RIGHT)
        l_ud = _clamp_to_range(l_ud_base + l_ud_amp * phase, LEFT_EYE_DOWN,  LEFT_EYE_UP)
        r_ud = _clamp_to_range(r_ud_base + r_ud_amp * phase, RIGHT_EYE_DOWN, RIGHT_EYE_UP)

        set_eyes(l_lr, l_ud, r_lr, r_ud)
        dt = min(NYSTAGMUS_STEP_S, duration - t)
        sleep(dt)
        t += dt

def blink_with_nystagmus(orientation='horizontal'):
    """Blink while continuing nystagmus oscillation during close/open waits."""
    set_lids(LEFT_BOTTOM_CLOSED, LEFT_TOP_CLOSED,
             RIGHT_BOTTOM_CLOSED, RIGHT_TOP_CLOSED)
    hold_with_nystagmus(CLOSE_TIME_S, orientation)
    set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
             RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
    hold_with_nystagmus(OPEN_TIME_S, orientation)

def run_nystagmus_sequence(orientation='horizontal'):
    """
    Nystagmus for BOTH eyes together, while performing:
    look LEFT -> look RIGHT -> blink -> look UP -> look DOWN -> blink
    with longer holds and continuous oscillation.
    orientation: 'horizontal' or 'vertical'
    """
    # Lids open
    set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
             RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)

    # 1) Look LEFT (LR to min, UD center)
    set_eyes(LEFT_EYE_LEFT, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_LEFT, RIGHT_EYE_CENTER_UD)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, orientation)

    # 2) Look RIGHT (LR to max, UD center)
    set_eyes(LEFT_EYE_RIGHT, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_RIGHT, RIGHT_EYE_CENTER_UD)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, orientation)

    # 3) Blink (nystagmus continues)
    blink_with_nystagmus(orientation)

    # 4) Look UP (UD to max, LR center)
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_UP,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_UP)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, orientation)

    # 5) Look DOWN (UD to min, LR center)
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_DOWN,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_DOWN)
    hold_with_nystagmus(NYSTAGMUS_HOLD_S, orientation)

    # 6) Blink (nystagmus continues)
    blink_with_nystagmus(orientation)

# --- Movement sequences (stroke/tremor/dysconjugate) ---
def run_sequence(*, stroke_side=None, tremor_side=None, dysconj_side=None):
    """Run one full 'normal' movement sequence (with optional stroke/tremor/dysconjugate)."""
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
    sleep_with_tremor(0.2, tremor_side)

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
            return RIGHT_EE_INWARD_LR if False else RIGHT_EYE_INWARD_LR  # keep API consistent
        return target

    def L_UD(target):
        return LEFT_EYE_DOWN if stroke_side == 'left' else target

    def R_UD(target):
        return RIGHT_EYE_DOWN if stroke_side == 'right' else target

    # Left/Right moves
    set_eyes(L_LR(LEFT_EYE_LEFT),  L_UD(LEFT_EYE_CENTER_UD),
             R_LR(RIGHT_EYE_LEFT), R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, tremor_side)

    set_eyes(L_LR(LEFT_EYE_RIGHT),  L_UD(LEFT_EYE_CENTER_UD),
             R_LR(RIGHT_EYE_RIGHT), R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, tremor_side)

    set_eyes(LEFT_EYE_CENTER_LR, L_UD(LEFT_EYE_CENTER_UD),
             RIGHT_EYE_CENTER_LR, R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, tremor_side)

    # Blink (no extra wait; tremor runs during close/open)
    blink(stroke_side=stroke_side, tremor_side=tremor_side)

    # Up/Down moves
    set_eyes(L_LR(LEFT_EYE_CENTER_LR), L_UD(LEFT_EYE_UP),
             R_LR(RIGHT_EYE_CENTER_LR), R_UD(RIGHT_EYE_UP))
    sleep_with_tremor(1.0, tremor_side)

    set_eyes(L_LR(LEFT_EYE_CENTER_LR), L_UD(LEFT_EYE_DOWN),
             R_LR(RIGHT_EYE_CENTER_LR), R_UD(RIGHT_EYE_DOWN))
    sleep_with_tremor(1.0, tremor_side)

    set_eyes(L_LR(LEFT_EYE_CENTER_LR), L_UD(LEFT_EYE_CENTER_UD),
             R_LR(RIGHT_EYE_CENTER_LR), R_UD(RIGHT_EYE_CENTER_UD))
    sleep_with_tremor(1.0, tremor_side)

    # Blink (no extra wait; tremor runs during close/open)
    blink(stroke_side=stroke_side, tremor_side=tremor_side)

# --- Start positions ---
set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
         RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
         RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)
sleep(0.5)

# --- Loop:
# normal → horizontal nystagmus → vertical nystagmus →
# left stroke → left tremor → left dysconjugate →
# right stroke → right tremor → right dysconjugate → repeat
while True:
    run_sequence(stroke_side=None,    tremor_side=None,    dysconj_side=None)      # normal
    run_nystagmus_sequence(orientation='horizontal')                                # both eyes, horizontal nystagmus + gaze sequence
    run_nystagmus_sequence(orientation='vertical')                                  # both eyes, vertical nystagmus + gaze sequence
    run_sequence(stroke_side='left',  tremor_side=None,    dysconj_side=None)      # left stroke
    run_sequence(stroke_side=None,    tremor_side='left',  dysconj_side=None)      # left eyelid tremor
    run_sequence(stroke_side=None,    tremor_side=None,    dysconj_side='left')    # left dysconjugate gaze
    run_sequence(stroke_side='right', tremor_side=None,    dysconj_side=None)      # right stroke
    run_sequence(stroke_side=None,    tremor_side='right', dysconj_side=None)      # right eyelid tremor
    run_sequence(stroke_side=None,    tremor_side=None,    dysconj_side='right')   # right dysconjugate gaze
