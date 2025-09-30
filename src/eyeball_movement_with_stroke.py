from time import sleep
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
# Ranges you provided: L/R 60–160 (center 110); U/D 110–160 (center 125)
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

# --- Helpers ---
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

def blink(stroke_side=None):
    """
    stroke_side:
      None   -> normal blink, both eyes close/open
      'left' -> left TOP stays closed; left BOTTOM blinks; right blinks normally
      'right'-> right TOP stays closed; right BOTTOM blinks; left blinks normally
    """
    if stroke_side is None:
        set_lids(LEFT_BOTTOM_CLOSED, LEFT_TOP_CLOSED,
                 RIGHT_BOTTOM_CLOSED, RIGHT_TOP_CLOSED)
        sleep(CLOSE_TIME_S)
        set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
                 RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
        sleep(OPEN_TIME_S)

    elif stroke_side == 'left':
        # Close: keep LT closed; blink LB; right normal
        left_top.angle     = LEFT_TOP_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_CLOSED
        right_top.angle    = RIGHT_TOP_CLOSED
        sleep(CLOSE_TIME_S)
        # Open: LT stays closed; LB opens; right opens
        left_top.angle     = LEFT_TOP_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_OPEN
        right_bottom.angle = RIGHT_BOTTOM_OPEN
        right_top.angle    = RIGHT_TOP_OPEN
        sleep(OPEN_TIME_S)

    elif stroke_side == 'right':
        # Close: keep RT closed; blink RB; left normal
        right_top.angle    = RIGHT_TOP_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_CLOSED
        left_bottom.angle  = LEFT_BOTTOM_CLOSED
        left_top.angle     = LEFT_TOP_CLOSED
        sleep(CLOSE_TIME_S)
        # Open: RT stays closed; RB opens; left opens
        right_top.angle    = RIGHT_TOP_CLOSED
        right_bottom.angle = RIGHT_BOTTOM_OPEN
        left_bottom.angle  = LEFT_BOTTOM_OPEN
        left_top.angle     = LEFT_TOP_OPEN
        sleep(OPEN_TIME_S)

def run_sequence(stroke_side=None):
    """
    Runs the movement/blink sequence once.
    stroke_side:
      None   -> normal
      'left' -> left top lid closed, left bottom blinks; left eye DOWN & LR CENTER (no LR moves)
      'right'-> right top lid closed, right bottom blinks; right eye DOWN & LR CENTER (no LR moves)
    """
    # --- Baseline posture before sequence ---
    if stroke_side is None:
        set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
                 RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
        set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
                 RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)
    elif stroke_side == 'left':
        set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_CLOSED,
                 RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
        set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_DOWN,           # left frozen: center LR, DOWN UD
                 RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)    # right normal
    elif stroke_side == 'right':
        set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
                 RIGHT_BOTTOM_OPEN, RIGHT_TOP_CLOSED)
        set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,      # left normal
                 RIGHT_EYE_CENTER_LR, RIGHT_EYE_DOWN)         # right frozen: center LR, DOWN UD
    sleep(0.2)

    # Utility to fetch LR/UD for each eye depending on stroke
    def L_LR(left, center):
        return center if stroke_side == 'left' else left
    def R_LR(right, center):
        return center if stroke_side == 'right' else right
    def L_UD(updown, center):
        return LEFT_EYE_DOWN if stroke_side == 'left' else (updown if updown is not None else center)
    def R_UD(updown, center):
        return RIGHT_EYE_DOWN if stroke_side == 'right' else (updown if updown is not None else center)

    # --- Left/Right moves ---
    # Move LEFT
    set_eyes(
        L_LR(LEFT_EYE_LEFT, LEFT_EYE_CENTER_LR),  L_UD(None, LEFT_EYE_CENTER_UD),
        R_LR(RIGHT_EYE_LEFT, RIGHT_EYE_CENTER_LR), R_UD(None, RIGHT_EYE_CENTER_UD)
    )
    sleep(1.0)

    # Move RIGHT
    set_eyes(
        L_LR(LEFT_EYE_RIGHT, LEFT_EYE_CENTER_LR), L_UD(None, LEFT_EYE_CENTER_UD),
        R_LR(RIGHT_EYE_RIGHT, RIGHT_EYE_CENTER_LR), R_UD(None, RIGHT_EYE_CENTER_UD)
    )
    sleep(1.0)

    # LR CENTER
    set_eyes(
        LEFT_EYE_CENTER_LR, L_UD(None, LEFT_EYE_CENTER_UD),
        RIGHT_EYE_CENTER_LR, R_UD(None, RIGHT_EYE_CENTER_UD)
    )
    sleep(1.0)

    # Blink
    blink(stroke_side=stroke_side)
    sleep(1.0)

    # --- Up/Down moves ---
    # Move UP
    set_eyes(
        LEFT_EYE_CENTER_LR, L_UD(LEFT_EYE_UP, LEFT_EYE_CENTER_UD),
        RIGHT_EYE_CENTER_LR, R_UD(RIGHT_EYE_UP, RIGHT_EYE_CENTER_UD)
    )
    sleep(1.0)

    # Move DOWN
    set_eyes(
        LEFT_EYE_CENTER_LR, L_UD(LEFT_EYE_DOWN, LEFT_EYE_CENTER_UD),
        RIGHT_EYE_CENTER_LR, R_UD(RIGHT_EYE_DOWN, RIGHT_EYE_CENTER_UD)
    )
    sleep(1.0)

    # UD CENTER (affected side stays DOWN)
    set_eyes(
        LEFT_EYE_CENTER_LR, L_UD(None, LEFT_EYE_CENTER_UD),
        RIGHT_EYE_CENTER_LR, R_UD(None, RIGHT_EYE_CENTER_UD)
    )
    sleep(1.0)

    # Blink
    blink(stroke_side=stroke_side)
    sleep(1.0)

# --- Start positions ---
set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
         RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
         RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)
sleep(0.5)

# --- Loop: normal → stroke left → normal → stroke right → repeat ---
while True:
    run_sequence(stroke_side=None)     # normal
    run_sequence(stroke_side='left')   # stroke left
    run_sequence(stroke_side=None)     # normal
    run_sequence(stroke_side='right')  # stroke right
