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
# Left eye (existing)
left_eye_lr    = kit.servo[0]  # left-right
left_eye_ud    = kit.servo[1]  # up-down
# Right eye (new)
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
# Ranges you provided:
#   L/R: 60–160 (center 110)
#   U/D: 110–160 (center 125)
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

# --- Functions ---
def set_lids(lb, lt, rb, rt):
    left_bottom.angle  = lb
    left_top.angle     = lt
    right_bottom.angle = rb
    right_top.angle    = rt

def blink():
    set_lids(LEFT_BOTTOM_CLOSED, LEFT_TOP_CLOSED,
             RIGHT_BOTTOM_CLOSED, RIGHT_TOP_CLOSED)
    sleep(CLOSE_TIME_S)
    set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
             RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
    sleep(OPEN_TIME_S)

def set_eyes(l_lr, l_ud, r_lr, r_ud):
    left_eye_lr.angle  = l_lr
    left_eye_ud.angle  = l_ud
    right_eye_lr.angle = r_lr
    right_eye_ud.angle = r_ud

# --- Start positions ---
set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
         RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
         RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)
sleep(0.5)

# --- Loop ---
while True:
    # 1) Left/right: move LEFT
    set_eyes(LEFT_EYE_LEFT, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_LEFT, RIGHT_EYE_CENTER_UD)
    sleep(1.0)

    # 2) Left/right: move RIGHT
    set_eyes(LEFT_EYE_RIGHT, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_RIGHT, RIGHT_EYE_CENTER_UD)
    sleep(1.0)

    # 3) Left/right: move CENTER
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)
    sleep(1.0)

    # 4) Blink
    blink()
    sleep(1.0)

    # 5) Up/down: move UP
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_UP,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_UP)
    sleep(1.0)

    # 6) Up/down: move DOWN
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_DOWN,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_DOWN)
    sleep(1.0)

    # 7) Up/down: move CENTER
    set_eyes(LEFT_EYE_CENTER_LR, LEFT_EYE_CENTER_UD,
             RIGHT_EYE_CENTER_LR, RIGHT_EYE_CENTER_UD)
    sleep(1.0)

    # 8) Blink
    blink()
    sleep(1.0)

