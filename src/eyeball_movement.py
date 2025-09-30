from time import sleep
from adafruit_servokit import ServoKit

# --- Setup ---
kit = ServoKit(channels=16)

# Left eyelids
left_top = kit.servo[2]
left_bottom    = kit.servo[3]

# Right eyelids
right_top = kit.servo[6]
right_bottom    = kit.servo[7]

# Match your servos' pulse range
for s in (left_bottom, left_top, right_bottom, right_top):
    s.set_pulse_width_range(500, 2500)

# --- Calibrated positions ---
# Left
LEFT_TOP_OPEN           = 110  # 130
LEFT_TOP_CLOSED         = 30
LEFT_BOTTOM_OPEN        = 35
LEFT_BOTTOM_CLOSED      = 130  # 125

# Right
RIGHT_TOP_OPEN          = 100  # 130
RIGHT_TOP_CLOSED        = 30
RIGHT_BOTTOM_OPEN       = 45
RIGHT_BOTTOM_CLOSED     = 135

# Blink timing
BLINK_PERIOD_S = 1.0
CLOSE_TIME_S   = 0.15
OPEN_TIME_S    = BLINK_PERIOD_S - CLOSE_TIME_S

# Move both eyes' lids together
def set_lids(lb, lt, rb, rt):
    left_bottom.angle  = lb
    left_top.angle     = lt
    right_bottom.angle = rb
    right_top.angle    = rt

# Start open
set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
         RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
sleep(0.5)

while True:
    # Close
    set_lids(LEFT_BOTTOM_CLOSED, LEFT_TOP_CLOSED,
             RIGHT_BOTTOM_CLOSED, RIGHT_TOP_CLOSED)
    sleep(CLOSE_TIME_S)

    # Open
    set_lids(LEFT_BOTTOM_OPEN, LEFT_TOP_OPEN,
             RIGHT_BOTTOM_OPEN, RIGHT_TOP_OPEN)
    sleep(OPEN_TIME_S)
