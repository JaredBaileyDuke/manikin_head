from time import sleep
from adafruit_servokit import ServoKit

# PCA9685 has 16 channels, so initialize with 16
kit = ServoKit(channels=16)

# Servo on channel x
servo = kit.servo[1]

# Limit the pulse width range to better match servos
servo.set_pulse_width_range(500, 2500)

while True:
    # servo.angle = 60
    # sleep(1)
    # servo.angle = 160
    # sleep(1)
    servo.angle = 125
    sleep(1)

### left eye ###
# eye left and right - 70 to 180 degrees - 125 center
# eye up and down - 90 to 140 degrees - 110 center
# bottom lid close and open - 30 to 130 degrees
# top lid close and open - 125 to 35 degrees

### right eye ###
# eye left and right - 60 to 160 degrees - 110 center
# eye up and down - 110 to 160 degrees - 125 center
# bottom lid close and open - 30 to 130 degrees
# top lid close and open - 135 to 45 degrees