from time import sleep
from adafruit_servokit import ServoKit

# PCA9685 has 16 channels, so initialize with 16
kit = ServoKit(channels=16)

# Servo on channel 000
servo = kit.servo[0]

# Optional: limit the pulse width range to better match your servo
# (many 9g servos like SG90s respond best to ~500–2500 µs)
servo.set_pulse_width_range(500, 2500)

while True:
    servo.angle = 80
    sleep(1)
    servo.angle = 100
    sleep(1)
