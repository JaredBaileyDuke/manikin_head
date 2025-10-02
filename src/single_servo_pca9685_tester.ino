#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150  // Min pulse length out of 4096
#define SERVOMAX  600  // Max pulse length out of 4096

uint8_t servonum = 0;  // Servo slot 0

// Convert angle (0–180°) to PCA9685 pulse
int angleToPulse(int ang) {
  int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz
  delay(10);
}

void loop() {
  // // Sweep from 0° to 180° and back, 3 times
  // for (int i = 0; i < 3; i++) {
  //   // 0 → 180
  //   for (int ang = 0; ang <= 180; ang++) {
  //     pwm.setPWM(servonum, 0, angleToPulse(ang));
  //     delay(15);
  //   }

  //   // 180 → 0
  //   for (int ang = 180; ang >= 0; ang--) {
  //     pwm.setPWM(servonum, 0, angleToPulse(ang));
  //     delay(15);
  //   }
  // }

  // After sweeps, hold at 90°
  pwm.setPWM(servonum, 0, angleToPulse(90));

  // Stop looping forever
  while (true) {
    delay(1000); // idle, keep servo at 90°
  }
}
