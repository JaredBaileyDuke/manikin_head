// ESP32: Read two joysticks + buttons, stream CSV over USB serial.
// Line format: j1x,j1y,j1sw,j2x,j2y,j2sw\n
// ADC: 0–4095; Buttons: 0=pressed, 1=released

const int J1_X = 34;   // ADC (input-only)
const int J1_Y = 35;   // ADC (input-only)
const int J1_SW = 25;  // digital, has pullup

const int J2_X = 32;   // ADC
const int J2_Y = 33;   // ADC
const int J2_SW = 26;  // digital, has pullup

int smoothRead(int pin, int samples = 4) {
  long s = 0;
  for (int i = 0; i < samples; i++) s += analogRead(pin);
  return (int)(s / samples);
}

void setup() {
  Serial.begin(115200);
  // Optional: set ADC attenuation for wider range (up to ~3.3V)
  // analogSetAttenuation(ADC_11db);

  pinMode(J1_SW, INPUT_PULLUP);
  pinMode(J2_SW, INPUT_PULLUP);

  delay(50);
}

void loop() {
  int j1x = smoothRead(J1_X);
  int j1y = smoothRead(J1_Y);
  int j2x = smoothRead(J2_X);
  int j2y = smoothRead(J2_Y);

  int j1sw = digitalRead(J1_SW) == LOW ? 0 : 1;
  int j2sw = digitalRead(J2_SW) == LOW ? 0 : 1;

  Serial.print(j1x); Serial.print(',');
  Serial.print(j1y); Serial.print(',');
  Serial.print(j1sw); Serial.print(',');
  Serial.print(j2x); Serial.print(',');
  Serial.print(j2y); Serial.print(',');
  Serial.println(j2sw);

  delay(10); // ~100 Hz
}
