#include <Wire.h>
#include "PCA9685.h"

PCA9685 pwm;
int servoChannel = 0;   // Change if your servo is connected to a different channel
int pwmValue = 307;     // Start at mid position (roughly 1.5ms pulse)

void setup() {
  Serial.begin(115200);
  Serial.println("=== MG996R Servo Calibration Tool ===");
  Serial.println("Type a PWM count (e.g. 100–600) and press ENTER.");
  Serial.println("Observe servo position. Find min, mid, and max where it moves safely.");
  Serial.println("---------------------------------------------------");

  Wire.begin();
  pwm.resetDevices();
  pwm.init();
  pwm.setPWMFrequency(50);   // 50Hz for servos
  delay(500);

  pwm.setChannelPWM(servoChannel, pwmValue);  // Move to mid
  Serial.print("Starting at count = ");
  Serial.println(pwmValue);
}

void loop() {
  // Check if user entered a new value in Serial Monitor
  if (Serial.available()) {
    pwmValue = Serial.parseInt();   // read the integer user typed

    if (pwmValue > 0 && pwmValue < 4096) {
      pwm.setChannelPWM(servoChannel, pwmValue);
      Serial.print("Set PWM count to: ");
      Serial.println(pwmValue);
    } else {
      Serial.println("Invalid value. Enter a number between 1 and 4095.");
    }
  }
}
