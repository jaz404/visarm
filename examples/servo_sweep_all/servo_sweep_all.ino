/*
  7-Servo Arm Sweep Test
  This program sequentially sweeps all joints of a 6-DOF robotic arm and the gripper
  using MG996R servos driven by a PCA9685 controller. Each servo moves smoothly from 
  -90° to +90° and back, allowing range-of-motion and wiring verification.
  Author: Jaspreet Chhabra
*/
#include "PCA9685.h"

PCA9685 pwmController(Wire);          

// 0.5ms ≈ 102, 1.5ms ≈ 307, 2.5ms ≈ 512  @ 50Hz (20 ms period)
//PCA9685_ServoEval servoEval(102, 307, 512);

PCA9685_ServoEval servoEval;                                                        // Linearly interpolates between standard 2.5%/12.5% phase length (102/512) for -90°/+90°

const int STEP_DEG = 5;                                                             // angle increment per step
const int STEP_DELAY_MS = 25;                                                       // dwell per step
const int BETWEEN_SERVOS_MS = 800;

const char* servo_config[7] = {
  "base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "wrist_yaw", "gripper"}; // TODO: confirm order

// helper function to take the channel and move the servo to the required angle
void moveServo(uint8_t ch, int angle) {
  int count = servoEval.pwmForAngle(angle);
  pwmController.setChannelPWM(ch, count);
}
// brings the servo to 0 deg
void centerServo(uint8_t ch) {
  moveServo(ch, 0);                                                                 // internal 0° = mechanical center (~90°)
  delay(300);
}

void sweepServo(uint8_t ch) {
                                                                                    // Sweep from -90° to +90°
  for (int a = -90; a <= 90; a += STEP_DEG) {
    moveServo(ch, a);
    delay(STEP_DELAY_MS);
  }
  // Sweep back to -90°
  for (int a = 90; a >= -90; a -= STEP_DEG) {
    moveServo(ch, a);
    delay(STEP_DELAY_MS);
  }
}

void setup() {
    Serial.begin(115200);               
    Wire.begin();

    pwmController.resetDevices();      

    pwmController.init();               

    pwmController.setPWMFreqServo();    
    // center the servos
    for (uint8_t i = 0; i < 7; i++) {
        centerServo(i);
    }
  delay(1000);
}

void loop() {
  for (uint8_t i = 0; i < 7; i++) {
    uint8_t ch = i;
    Serial.print("Sweeping "); Serial.print(servo_config[i]);
    Serial.print("  (channel "); Serial.print(ch); Serial.println(")");

    sweepServo(ch);
    delay(500);
    centerServo(ch);

    delay(BETWEEN_SERVOS_MS);
  }
  Serial.println("Cycle complete.\n");
  delay(2000);
}
  
