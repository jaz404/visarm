#include "PCA9685.h"

PCA9685 pwmController(Wire);
PCA9685_ServoEval servoEval;

const int STEP_DEG = 5;                 // angle increment per step
const int STEP_DELAY_MS = 300;          // dwell per step
const char* servo_config[7] = {
  "base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "wrist_yaw", "gripper"
};

// Helper: move servo to requested angle

void moveServo(uint8_t ch, int targetAngle, int speed = 5) {
  static int currentAngle[16] = {0};

  targetAngle = constrain(targetAngle, -90, 90);
  speed = constrain(speed, 1, 100);  

  int start = currentAngle[ch];
  int step = (targetAngle > start) ? 1 : -1;  // move 1° per iteration

  // compute delay inversely proportional to speed
  int delayMs = map(speed, 1, 100, 50, 5);    // higher speed → shorter delay

  for (int a = start; (step > 0) ? a <= targetAngle : a >= targetAngle; a += step) {
    pwmController.setChannelPWM(ch, servoEval.pwmForAngle(a));
    delay(delayMs);
  }

  pwmController.setChannelPWM(ch, servoEval.pwmForAngle(targetAngle));
  currentAngle[ch] = targetAngle;
}

void CenterAllServos(){
  for (int i = 0; i < 7;i++){
      moveServo(i,0,5);
      delay(5);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pwmController.resetDevices();
  pwmController.init();
  pwmController.setPWMFreqServo();

  CenterAllServos();
// CENTERING ALL JOINTS

  // motor 1
  // moveServo(0,0,5);
  // delay(5);

  // // motor 2
  // moveServo(1,0,5);
  //     delay(5);

  // // motor 3
  // moveServo(2,0,5);
  // delay(1000);

  // // motor 4
  // moveServo(3,0,5);
  // delay(5);

  // // motor 5
  // moveServo(4,0,5);
  // delay(5);

  // motor 6
  // open claw
//  moveServo(5,65,3);
//  delay(1000);
  // close claw   
//  moveServo(5,90,3);
//  delay(2000);  

// CENTERING DONE

// FULL SWEEP

  // m1
//  moveServo(0,90,5);
//  moveServo(0,-80,5);
//  moveServo(0,0,5);
//  delay(500); 
//
//  // m2
//  moveServo(1,30,5);
//  moveServo(1,-80,5);
//  moveServo(1,0,5);
//  delay(500); 
//
//  // m3
//  moveServo(2,90,5);
//  moveServo(2,-90,5);
//  moveServo(2,0,5);
//  delay(500); 
//
//  // m4
//  moveServo(3,90,5);
//  moveServo(3,-90,5);
//  moveServo(3,0,5);
//  delay(500); 
//
//
//  // m5
//  moveServo(4,90,5);
//  moveServo(4,-90,5);
//  moveServo(4,0,5);
//  delay(500); 
    
}


void loop() {
}
