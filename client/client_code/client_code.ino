/*
  Client-server test
  Acts as a client for a 6-DOF robotic arm driven by MG996R servos via PCA9685.
  Communicates with a host computer (server) through USB serial.
  
  Commands:
    1. SET a1 a2 a3 a4 a5 a6   -> sets each servo_config to the given angle in degrees (-90 to +90) 
    2. GET                        -> returns the current angles of all servos for fk

  Example:
    SET 0 0 0 0 45 0
    GET
*/
#include "PCA9685.h"

PCA9685 pwmController;          

PCA9685_ServoEval servoEval;                                                        // Linearly interpolates between standard 2.5%/12.5% phase length (102/512) for -90°/+90°

const int STEP_DEG = 5;                                                             // angle increment per step
const int STEP_DELAY_MS = 25;                                                       // dwell per step
const int BETWEEN_SERVOS_MS = 800;

float currentAngles[6] = {0};  // stores current angles (degrees)

const char* servo_config[6] = {
  "base", "shoulder", "elbow", "wrist_pitch", 
  "wrist_roll", "gripper"};                                                         // TODO: confirm order


// helper function to take the channel and move the servo to the required angle
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
  currentAngles[ch] = targetAngle;   
}
void CenterAllServos(){
  for (int i = 0; i < 6; i++) {
      moveServo(i, 0, 5);
  }

}
void setJointAngles(float angles[]) {
for (uint8_t i = 0; i < 6; i++) {
    moveServo(i, angles[i]);
}

}

void sendCurrentAngles() {
  Serial.print("ANGLES ");
for (uint8_t i = 0; i < 6; i++) {
    Serial.print(currentAngles[i], 1);
    if (i < 5) Serial.print(" ");
}
Serial.println();

}

void setup() {
    Serial.begin(115200);               
    Wire.begin();

    pwmController.resetDevices();      
    pwmController.init();               
    pwmController.setPWMFreqServo();    
    
    // center the servos
    CenterAllServos();
    delay(1000);
  
    Serial.println("READY"); 
}

void loop() {
  static String inputLine = "";

  while (Serial.available()) {
    char c = Serial.read();

    // Build input line until newline
    if (c != '\n' && c != '\r') {
      inputLine += c;
      continue;
    }

    // Ignore empty lines
    if (inputLine.length() == 0) {
      inputLine = "";
      continue;
    }

    inputLine.trim();

    Serial.print("RAW: '");
    Serial.print(inputLine);
    Serial.println("'");

    if (inputLine.startsWith("SET")) {
      int ai[6];
      float newAngles[6];

      int count = sscanf(inputLine.c_str(),
                         "SET %d %d %d %d %d %d",
                         &ai[0], &ai[1], &ai[2],
                         &ai[3], &ai[4], &ai[5]);

      Serial.print("COUNT="); Serial.println(count);

      if (count == 6) {
        for (int i = 0; i < 6; i++) newAngles[i] = (float)ai[i];
        setJointAngles(newAngles);
        Serial.println("OK");
      } else {
        Serial.println("ERR BAD_FORMAT");
      }
    }

    else if (inputLine.startsWith("GET")) {
      sendCurrentAngles();
    }

    else {
      Serial.println("ERR UNKNOWN_CMD");
    }

    // Reset input buffer
    inputLine = "";
  }
}
