/*
  Client-server test
  Acts as a client for a 7-DOF robotic arm driven by MG996R servos via PCA9685.
  Communicates with a host computer (server) through USB serial.
  
  Commands:
    1. SET a1 a2 a3 a4 a5 a6 a7   -> sets each servo_config to the given angle in degrees (-90 to +90) 
    2. GET                        -> returns the current angles of all servos for fk

  Example:
    SET 0 45 30 15 -10 20 80
    GET

  Author: Jaspreet Chhabra
*/
#include "PCA9685.h"

PCA9685 pwmController;          

PCA9685_ServoEval servoEval;                                                        // Linearly interpolates between standard 2.5%/12.5% phase length (102/512) for -90°/+90°

const int STEP_DEG = 5;                                                             // angle increment per step
const int STEP_DELAY_MS = 25;                                                       // dwell per step
const int BETWEEN_SERVOS_MS = 800;

float currentAngles[7] = {0};  // stores current angles (degrees)

const char* servo_config[7] = {
  "base", "shoulder", "elbow", "wrist_pitch", 
  "wrist_roll", "wrist_yaw", "gripper"};                                            // TODO: confirm order

// helper function to take the channel and move the servo to the required angle
void moveServo(uint8_t ch, int angle) {
  if (angle > 90) angle = 90;                                                       // ensure not out of range
  if (angle < -90) angle = -90;
  
  int count = servoEval.pwmForAngle(angle);
  pwmController.setChannelPWM(ch, count);
  currentAngles[ch] = angle;
}

void setJointAngles(float angles[]) {
  for (uint8_t i = 0; i < 7; i++) {
    moveServo(i, angles[i]);
    delay(20);  // small delay for smooth motion
  }
}

// brings the servo to 0 deg
void centerServo(uint8_t ch) {
  moveServo(ch, 0);                                                                 // internal 0° = mechanical center (~90°)
  delay(300);
}

void sendCurrentAngles() {
  Serial.print("ANGLES ");
  for (uint8_t i = 0; i < 7; i++) {
    Serial.print(currentAngles[i], 1);  // print with one decimal place
    if (i < 7 - 1) Serial.print(" ");
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
    for (uint8_t i = 0; i < 7; i++) {
        centerServo(i);
    }
    delay(1000);
  
    Serial.println("READY"); 
}

void loop() {
  static String inputLine = "";

  while (Serial.available()) {                                                        // read from serial 
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputLine.length() == 0) continue;                                          // skip empty lines

      inputLine.trim();

      // Parse commands
      if (inputLine.startsWith("SET")) {                                              // SET command
        float newAngles[7];
        int count = sscanf(inputLine.c_str(), "SET %f %f %f %f %f %f %f",
                           &newAngles[0], &newAngles[1], &newAngles[2],
                           &newAngles[3], &newAngles[4], &newAngles[5], &newAngles[6]);
        if (count == 7) {
          setJointAngles(newAngles);
          Serial.println("OK");
        } else {
          Serial.println("ERR BAD_FORMAT");
        }
      }

      else if (inputLine.startsWith("GET")) {                                        // GET command
        sendCurrentAngles();
      }

      else {
        Serial.println("ERR UNKNOWN_CMD");
      }

      inputLine = "";                                                                // reset buffer
    } else {
      inputLine += c;
    }
  }
}
  
