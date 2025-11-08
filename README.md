# visarm

Documentation on the motor driver: https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all


servo driver lib: https://github.com/NachtRaveVL/PCA9685-Arduino


Important Note: The servo.read() function does not provide feedback on the actual, current physical position of the servo motor shaft. Standard hobby servos are "open-loop" systems from the Arduino's perspective; the Arduino only sends a command signal and does not receive the actual position back from the servo's internal potentiometer. 


## joint bounds

| Motor | Joint Name  | Motion 1 (°) | Motion 2 (°) | Speed | Delay After (ms) | Notes |
|--------|--------------|--------------|--------------|--------|------------------|-------|
| 1 | Base (ch 0) | -90 | +75 | 5 | 1000 | Base rotation sweep |
| 2 | Shoulder (ch 1) | -90 | +90 | 5 | 1000 | Shoulder up/down |
| 3 | Elbow (ch 2) | -90 | +80 | 5 | 1000 | Elbow bend |
| 4 | Wrist Roll (ch 3) | -90 | +90 | 5 | 1000 | Wrist forward/backward |
| 5 | Wrist Yaw (ch 4) | -90 | +90 | 5 | 1000 | Wrist rotation |
| 6 | Gripper (ch 5) | +65 (open) | +90 (close) | 3 | 2000 | Claw open/close |

