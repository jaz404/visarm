# visarm

Documentation on the motor driver: https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all


servo driver lib: https://github.com/NachtRaveVL/PCA9685-Arduino


Important Note: The servo.read() function does not provide feedback on the actual, current physical position of the servo motor shaft. Standard hobby servos are "open-loop" systems from the Arduino's perspective; the Arduino only sends a command signal and does not receive the actual position back from the servo's internal potentiometer. 

