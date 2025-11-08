# visarm

Documentation on the motor driver: https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all

servo driver lib: https://github.com/NachtRaveVL/PCA9685-Arduino

Important Note: The servo.read() function does not provide feedback on the actual, current physical position of the servo motor shaft. Standard hobby servos are "open-loop" systems from the Arduino's perspective; the Arduino only sends a command signal and does not receive the actual position back from the servo's internal potentiometer.

# Base offset: 9.5cm

# Offset between Base and Joint 2 = 2 cm

# Link1: Joint 2 to Joint 3 = 10.3cm

# Link 2: Joint 3 to Joint 4 = 9.6

# Offset between Joint 4 and Joint 5 = 4cm (2.5)

# Offset between Joint 5 and End Effector = 5.0cm

# End Effector = 11cm

# Check position of camera (above the end effector) to be placed
