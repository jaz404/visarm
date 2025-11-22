# visarm

Documentation on the motor driver: https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all

servo driver lib: https://github.com/NachtRaveVL/PCA9685-Arduino

Important Note: The servo.read() function does not provide feedback on the actual, current physical position of the servo motor shaft. Standard hobby servos are "open-loop" systems from the Arduino's perspective; the Arduino only sends a command signal and does not receive the actual position back from the servo's internal potentiometer.

## joint bounds

| Motor | Joint Name        | Motion 1 (°) | Motion 2 (°) |
| ----- | ----------------- | ------------ | ------------ |
| 1     | Base (ch 0)       | +90          | -80          |
| 2     | Shoulder (ch 1)   | +30          | -80          |
| 3     | Elbow (ch 2)      | +90          | -90          |
| 4     | Wrist Roll (ch 3) | +90          | -90          |
| 5     | Wrist Yaw (ch 4)  | +90          | -90          |
| 6     | Gripper (ch 5)    | +65 (open)   | +90 (close)  |

## Inverse Kinematics:

#### When does inverse kinematics have a closed form solution

The inverse kinematics has a closed form solution in the following two conditions:

- 3 of its joints are parallel to each other
- 3 of its joints has a rotation axis that intersect at a common point.

#### Overview

Given the position to point to, find the rotation and translation points for each joint so that the end-effector moves to the point.

#### Calculation

Since the robot has 6 revolute joints, we need to find 6 angles to move the robot. This involves transformations from the end-effector to the base of the robot.

- There will be 12 equations (3 position, 9 orientations)

## Overview of the arm

<div align="center">
    <img src="images/diagram.jpeg" alt="Description">
</div>

The arm has to following D-H parameters:

| Link (i) | θ<sub>i</sub> (°)  | α<sub>i-1 (°) | d<sub>i</sub> (in cm)         | a<sub>i-1</sub> (in cm) |
| -------- | ------------------ | ------------- | ----------------------------- | ----------------------- |
| 1        | θ<sub>1</sub>      | 90            | a<sub>1</sub>                 | 0                       |
| 2        | θ<sub>2</sub>      | 0             | 0                             | a<sub>2</sub>           |
| 3        | θ<sub>3</sub>      | 0             | 0                             | a<sub>3</sub>           |
| 4        | θ<sub>4</sub> + 90 | 90            | 0                             | a<sub>5</sub>           |
| 5        | θ<sub>5</sub>      | 0             | a<sub>4</sub> + a<sub>6</sub> | 0                       |

Substituting values we get:

| Link (i) | θ<sub>i</sub> (°)  | α<sub>i-1 (°) | d<sub>i</sub> (in cm) | a<sub>i-1</sub> (in cm) |
| -------- | ------------------ | ------------- | --------------------- | ----------------------- |
| 1        | θ<sub>1</sub>      | 90            | 2                     | 0                       |
| 2        | θ<sub>2</sub>      | 0             | 0                     | 10.3                    |
| 3        | θ<sub>3</sub>      | 0             | 0                     | 9.6                     |
| 4        | θ<sub>4</sub> + 90 | 90            | 0                     | 2.5                     |
| 5        | θ<sub>5</sub>      | 0             | 9                     | 0                       |

## Forward kinematics:

./tutorial-chessboard-pose --square_size 0.0260 --input "/home/krupal/Documents/class files/CMPUT 312/final_project/visarm/server/output/image_%d.png" --intrinsic "/home/krupal/Documents/class files/CMPUT 312/final_project/visarm/server/output/custom_camera.xml" --output "/home/krupal/Documents/class files/CMPUT 312/final_project/visarm/server/output/pose_cPo_%d.yaml"
./tutorial-hand-eye-calibration --data-path "/home/krupal/Documents/class files/CMPUT 312/final_project/visarm/server/output" --fPe pose_rPe_%d.yaml --cPo pose_cPo_%d.yaml --output "/home/krupal/Documents/class files/CMPUT 312/final_project/visarm/server/output/visarm_final_cal_output.yaml"
