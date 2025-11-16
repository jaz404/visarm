# visarm

Documentation on the motor driver: https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all

servo driver lib: https://github.com/NachtRaveVL/PCA9685-Arduino

Important Note: The servo.read() function does not provide feedback on the actual, current physical position of the servo motor shaft. Standard hobby servos are "open-loop" systems from the Arduino's perspective; the Arduino only sends a command signal and does not receive the actual position back from the servo's internal potentiometer.

## joint bounds

| Motor | Joint Name        | Motion 1 (°) | Motion 2 (°) |
|-------|-------------------|--------------|--------------|
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

<table>
    <tr>
        <th>Link (i)</th>
        <th>a<sub>i-1</sub></th>
        <th>α<sub>i-1</sub></th>
        <th>d<sub>i</sub></th>
        <th>θ<sub>i</sub></th>
    </tr>
    <tr>
        <td>1</td>
        <td>0</td>
        <td>90</td>
        <td>a<sub>1</sub></td>
        <td>θ<sub>1</sub></td>
    </tr>
    <tr>
        <td>2</td>
        <td>a<sub>2</sub></td>
        <td>0</td>
        <td>0</td>
        <td>θ<sub>2</sub></td>
    </tr>
    <tr>
        <td>3</td>
        <td>a<sub>3</sub></td>
        <td>0</td>
        <td>0</td>
        <td>θ<sub>3</sub></td>
    </tr>
        <tr>
        <td>4</td>
        <td>a<sub>4</sub></td>
        <td>90</td>
        <td>a<sub>5</sub></td>
        <td>θ<sub>4</sub></td>
    </tr>
        <tr>
        <td>5</td>
        <td>a<sub>6</sub></td>
        <td>0</td>
        <td>0</td>
        <td>θ<sub>5</sub></td>
</table>
</div>

Note: For link 4, is the <th>α<sub>i-1</sub></th> correct?

Substituting values we get:

<table>
    <tr>
        <th>Link (i)</th>
        <th>a<sub>i-1</sub> (in cm)</th>
        <th>α<sub>i-1</sub> (in °)</th>
        <th>d<sub>i</sub> (in cm)</th>
        <th>θ<sub>i</sub> (in °)</th>
    </tr>
    <tr>
        <td>1</td>
        <td>0</td>
        <td>90</td>
        <td>2</td>
        <td>θ<sub>1</sub></td>
    </tr>
    <tr>
        <td>2</td>
        <td>10.3</td>
        <td>0</td>
        <td>0</td>
        <td>θ<sub>2</sub></td>
    </tr>
    <tr>
        <td>3</td>
        <td>9.6</td>
        <td>0</td>
        <td>0</td>
        <td>θ<sub>3</sub></td>
    </tr>
        <tr>
        <td>4</td>
        <td>4</td>
        <td>90</td>
        <td>2.5</td>
        <td>θ<sub>4</sub></td>
    </tr>
        <tr>
        <td>5</td>
        <td>5</td>
        <td>0</td>
        <td>0</td>
        <td>θ<sub>5</sub></td>
</table>
</div>