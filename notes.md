# Kinematics of 5DOF Arm

## Introduction

Our arm has 6 revolute joints, with no prismatic joints (i.e., the robot can only rotate at places). The wrist rotates as well. It has 3 joints parallel to each other.

### Types of joints

![Revolute vs Prismatic Joint](https://external-content.duckduckgo.com/iu/?u=https%3A%2F%2Fimages.wevolver.com%2FeyJidWNrZXQiOiJ3ZXZvbHZlci1wcm9qZWN0LWltYWdlcyIsImtleSI6ImZyb2FsYS8xNjk5MTkzOTIzOTA2LWltYWdlOS5qcGciLCJlZGl0cyI6eyJyZXNpemUiOnsid2lkdGgiOjk1MCwiZml0IjoiY292ZXIifX19&f=1&nofb=1&ipt=45091a64d6e4399ba966919954914d51354b4b0db4e683d2e2b96881c6671244 "Revolute vs Prismatic Joint")

## D-H (Denavit Hartenburg Parameters)

This is a convention used to understand and define the parameters of a robot.

#### Link and Joint Paramters:

- Link Lenght (a<sub>i-1</sub>): It is the mutual perpendictular distance between 2 axis. This is the distance from z<sub>i</sub> to z<sub>i+1</sub>
- Link Twist (α<sub>i-1</sub>): Angle from axis i-1 to i. This is the angle from z<sub>i</sub> to z<sub>i+1</sub> measured about the x-axis.
- Link Offset (d<sub>i</sub>): The offset distance from one link to another. This is the distance from x<sub>i-1</sub> to x<sub>i</sub> along the z-axis
- Joint Angle (θ<sub>i</sub>): Rotation of link with respect to its neighbor (along the common axis). This is the angle from x<sub>i-1</sub> to x<sub>i</sub> measured about the z-axis.

<div align="center">
    <img src="https://www.researchgate.net/publication/357668555/figure/fig1/AS:1109779019771915@1641603345650/Representation-of-the-Denavit-Hartenberg-parameters.png" alt="Description">
</div>

#### Example

<div align="center">
    <img src="images/1.png" alt="Description">

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
        <td>0</td>
        <td>d<sub>1</sub></td>
        <td>0</td>
    </tr>
    <tr>
        <td>2</td>
        <td>L<sub>1</sub></td>
        <td>0</td>
        <td>0</td>
        <td>θ<sub>2</sub></td>
    </tr>
    <tr>
        <td>3</td>
        <td>L<sub>2</sub></td>
        <td>0</td>
        <td>0</td>
        <td>θ<sub>3</sub></td>
    </tr>
</table>
</div>

## Overview of the arm

<img src="images/diagram.jpeg" alt="Description"/>

Note:

- θ is the angle from xn-1 to xn around zn-1.
- α is the angle from zn-1 to zn around xn.
- a is the distance between the origin of the n-1 frame and the origin of the n frame along the xn direction.
- d is the distance from xn-1 to xn along the zn-1 direction.

The arm has to following D-H parameters:

| Link (i) | θ<sub>i</sub> (°)     | α<sub>i-1 (°) | d<sub>i</sub> (in cm)         | a<sub>i-1</sub> (in cm) |
| -------- | --------------------- | ------------- | ----------------------------- | ----------------------- |
| 1        | θ<sub>1</sub>         | 90            | a<sub>1</sub>                 | 0                       |
| 2        | θ<sub>2</sub> + pi/2  | 0             | 0                             | a<sub>2</sub>           |
| 3        | -θ<sub>3</sub>        | 0             | 0                             | a<sub>3</sub>           |
| 4        | -θ<sub>4</sub> + pi/2 | 90            | 0                             | a<sub>5</sub>           |
| 5        | θ<sub>5</sub>         | 0             | a<sub>4</sub> + a<sub>6</sub> | 0                       |

(For joints 2, 3, and 4 if you want to change the direction in the global x axis, negate theta.)

Substituting values we get:

| Link (i) | θ<sub>i</sub> (°)     | α<sub>i-1 (°) | d<sub>i</sub> (in cm) | a<sub>i-1</sub> (in cm) |
| -------- | --------------------- | ------------- | --------------------- | ----------------------- |
| 1        | θ<sub>1</sub>         | 90            | 2                     | 0                       |
| 2        | θ<sub>2</sub> + pi/2  | 0             | 0                     | 10.3                    |
| 3        | -θ<sub>3</sub>        | 0             | 0                     | 9.6                     |
| 4        | -θ<sub>4</sub> + pi/2 | 90            | 0                     | 2.5                     |
| 5        | θ<sub>5</sub>         | 0             | 9                     | 0                       |

## Forward kinematics:

This is easy to calculate. Give the angles, the translations and the link lenghts and it will output the position of the end-effector. It works by applying transformation matrices at each joint and hence we get the end-effector position in the final.

## Inverse Kinematics:

### When does inverse kinematics have a closed form solution

The inverse kinematics has a closed form solution in the following two conditions:

- 3 of its joints are parallel to each other
- 3 of its joints has a rotation axis that intersect at a common point.

### Overview

Given the position and orientaion to point to, find the joint angles so that the end-effector moves and orients to the desired point.

### Calculation

Since the robot has 5 revolute joints, we need to find 5 angles to move the robot. This involves transformations from the end-effector to the base of the robot.

#### Velocities

The Jacobian is used to transform the joint velocities into end-effector velocities as follows:

$$
\begin{bmatrix}
ẋ \cr
ẏ \cr
ż \cr
ω_x\cr
ω_y\cr
ω_z\cr
\end{bmatrix}_{6x1} =
J_{6xn}\begin{bmatrix}
q̇_1  \cr
q̇_2 \cr
q̇_3 \cr
... \cr
q̇_n\cr
\end{bmatrix}_{nx1}
$$

Therefore, in our case it would be:

$$
\begin{bmatrix}
ẋ \cr
ẏ \cr
ż \cr
ω_x\cr
ω_y\cr
ω_z\cr
\end{bmatrix}_{6x1} =
J_{6x5}\begin{bmatrix}
θ_1  \cr
θ_2 \cr
θ_3 \cr
θ_4 \cr
θ_5\cr
\end{bmatrix}_{5x1}
$$

#### Jacobian

The jacobian would be a mxn matrix where m represents the dimensions (2D or 3D) and n represents the number of joints. In our case it would be 6x5 matrix as we are in 3D (so 3 for position and 3 for orientation) and 5 joints that determine the arms position / orientation.

The jacobian can be divided into two parts, the first three for linear velocities and the bottom three for angular velocities:

$$
\begin{bmatrix}
J_v  \cr
J_ω \cr
\end{bmatrix}_{6x5}
$$

#### Analytical Method

We will try to extract a mathematical formula for the arm so that it is easy to compute. Our robot satifies the Pieper's law as the 2nd, 3rd and 4th joints are parallel and so we can find an analytical solution.

- Assumption 1: The wrist position (Joint 4) depends on the first three joints. (0 to 3)
- Assumption 2: The wrist orientation depends on the last two (that is joint 4 and joint 5). (The homogenous transformation from 5 to 3).

## References

- https://www.youtube.com/watch?v=wDus2EKLg3s
- https://automaticaddison.com/the-ultimate-guide-to-jacobian-matrices-for-robotics/
- https://automaticaddison.com/homogeneous-transformation-matrices-using-denavit-hartenberg/#Example_3_%E2%80%93_Six_Degree_of_Freedom_Robotic_Arm
- https://automaticaddison.com/the-ultimate-guide-to-inverse-kinematics-for-6dof-robot-arms/#Analytical_Approach_vs_Numerical_Approach_to_Inverse_Kinematics

## Things to look into

- https://moveit.github.io/moveit_tutorials/doc/hand_eye_calibration/hand_eye_calibration_tutorial.html
- https://openrave.org/docs/0.8.0/openravepy/ikfast/
