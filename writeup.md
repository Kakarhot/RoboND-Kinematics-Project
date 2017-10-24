## Project: Kinematics Pick & Place


[//]: # (Image References)

[image1]: ./misc_images/dh_diagram.jpg
[image2]: ./misc_images/inverse-kinematics.png
[image3]: ./misc_images/figure_theta123.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is a Denavit-Hartenberg (DH) diagram of the arm:

![alt text][image1]

From the above, and using the parameters from the URDF file, the following DH parameter table was derived.

i | alpha(i-1) | a(i-1) | di | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q1
2 | -pi/2 | 0.35 | 0 | q2
3 | 0 | 1.25 | 0 | q3
4 |  -pi/2 | -0.054 | 1.5 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
7(gripper) | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The code to determine the individual transformation matrices about each joint can be seen below. The name of each matrix is derived from the 2 links the joint connects. For example `T1_2` represents the transform matrix about joint 2 which connects links 1 and 2.

```

T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0 ],
			   [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1 ],
			   [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1 ],
			   [                   0,                   0,            0,               1 ]])

T0_1 = T0_1.subs(s)

T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1 ],
			   [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2 ],
			   [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2 ],
			   [                   0,                   0,            0,               1 ]])

T1_2 = T1_2.subs(s)]

T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2 ],
			   [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3 ],
			   [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3 ],
			   [                   0,                   0,            0,               1 ]])

T2_3 = T2_3.subs(s)

T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3 ],
			   [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4 ],
			   [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4 ],
			   [                   0,                   0,            0,               1 ]])

T3_4 = T3_4.subs(s)

T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4 ],
			   [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5 ],
			   [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5 ],
			   [                   0,                   0,            0,               1 ]])

T4_5 = T4_5.subs(s)

T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5 ],
			   [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6 ],
			   [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6 ],
			   [                   0,                   0,            0,               1 ]])

T5_6 = T5_6.subs(s)

T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6 ],
			   [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7 ],
			   [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7 ],
			   [                   0,                   0,            0,               1 ]])

T6_G = T6_G.subs(s)

```

The above matrices can be simplified by substituting the DH parameters determined earlier.

The transform from base_link to gripper_link can be determined by a simple multiplication of the above matrices as follows: (Note: additional rotations are needed to align the gripper_link frame to the world frame)

```
T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
```

However, the homogeneous transform matrix from base_link to gripper_link can also be determined using only the position and orientation of the gripper_link.

```
T0_G = Matrix([[R0_G, Pxyz],
               [   0,    1]])
```

The rotation part of the transform (`R0_G`) is determined using the gripper_link's `roll`, `pitch`, `yaw` parameters as follows:

```
R_x = Matrix([[ 1,                0,          0],
			  [ 0,        cos(roll), -sin(roll)],
			  [ 0,        sin(roll),  cos(roll)]])

R_y = Matrix([[ cos(pitch),        0,  sin(pitch)],
			  [          0,        1,           0],
			  [-sin(pitch),        0,  cos(pitch)]])

R_z = Matrix([[ cos(yaw), -sin(yaw), 0],
			  [ sin(yaw),  cos(yaw), 0],
			  [ 0,                0, 1]])    

R0_G = R_x * R_y * R_z    

```
We therefore now have two equations for `T0_G`, one in terms of each joint angle and the robot parameters, and the other in terms of only the position and orientation of the end effector. This proves to be extremely useful when performing inverse kinematics as will be seen later.


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The fact that the Kuka KR210 has a spherical wrist formed by joints 4, 5 and 6, the Inverse Kinematics problem can be split into Inverse Position and Inverse Orientation Kinematics. The Inverse Position Kinematics problem is the process of determining the first three joint angles, `theta1`, `theta2` and `theta3` to place joint 5 (the wrist centre) at the correct position. The Inverse Orientation problem is then to orient the wrist correctly (by calculating `theta4`, `theta5` and `theta6`) such that the end effector is pointing the right way.

We will start with the Inverse Position Kinematics.

The first step is to notice that the position of the end effector is not the same as the position of the wrist centre. The wrist centre is at joint5. Therefore, the position of the wrist centre can be calculated by subtracting the distance from the gripper_link to joint 5. However, this subtraction needs to take into account the orientation of the wrist. Luckily we calculated the homogeneous transform matrix from base_link to gripper_link previously. Therefore, the position of the wrist centre is calculated as follows:
```
WC = Matrix([px, py, pz]) - d7 * R0_G * Matrix([1,0,0])
```

Here, `px`, `py`, `pz` are the end effector positions, `d7` is the length from joint5 to the gripper_link and `R0_G` is the rotation part of the transform.

Now that we have the position of the wrist centre, we can proceed to calculate the first three joint angles, `theta1`, `theta2` and `theta3`. The image below shows a simplified drawing of the current problem.

![alt text][image2]
![alt text][image3]

From the above image we can see that `theta1` can be calculated quite easily as follows:
```
theta1 = atan2(WC[1],WC[0])
```

With the above image as reference, we calculate `theta2` and `theta3` by constructing a triangle from joint2 to joint3 to joint5 (wrist centre).


Using trigonometry, we calculate `s1`, `s2`, `s3`, `s4`, `beta2`, `beta3`:

```
s1 = sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35
s2 = WC[2] - 0.75

s3 = sqrt(pow(s1, 2) + pow(s2, 2))
s4 = 1.501

beta2 = acos((1.25*1.25 + s3 * s3 - s4 * s4) / (2 * 1.25 * s3))
beta3 = acos((1.25*1.25 + s4 * s4 - s3 * s3) / (2 * 1.25 * s4))
```
Then, `theta2` and `theta3` can be calculated:

```
theta2 = pi / 2 - beta2 - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
theta3 = pi / 2 - (beta3 + 0.054/1.5)
```

For the Inverse Orientation problem, we need to find values of the final three joint variables.

R3_6 = inv(R0_3) * Rrpy

The resultant matrix on the RHS (Right Hand Side of the equation) does not have any variables after substituting the joint angle values, and hence comparing LHS (Left Hand Side of the equation) with RHS will result in equations for joint 4, 5, and 6.

```
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


The Forward Kinematics steps start with defining the DH parameter and joint angle symbols, as well as the DH parameter table.The individual transformation matrices are then defined in terms of these symbolic parameters and DH parameters. The Forward Kinematics steps were moved outside of the for loop in the handle_calculate_IK function. This was done so that the loop could execute faster. Since the Forward Kinematics matrices are declared symbolically, there is no need to re-calculate each loop. The matrices can just be evaluated with different parameters within the loop. All the Inverse Kinematics steps are executed within the for loop, which starts on line 89.

##### Shortfalls and Improvements

 The Kuka arm in the simulator still runs a lot slower than I would like. This is likely due to a number of external factors such as the virtual machine and the simulator. I allocated 4 cores, 8GB memory and 768MB graphics memory to the simulator and it still ran pretty slowly. The speed of the Kinematics calculations could still be increased in a number of ways such as moving away from sympy entirely and just using numpy for the matrix calculations. The effect this has on accuracy would need to be investigated however.

