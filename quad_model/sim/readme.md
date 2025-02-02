# Overview

In physics_node.cpp, the drone forcing functions are used to calculate the velocities and angular velocities at each time step. This is then published to physics_sim.cpp to update the pose and publish a transform to RVIZ to update the visual.

# Forcing Functions

The drone is modeled as a quadcopter with four motors where the top-left and bottom-right turn CCW while the top-right and bottom-left turn CW. This leads to the following forcing functions:

(note: these forcing functions are all expressed in the body frame)

$F_z = \sum_{i=0}^3{f_i}$

$\tau_x = d * ( f_1 + f_2 - (f_0 + f_3) )$

$\tau_y = d * ( f_2 + f_3 - (f_0 + f_1) )$

$\tau_z = f_0 + f_2 - (f+1 + f_3)$

where:

$f_i$ is the motor force; $f_0$ is top-left, $f_1$ top-right, $f_2$ bottom-right, $f_3$ bottom-left

d is the distance from the drone's center of mass to the motors. the CM is assumed to be at the geometric center.

# Attitude Calculation

The attitude calculation is done utlizing the Rodrigues' rotation formula which is outlined in https://www.roboticsbook.org/S72_drone_actions.html

This results in an SO(3) matrix and thus the columns correspond to the unit vectors of the x,y,z body frame axis.

From here, the roll, pitch, and yaw angles are extracted

# Inertial Frame Forces

From the attitude matrix, the forces in the inertial frame can be extracted by creating a rotation matrix to multiply the forces in the body frame by.

Then, the velocities are calculated from the inertial frame as:

$v_i = \frac{F_i}{m} * \Delta t$

These velocities, along with the angular velocities, are published to the sim node

# Simulation

The simulation is performed by publishing a ROS transform between the base_link (drone) frame and the map (inertial) frame.
