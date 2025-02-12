# High-Level Controller

The high-level controller is quite simple as most complexity is handled through the low-level controller.

The current implementation is to track the velocity error based on the pose difference and use a PD control to scale this error and publish it to the low-level controller.

# Low-Level Controller

The low-level controller is really just a node that houses three separate controllers whose outputs are superimposed to get individual motor forces.

## Z control

Vertical control is the most simple. Using the roll and pitch angles, the required vertical force is:

 $F^N = (R_B^N)^{-1}(\frac{mv}{\Delta t} + mg)$

where:

v is the error between the desired vertical velocity and the actual vertical velocity

$R_B^N$ is the rotation matrix from the body to the inertial frame based on the roll,pitch,yaw angles and a +x forward, +y left, +z up coordinate frame.

$\Delta t$ is the desired convergence time. This should be sufficiently small to ensure quick controller adjustments but large enough to reduce total motor force. It is currently around 0.5s.

By understanding the vertical velocity that the drone needs to maintain hover or reach a certain altitude, this gives a baseline for how to control the pitch and roll.


## Pitch Control

Pitch control is a bit more nuanced. Because there are no opposing forces in the x direction (assuming drag is small which, from testing, appears to be the case at the speeds intended for this drone), acceleration and deceleration must be entirely handled by the pitch rate.

What can be leveraged is that the vertical force must maintain constant or else the z error will grow. Using first principles, an equation for the desired pitch angle can be achieved to reach a horizontal velocity in one time-step. The derivation can be found here:

-> $Fx^N = F_z^Nsin(\theta)$

-> the value for $F_z^N$ is known as it was calculated for the z controller

-> $\frac{mv_x}{\Delta t} = F_z^Nsin(\theta)$

-> $\theta = arcsin(\frac{mv_x}{F_z^N\Delta t})$

Of course, the domain of arcsin is $-1 \leq x \leq 1$ and so the desired roll angle is limited by the update rate of the function. Too fast of an update rate or too large of a velocity error will lead to a nan value. To handle this, I have a maximum pitch value that is utilized when the pitch angle is too large (indicated by a nan arcsin value).

Because angular velocity is simply the change in angle over time, the desired pitch can be related to a desired pitch rate by:

-> $\omega_y = \frac{\theta_{req}}{\Delta t}$

By understanding the desired angular velocity, the desired torque can be calculated to reach that pitch angle in one timestep. For a quadcopter, the drone will pitch forward if the back two motors output higher force than the front two and vice versa. the required torque can be calculated as:

$\tau_{req} = I_{yy} * \frac{\omega_{y,err}}{\Delta t}$

PD control is implemented by scaling the force inputs to the motors.

## Roll Control

Roll control is very similar to pitch control and has the same structure while using angular velocity around the x axis.

## Yaw Control

Because the current goal is to simply reach a point on the map and not to get there with the correct orientation, no yaw control is implemented. When sensors are implemented, yaw control will be essential especially in the case of cameras.

## Controller Superposition

At the end of the day, all that the force output is really doing is vector addition. By calculating the z force required to maintain hover, all that needs to be changed is the torque. Thus, the forces can be superimposed as long as the overall thrust is maintained.

For each controller, I:

1. Calculate the control law and add it to the intended motors (i.e. if controlling vx in the positive direction, add control law to the back motors)
2. Subtract the control law output from the motors that are not in the direction of control. This ensures that the previous control law is maintained as there is a net-zero addition in overall thrust.
