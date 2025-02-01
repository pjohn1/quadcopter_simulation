# quadcopter_simulation
Physics &amp; controls simulation of a quadcopter built from scratch using ROS2, RVIZ, and some 3d modeling from an iPhone camera

# Motivation

The goal of this project is to create a sandbox environment for me to practice different topics in control theory. Additionally, I challenged myself to code all of the ROS nodes in C++ to improve my proficiency in the language.

# Control Structure

During my 16.30 (Feedback Control Systems) class at MIT, the final project was to control a DJI Tello drone. Our control variables were the velocity vectors and yaw rate. I wanted to replicate this in this project. Thus, the high-level control is centered around velocity vector input which is fed into a low-level controller that translates the velocity vectors into individual motor forces to control roll, pitch, and vertical acceleration. I found this approach to be much more intuitive as the velocity control can then just be based on the pose difference between the drone and the goal pose. In flowchart form, this control looks like:

![flowchart](https://github.com/user-attachments/assets/72e4a0de-c771-4d85-a82d-be261a7b5fcf)

# Low-Level Controller

The low-level controller is really just a node that houses three separate controllers whose outputs are added together to get individual motor forces.

## Z control

Vertical control is the most simple. Using the roll and pitch angles, the required vertical force is:

![equation]([[https://latex.codecogs.com/svg.image?&space;F^N=(R_B^N)^{-1}(\frac{mv}{\Delta&space;t}&plus;mg)](https://latex.codecogs.com/svg.image?%20F%5EN=(R_B%5EN)%5E%7B-1%7D(%5Cfrac%7Bmv%7D%7B%5CDelta%20t%7D&plus;mg))](https://latex.codecogs.com/png.image?%5Cdpi%7B110%7D%20F%5EN=(R_B%5EN)%5E%7B-1%7D(%5Cfrac%7Bmv%7D%7B%5CDelta%20t%7D&plus;mg)))

where v is the error between the desired vertical velocity and the actual vertical velocity;
![equation](https://latex.codecogs.com/png.image?%5Cdpi%7B110%7D%5CDelta%20t) is the update rate. This ensures that the drone reaches the intended vertical velocity by the next frame so that the high-level controller can function accordingly.

By understanding the vertical velocity that the drone needs to maintain hover or reach a certain altitude, this gives a baseline for how to control the pitch and roll.
