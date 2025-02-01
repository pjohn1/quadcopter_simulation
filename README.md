# quadcopter_simulation
Physics &amp; controls simulation of a quadcopter built from scratch using ROS2, RVIZ, and some 3d modeling from an iPhone camera

# Motivation

The goal of this project is to create a sandbox environment for me to practice different topics in control theory. Additionally, I challenged myself to code all of the ROS nodes in C++ to improve my proficiency in the language.

# Use of Generative AI

There is no doubt that generative AI has become an incredible tool to aid in code generation. However, the goal of this project was for me to hone my coding skills. As such, generative AI was $\textbb{not}$ used for generating C++ code outside of the structure of the physics_sim.cpp file.

Generative AI aided in the implementation of:

- URDF files to model the drone and map
- Launch file creation
- Traceback debugging and code logic verification (i.e. "does this rotation matrix make sense for my coordinate frame?")

# Control Structure

During my 16.30 (Feedback Control Systems) class at MIT, the final project was to control a DJI Tello drone. Our control variables were the velocity vectors and yaw rate. I wanted to replicate this in this project. Thus, the high-level control is centered around velocity vector input which is fed into a low-level controller that translates the velocity vectors into individual motor forces to control roll, pitch, and vertical acceleration. I found this approach to be much more intuitive as the velocity control can then just be based on the pose difference between the drone and the goal pose. In flowchart form, this control looks like:

![flowchart](https://github.com/user-attachments/assets/72e4a0de-c771-4d85-a82d-be261a7b5fcf)

More about the controllers can be read in the controllers folder.

# Physics Simulation

The physics simulation is centered around the first principles dynamics of a quadcopter. One paper that significantly aided in understanding how quadcopters function and how we can model the dynamics is linked here:

https://www.roboticsbook.org/S70_drone_intro.html

More information on how the physics were implemented can be found in the sim folder.
