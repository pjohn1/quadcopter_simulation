# quadcopter_simulation
Physics &amp; controls simulation of a quadcopter built from scratch using ROS2, RVIZ, and some 3d modeling from an iPhone camera

# Motivation

The goal of this project is to create a sandbox environment for me to practice different topics in control theory. Additionally, I challenged myself to code all of the ROS nodes in C++ to improve my proficiency in the language.

# Control Structure

During my 16.30 (Feedback Control Systems) class at MIT, the final project was to control a DJI Tello drone. Our control variables were the velocity vectors and yaw rate. I wanted to replicate this in this project. Thus, the high-level control is centered around velocity vector input which is fed into a low-level controller that translates the velocity vectors into individual motor forces to control roll, pitch, and vertical acceleration. I found this approach to be much more intuitive as the velocity control can then just be based on the pose difference between the drone and the goal pose. In flowchart form, this control looks like:

![flowchart](https://github.com/user-attachments/assets/72e4a0de-c771-4d85-a82d-be261a7b5fcf)

More about the controllers can be read in the controllers folder.

