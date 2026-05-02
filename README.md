UR10 Robot Motion Control using ROS and MATLAB

Overview:

This repository contains an implementation of motion planning and control for a UR10 industrial robotic manipulator using MATLAB and ROS. The project demonstrates a complete robotics control pipeline including inverse kinematics, trajectory generation, and execution in a Gazebo simulation environment.
The system integrates MATLAB (Robotics System Toolbox) with ROS Noetic to communicate with a simulated UR10 robot. Motion execution is implemented using both ROS topic-based and action-based interfaces, allowing comparison between open-loop trajectory execution and feedback-driven control.

Objectives:

The main objectives of this project are:

1. Implementation of inverse kinematics for a 6-DOF robotic manipulator
2. Development of joint-space trajectory planning methods
3. Execution of robot motion in ROS Gazebo simulation
4. Integration of MATLAB with ROS communication framework
5. Comparison of topic-based and action-based trajectory control methods
6. Real-time monitoring of joint states and velocities
7. System Description

The system consists of three main components:

1. MATLAB Environment
Used for inverse kinematics computation, trajectory generation, and ROS communication.

2. ROS Framework
Provides communication infrastructure using publishers, subscribers, and action servers.

3. Gazebo Simulation
Simulates the UR10 robot in a physically realistic environment, including dynamics and joint controllers.

Key Features:

-UR10 robot simulation in Gazebo and RViz
-Numerical inverse kinematics using MATLAB Robotics Toolbox
-Point-to-point and multi-waypoint trajectory generation
-Quintic polynomial-based joint trajectory execution
-ROS topic-based trajectory control interface
-ROS action client-server implementation with feedback handling
-Real-time joint state acquisition and monitoring

Implementation Details:

The project is structured into the following stages:

-Inverse kinematics computation for target end-effector poses
-Definition of joint-space trajectories with time parametrization
-Execution of trajectories using ROS trajectory controllers
-Monitoring of joint states via ROS subscribers
-Implementation of both blocking and non-blocking action-based control
-Visualization and analysis of joint position and velocity profiles

Technologies Used:
MATLAB (Robotics System Toolbox)
ROS Noetic
Gazebo Simulator
RViz
UR10 URDF Model

Author:
Vedant Jaiswal
M.Sc. Automation and Robotics
TU Dortmund University
