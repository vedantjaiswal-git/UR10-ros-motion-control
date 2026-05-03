# UR10 Robot Motion Control using ROS and MATLAB

## Overview

This project implements kinematic modeling, inverse kinematics, and trajectory control of a UR10 robotic manipulator using ROS and MATLAB Robotics System Toolbox. The system integrates robot modeling via URDF, numerical inverse kinematics, and trajectory execution using both ROS topic and action interfaces. The simulation is performed in Gazebo with visualization in RViz.

The project demonstrates end-to-end robotic motion control including trajectory generation, execution, and real-time joint state monitoring.

---

## Objectives

The primary objectives of this project are:

* Implementation of inverse kinematics for a UR10 manipulator using MATLAB’s numerical IK solver.
* Generation of joint-space trajectories for point-to-point and multi-waypoint motion.
* Execution of trajectories using ROS topic-based and action-based interfaces.
* Real-time subscription and logging of joint states and joint velocities.
* Analysis of motion profiles including position and velocity continuity.

---

## System Architecture

The system consists of the following components:

* MATLAB: Motion planning, inverse kinematics, and ROS communication.
* ROS: Middleware for communication between nodes.
* Gazebo: Physics-based simulation environment for UR10 robot.
* RViz: Visualization of robot state and transformations.
* UR10 URDF model: Robot kinematic and dynamic representation.

Communication is established through ROS publishers and subscribers, with trajectory commands sent via topic and action interfaces.

---

## Features

### Inverse Kinematics

A numerical inverse kinematics solver is used to compute joint configurations for a given end-effector pose. The solver minimizes pose error in position and orientation space using weighted least squares optimization.

### Trajectory Generation

Joint trajectories are generated using quintic polynomial interpolation ensuring continuity in position, velocity, and acceleration. The system supports:

* Point-to-point motion
* Multi-waypoint trajectories
* Velocity-constrained waypoint transitions

### ROS Topic Interface

Trajectory commands are published to the following topic:

/ur10/vel_based_pos_traj_controller/command

This interface provides a fire-and-forget execution model without feedback monitoring.

### ROS Action Interface

Trajectory execution is also implemented using the ROS action server:

/ur10/vel_based_pos_traj_controller/follow_joint_trajectory

This interface provides execution feedback, goal status tracking, and result validation.

### Real-Time Monitoring

Joint state data is obtained via subscription to:

/ur10/joint_states

The system records:

* Joint positions
* Joint velocities
* Execution timestamps

---

## Requirements

### Software

* MATLAB (Robotics System Toolbox)
* ROS Noetic
* Gazebo
* RViz
* Ubuntu 20.04 (or compatible ROS environment)

### MATLAB Dependencies

* Robotics System Toolbox
* Navigation Toolbox (optional for extensions)

---

## Setup Instructions

### 1. Start ROS and Gazebo Simulation

```bash
roslaunch ur_launch ur10_sim_gazebo.launch rqt:=false
```

### 2. Initialize ROS connection in MATLAB

```matlab
rosinit(<ROS_MASTER_IP>)
```

### 3. Run Main Control Script

```matlab
Final_UR10_Motion_Control
```

---

## Results

The system successfully demonstrates:

* Accurate inverse kinematics solutions within workspace constraints
* Smooth joint-space trajectory execution
* Continuous velocity profiles during motion
* Multi-waypoint trajectory tracking with intermediate stopping conditions
* Reliable real-time synchronization between MATLAB and ROS simulation

---

## Notes

* Topic-based control is non-blocking but lacks execution feedback.
* Action-based control provides full lifecycle monitoring including goal status and feedback callbacks.
* Velocity constraints at waypoints significantly affect trajectory smoothness.

---

## Future Improvements

* Modularization into a MATLAB package structure
* Integration with real UR10 hardware
* Implementation of Cartesian-space trajectory planning
* Addition of obstacle-aware motion planning
* Performance benchmarking of IK convergence

---

## License

This project is intended for academic and educational use. You may adapt it for research or portfolio purposes with appropriate attribution.
