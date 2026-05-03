# Methodology

## 1. Robot Modeling

The UR10 robot is imported into MATLAB using its URDF model.

## 2. Inverse Kinematics

A numerical inverse kinematics solver computes the joint configuration required to achieve a desired end-effector pose.

The solver minimizes position and orientation error using weighted least squares optimization.

## 3. Topic-Based Trajectory Control

Trajectory commands are published using ROS JointTrajectory messages.

This provides point-to-point and waypoint-based motion execution.

## 4. Action-Based Trajectory Control

Trajectory execution is monitored using ROS action clients.

This enables:

- Goal monitoring
- Feedback callbacks
- Result validation

## 5. Real-Time Monitoring

Joint positions and velocities are recorded from:

/ur10/joint_states

Recorded data is used for motion profile analysis.