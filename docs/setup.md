# Environment Setup

## Software Requirements

The project was developed and tested using:

- Ubuntu 20.04
- ROS Noetic
- MATLAB R2024a
- MATLAB Robotics System Toolbox
- Gazebo
- RViz

## ROS Simulation

Launch the UR10 simulation:

```bash
roslaunch ur_launch ur10_sim_gazebo.launch rqt:=false
```

## MATLAB Connection

Connect MATLAB to the ROS master:

```matlab
rosinit('http://<ROS_MASTER_IP>:11311')
```

## Verify Topics

Use:

```matlab
rostopic list
```

Expected topics include:

- /ur10/joint_states
- /ur10/vel_based_pos_traj_controller/command