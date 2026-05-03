%% ROS Robot Motion Control
% UR10 Motion Control Project
% Author: Vedant Jaiswal
% MSc Automation and Robotics, TU Dortmund University
%
% Description:
% This script implements end-to-end motion control of the UR10 robotic
% manipulator in ROS using MATLAB. It covers inverse kinematics,
% trajectory generation, topic-based control, action-based control,
% and real-time joint state monitoring in Gazebo simulation.
%
% The workflow follows a structured pipeline:
% 1. Inverse kinematics computation for target pose
% 2. ROS initialization and communication setup
% 3. Point-to-point trajectory execution (topic interface)
% 4. Real-time joint state monitoring and logging
% 5. Multi-waypoint trajectory execution
% 6. Action server-based trajectory execution

%% 1. Inverse Kinematics
% Create inverse kinematics solver for UR10 rigid body tree
ur10=importrobot('ur10.urdf');
ur10.showdetails();
ikur10 = robotics.InverseKinematics('RigidBodyTree',ur10);

% Define desired end-effector pose (position + orientation)
targetPosition=[0.6, 0, 1.0];
targetOrientation=[pi/4, pi/4, -pi/4];

% Construct homogeneous transformation matrix
t1=trvec2tform(targetPosition);
t2=eul2tform(targetOrientation, "ZYZ");
targetposeur10=t1*t2;

disp('Target Pose Transformation Matrix:');
disp(targetposeur10);

% Solve inverse kinematics
weights = ones(6,1);
initialpose = JointVec2JointConf(ur10,[0 1 -2 2 1 1]);
[targetsol, solnInfo] = ikur10('ee_link',targetposeur10,weights,initialpose);
ur10.show(targetsol);

%% 2. ROS Initialization
% Reset and initialize ROS connection
rosshutdown
rosinit()

% Inspect available topics
rostopic list
rostopic info /ur10/joint_states
rostopic info ur10/vel_based_pos_traj_controller/command

%% 3. Joint State Subscriber
% Create subscriber for joint states
jointStateSub = rossubscriber('/ur10/joint_states', 'sensor_msgs/JointState');

% Receive and inspect current joint state
jointStateMsg = receive(jointStateSub, 10);
jointPositions = jointStateMsg.Position;

disp('Joint Positions (Shoulder to Wrist):');
disp(jointPositions(1:6));

%% 4. Topic-based Trajectory Publisher
jointTrajectoryPub = rospublisher('/ur10/vel_based_pos_traj_controller/command', ...
    'trajectory_msgs/JointTrajectory');

% Extract target joint configuration
q_target = [targetsol.JointPosition];

% Define trajectory timing parameters
t_target = 2.5;
t_offset = 3;

% Define start configuration
q_start = [ur10.homeConfiguration.JointPosition];

t = [0; t_target];
q = [q_start; q_target];
qvel = zeros(size(q));
qacc = zeros(size(q));

% Generate and send trajectory message
jointTrajectoryMsg = JointVec2JointTrajectoryMsg(ur10, q, t, qvel, qacc);
send(jointTrajectoryPub, jointTrajectoryMsg);

pause(t_target + t_offset);
disp('Robot reached the target configuration.');

%% 5. Real-time Joint State Monitoring
rate = 50;
rateObj = robotics.Rate(rate);
tf = t_target + 2.0;
rateObj.reset();

N = tf * rate;
timeStamp = zeros(N, 1);
jointStateStamped = zeros(N, 6);
jointVelStamped = zeros(N, 6);

jointStateMsg = receive(jointStateSub);
t0 = double(jointStateMsg.Header.Stamp.Sec) + ...
     double(jointStateMsg.Header.Stamp.Nsec) * 10^-9;

for i = 1:N
    jointStateMsg = receive(jointStateSub);
    
    [jointState, jointVel] = JointStateMsg2JointState(ur10, jointStateMsg);
    
    jointStateStamped(i, :) = jointState;
    jointVelStamped(i, :) = jointVel;
    
    timeStamp(i) = double(jointStateMsg.Header.Stamp.Sec) + ...
                   double(jointStateMsg.Header.Stamp.Nsec) * 10^-9 - t0;
    
    waitfor(rateObj);
end

disp('Trajectory execution and monitoring complete.');

figure;
subplot(2,1,1);
plot(timeStamp, jointStateStamped);
xlabel('Time (s)'); ylabel('Joint Positions (rad)');
grid on;
title('Joint State Evolution');

subplot(2,1,2);
plot(timeStamp, jointVelStamped);
xlabel('Time (s)'); ylabel('Joint Velocities (rad/s)');
grid on;
title('Joint Velocity Evolution');

%% 6. Multi-waypoint Trajectory (Non-0 Joint Velocity)
qf = [-1 0 -1 1 0 2];
t_targets = [2.5; 6.0];
q_targets = [q_target; qf];

qvel = zeros(size(q_targets));
qvel(1, :) = [-0.4 0.0 0.0 -0.3 0.0 0.0];

jointTrajectoryMsg = JointVec2JointTrajectoryMsg(ur10, q_targets, t_targets, qvel, zeros(size(q_targets)));
send(jointTrajectoryPub, jointTrajectoryMsg);

%% 7. Monitoring Multi-waypoint Motion
rate = 100;
rateObj = robotics.Rate(rate);
tf = t_targets(end) + 2.0;
rateObj.reset();

N = tf * rate;
timeStamp = zeros(N, 1);
jointStateStamped = zeros(N, 6);
jointVelStamped = zeros(N, 6);

jointStateMsg = jointStateSub.receive();
t0 = double(jointStateMsg.Header.Stamp.Sec) + ...
     double(jointStateMsg.Header.Stamp.Nsec) * 10^-9;

for i = 1:N
    jointStateMsg = jointStateSub.receive();
    
    [jointState, jointVel] = JointStateMsg2JointState(ur10, jointStateMsg);
    
    jointStateStamped(i, :) = jointState;
    jointVelStamped(i, :) = jointVel;
    
    current_time = double(jointStateMsg.Header.Stamp.Sec) + ...
                   double(jointStateMsg.Header.Stamp.Nsec) * 10^-9;
    timeStamp(i) = current_time - t0;
    
    waitfor(rateObj);
end

disp('Motion completed. Plotting results...');

figure;
subplot(2,1,1);
plot(timeStamp, jointStateStamped);
title('Joint States for Multi-waypoint Motion');
grid on;

subplot(2,1,2);
plot(timeStamp, jointVelStamped);
title('Joint Velocities for Multi-waypoint Motion');
grid on;

%% 8. Action Server Interface
rosactionList = rosaction('list');

action_name = '/ur10/vel_based_pos_traj_controller/follow_joint_trajectory';
followJointTrajectoryActClient = rosactionclient(action_name);
waitForServer(followJointTrajectoryActClient);

goalMsg = rosmessage(followJointTrajectoryActClient);

followJointTrajectoryMsg = rosmessage('control_msgs/FollowJointTrajectoryGoal');
trajectoryMsg = rosmessage('trajectory_msgs/JointTrajectory');
trajectoryMsg.JointNames = {'shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'};

for i = 1:size(q_targets,1)
    point = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    point.Positions = q_targets(i,:);
    point.TimeFromStart = rosduration(t_targets(i));
    point.Velocities = zeros(1,6);
    trajectoryMsg.Points(i) = point;
end

followJointTrajectoryMsg.Trajectory = trajectoryMsg;

[resultMsg, resultState] = followJointTrajectoryActClient.sendGoalAndWait(followJointTrajectoryMsg);

%% 9. Non-blocking Action Execution
followJointTrajectoryActClient.sendGoal(followJointTrajectoryMsg);

disp('Non-blocking trajectory execution started.');

%% End of Script
disp('Pipeline execution complete.');
rosshutdown;

