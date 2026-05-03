function [ followJointTrajectoryMsg ] = JointVec2FollowJointTrajectoryMsg(robot, q, t, qvel, qacc)
%  converts sequence of joint configuration q_1,...,q_m to  joint Trajectory message
% q : n x m , matrix of waypoints,  m joint space dimension, n #waypoints
% t : n x 1 vector of time stamps of motion in seconds
% qvel : n x m , matrix of joint velocities,  m joint space dimension, n #waypoints
% qacc : n x m , matrix of joint accelerations,  m joint space dimension, n #waypoints

if (nargin < 5)
    qacc=0*q;
end

if (nargin < 4)
    qvel=0*q;
end

qtol=-1;
qveltol=-1;
qacctol=-1;

jointConf=homeConfiguration(robot);
followJointTrajectoryMsg=rosmessage('control_msgs/FollowJointTrajectoryGoal');
followJointTrajectoryMsg.Trajectory=JointVec2JointTrajectoryMsg(robot, q, t, qvel, qacc);
for j=1:size(q,2)
    followJointTrajectoryMsg.GoalTolerance(j)=rosmessage('control_msgs/JointTolerance');
    followJointTrajectoryMsg.PathTolerance(j)=rosmessage('control_msgs/JointTolerance');
    % followJointTrajectoryMsg.GoalTimeTolerance(j)=rosmessage('std_msgs/Duration');
    followJointTrajectoryMsg.GoalTimeTolerance.Sec=0;
    followJointTrajectoryMsg.GoalTimeTolerance.Nsec=int32(1e7);  % 10 ms
    followJointTrajectoryMsg.GoalTolerance(j).Name=jointConf(j).JointName;
    followJointTrajectoryMsg.PathTolerance(j).Name=jointConf(j).JointName;
    followJointTrajectoryMsg.GoalTolerance(j).Position=qtol;
    followJointTrajectoryMsg.GoalTolerance(j).Velocity=qveltol;
    followJointTrajectoryMsg.GoalTolerance(j).Acceleration=qacctol;
    followJointTrajectoryMsg.PathTolerance(j).Position=qtol;
    followJointTrajectoryMsg.PathTolerance(j).Velocity=qveltol;
    followJointTrajectoryMsg.PathTolerance(j).Acceleration=qacctol;

%     for i=1:size(q,1)
%        followJointTrajectoryMsg.GoalTolerance(j).Position(i)=qtol;
%        followJointTrajectoryMsg.GoalTolerance(j).Velocity(i)=qveltol;
%        followJointTrajectoryMsg.GoalTolerance(j).Acceleration(i)=qacctol;
%        followJointTrajectoryMsg.PathTolerance(j).Position(i)=qtol;
%        followJointTrajectoryMsg.PathTolerance(j).Velocity(i)=qveltol;
%        followJointTrajectoryMsg.PathTolerance(j).Acceleration(i)=qacctol;
%     end
end

end

