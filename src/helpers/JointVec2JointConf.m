function [ jointConf ] = JointVec2JointConf( robot, q )
% robot : RigidBodyTree
% q : joint vector
% configuration : configuration structue
% create default configuration struct
jointConf=homeConfiguration(robot);
% assert(isequal(length(jointConf),length(q)),'mismatch of length of robot joint variables and joint vector');
% for i=1:length(jointConf)
for i=1:length(q)   
    jointConf(i).JointPosition=q(i);
end

end

