function [ q ] = JointConf2JointVec( jointConf )
% robot : RigidBodyTree
% q : joint vector
% jointConf  : configuration structue
for i=1:length(jointConf)
    q(i)=jointConf(i).JointPosition;
end

end

