function [ ind ] = JointName2Index( jointName, jointStateMsg )
% computes the index in a joint state message that corresponds to a joint name
ind=0;
for i=1:length(jointStateMsg.Name)
    if strcmp(jointName,jointStateMsg.Name{i})
        ind=i;
        break;
    end
end
end

