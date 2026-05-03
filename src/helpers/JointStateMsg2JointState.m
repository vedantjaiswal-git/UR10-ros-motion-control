function [ jointState, jointVel ] = JointStateMsg2JointState( robot, jointMsg )
%  converts joint state message to joint state vector and joint velocity
% !!! the order of joints in msg is not equal to order in URDF
jointConf=robot.homeConfiguration; % empty joint configuration to determine joint names
jointState=zeros(1,length(jointConf));
jointVel=zeros(1,length(jointConf));
for i=1:length(jointConf)
% match jointConf names with jointMsg
    ind=JointName2Index(jointConf(i).JointName,jointMsg);
    if (ind>0)
      jointState(i)=jointMsg.Position(ind);
      jointVel(i)=jointMsg.Velocity(ind);
    end
end
end

