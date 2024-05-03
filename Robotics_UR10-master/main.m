%% 6DOF Robot Arm Project v5
% student name: Muhammad Yehia Muhammad

%% connect matlab with v-rep
vrep=remApi('remoteApi');
vrep.simxFinish(-1);
ClientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
if ClientID < 0
    disp('Failed connecting to remote API server. Exiting.');
    vrep.delete();
    return;
end
fprintf('Connection %d to remote API server open.\n', ClientID);

%% joint handles
[res,joint1] = vrep.simxGetObjectHandle(ClientID,'UR10_joint1',vrep.simx_opmode_blocking);
[res,joint2] = vrep.simxGetObjectHandle(ClientID,'UR10_joint2',vrep.simx_opmode_blocking);
[res,joint3] = vrep.simxGetObjectHandle(ClientID,'UR10_joint3',vrep.simx_opmode_blocking);
[res,joint4] = vrep.simxGetObjectHandle(ClientID,'UR10_joint4',vrep.simx_opmode_blocking);
[res,joint5] = vrep.simxGetObjectHandle(ClientID,'UR10_joint5',vrep.simx_opmode_blocking);
[res,joint6] = vrep.simxGetObjectHandle(ClientID,'UR10_joint6',vrep.simx_opmode_blocking);
[res,gripper] = vrep.simxGetObjectHandle(ClientID,'RG2_openCloseJoint',vrep.simx_opmode_blocking);


%% reset joint position
[res] = vrep.simxSetJointTargetPosition(ClientID,joint1,0*(pi/180),vrep.simx_opmode_blocking);
[res] = vrep.simxSetJointTargetPosition(ClientID,joint2,0*(pi/180),vrep.simx_opmode_blocking);
[res] = vrep.simxSetJointTargetPosition(ClientID,joint3,0*(pi/180),vrep.simx_opmode_blocking);
[res] = vrep.simxSetJointTargetPosition(ClientID,joint4,0*(pi/180),vrep.simx_opmode_blocking);
[res] = vrep.simxSetJointTargetPosition(ClientID,joint5,0*(pi/180),vrep.simx_opmode_blocking);
[res] = vrep.simxSetJointTargetPosition(ClientID,joint6,0*(pi/180),vrep.simx_opmode_blocking);
pause(0.5);


%% Use inverse kinematics function of ur10 robot to generat the joints position
%q1 = inversekin(-1,0.06,0.348,0,0,0);
q1 = [1 1 1 1 1 1];
%object postition

%% pass the joint position
for i = 1:1:180
    [res] = vrep.simxSetJointTargetPosition(ClientID,joint6,i*(pi/180),vrep.simx_opmode_blocking);
    pause(0.005);
end

% [res] = vrep.simxSetJointTargetPosition(ClientID,joint2,q1(2)+(pi/2),vrep.simx_opmode_blocking);
% pause(0.05);
% 
% [res] = vrep.simxSetJointTargetPosition(ClientID,joint3,q1(3),vrep.simx_opmode_blocking);
% pause(0.05);
% 
% [res] = vrep.simxSetJointTargetPosition(ClientID,joint4,q1(4)-(pi/2),vrep.simx_opmode_blocking);
% pause(0.05);
% 
% [res] = vrep.simxSetJointTargetPosition(ClientID,joint5,q1(5),vrep.simx_opmode_blocking);
% pause(0.05);
% 
% [res] = vrep.simxSetJointTargetPosition(ClientID,joint6,q1(6),vrep.simx_opmode_blocking);
% pause(0.05);

%[res] = vrep.simxSetJointForce(ClientID,gripper,20,vrep.simx_opmode_blocking);

%[res] = vrep.simxSetJointTargetVelocity(ClientID,gripper,0.05,vrep.simx_opmode_blocking);

pause(0.1);


