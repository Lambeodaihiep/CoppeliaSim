%clc;
%clear;
disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
%Diem goc cho dong hoc nguoc
xx_0 = -1.0024;
yy_0 = -0.2653;
zz_0 = 0.38823;
X_0 = [-yy_0; xx_0; zz_0]; % Vector vi tri E
% Gia tri gan dung cua cac goc khop ban dau
q1_0 = 1.6708;
q2_0 = 0.62;
q3_0 = 1.208; 
q4_0 = -0.25;
q5_0 = -1.5708;
q6_0 = 0;
%% Tinh chinh xac gia tri goc khop ban dau q_0
if (clientID>-1)
    disp('Connected to remote API server');
    qqqE = zeros(6,length(qqq));
    dqE = zeros(6,length(qqq));
    % Retreive joint handles from CoppeliaSim
    for i = 1 : 6
        [r,joint(i)]=sim.simxGetObjectHandle(clientID, strcat('UR10_joint', int2str(i)) , sim.simx_opmode_blocking);
    end
    for ii = 1:1:length(qqq)
%         promptForward = {'Enter xE','Enter yE', 'Enter zE'};
%         dlgtitleForward = 'Inverse Kinematic for UR10';
%         inputPos = inputdlg(promptForward,dlgtitleForward);
%         inputPos = str2double(inputPos);
%         X_0 = [inputPos(2); inputPos(1); inputPos(3)] % Vector vi tri E
        %Diem goc cho dong hoc nguoc
        xx_0 = qqq(1,ii);
        yy_0 = qqq(2,ii);
        zz_0 = qqq(3,ii);
        % Do he toa do cua Vrep khac nen moi de nhu the nay
        X_0 = [-yy_0; xx_0; zz_0];
        for n = 1: 1: 10^5
            Jnd_0 = computeJnd(q1_0, q2_0, q3_0, q4_0, q5_0, q6_0);
            [xE_0, yE_0, zE_0] = UR10_forwardKinematic(q1_0, q2_0, q3_0, q4_0, q5_0, q6_0);% tinh lai xx_0, yy_0 theo q_0
            XX_0 = [xE_0; yE_0; zE_0];
            delta_q_0 = Jnd_0*(X_0 - XX_0);% Tinh gia tri hieu chinh delta_q_0
            % Tinh lai cac gia tri q_0 hieu chinh
            q1_0 = q1_0 + delta_q_0(1, 1);
            q2_0 = q2_0 + delta_q_0(2, 1);
            q3_0 = q3_0 + delta_q_0(3, 1);
            q4_0 = q4_0 + delta_q_0(4, 1);
            q5_0 = q5_0 + delta_q_0(5, 1);
            q5_0 = q5_0 + delta_q_0(6, 1);
            % Khai bao do chinh xac can thiet va ta vong lap tinh toan
            ss = 10^(-10);
            if abs(delta_q_0(1, 1)) < ss
                if abs(delta_q_0(2, 1)) < ss
                    if abs(delta_q_0(3, 1)) < ss
                        if abs(delta_q_0(4, 1)) < ss
                            if abs(delta_q_0(5, 1)) < ss
                                if abs(delta_q_0(6, 1)) < ss
                                    break
                                end
                            end
                        end
                    end
                end
            end
            n;
        end
        % Xac nhan cac gia tri q_0 chinh xac sau khi hieu chinh
        q1 = q1_0;
        q2 = q2_0;
        q3 = q3_0;
        q4 = q4_0;
        q5 = q5_0;
        q6 = q6_0;
        %q = [q1; q2; q3; q4; q5; q6];
        qqqE(1,ii) = q1;
        qqqE(2,ii) = q2;
        qqqE(3,ii) = q3;
        qqqE(4,ii) = q4;
        qqqE(5,ii) = q5;
        qqqE(6,ii) = q6;
%         dq = Jnd_0 * dqqq(:,ii);
%         dqE(1,ii) = dq(1);
%         dqE(2,ii) = dq(2);
%         dqE(3,ii) = dq(3);
%         dqE(4,ii) = dq(4);
%         dqE(5,ii) = dq(5);
%         dqE(6,ii) = dq(6);
%         for j = 1 : 6
%             sim.simxSetJointTargetPosition(clientID, joint(j), q(j), sim.simx_opmode_blocking);
%         end   
    end
    for i = 1:1:length(qqqE)
        for j = 1 : 6
            %sim.simxSetJointTargetVelocity(clientID, joint(j), 1, sim.simx_opmode_blocking);
            sim.simxSetJointTargetPosition(clientID, joint(j), qqqE(j,i), sim.simx_opmode_blocking);
        end
    end
else
    disp('Failed connecting to remote API server');
end
sim.delete();% call the destructor!
disp('Program ended');