
disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    for i = 1 : 6
        [r,joint(i)]=sim.simxGetObjectHandle(clientID, strcat('UR10_joint', int2str(i)) , sim.simx_opmode_blocking);
    end
    PID = [[30 1 0],
            [1 1 1],
            [1 1 1],
            [1 1 1],
            [1 1 1],
            [1 1 1]];
    default_force = [0 185.8 69 0 -0.5 0];
    force = [0 0 0 0 0 0];
    dt = 0.01;
    num_of_sample = 20;
    error = zeros(6,num_of_sample);
    aaa = [];
%     for i = 1:length(q)
    while(true)
        for j = 1 : 6
            %sim.simxSetJointTargetVelocity(clientID, joint(j), 1, sim.simx_opmode_blocking);
            %sim.simxSetJointTargetPosition(clientID, joint(j), q(j,i), sim.simx_opmode_blocking);
            [er, pos(j)] = sim.simxGetJointPosition(clientID, joint(j), sim.simx_opmode_blocking);
        end
            error = error(:,2:end); % bỏ giá trị đầu tiên (giá trị cũ nhất) ra khỏi cửa sổ
            error(:,num_of_sample) = pos - 0; % đưa giá trị mới nhất vào cuối cửa sổ
            
%             for k = 1 : 6
                force(1) = default_force(1) - PID(1,1)*error(1,end) + PID(1,2)*sum(error(1,:)) + PID(1,3)*(error(1,end) - error(1,end-1))/dt;
%             end
            aaa(end+1) = force(1);
            sim.simxSetJointForce(clientID, joint(1), force(1), sim.simx_opmode_blocking);
%             [er, force(j)] = sim.simxGetJointForce(clientID, joint(j), sim.simx_opmode_blocking);
%         end
    end
%     end
    
%     pos
%     force
%     [er, aa] = sim.simxGetStringSignal(clientID, 'ahihi', sim.simx_opmode_streaming)
%     while(true)
%         [er, aa] = sim.simxGetStringSignal(clientID, 'ahihi', sim.simx_opmode_buffer)
%     end
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended');