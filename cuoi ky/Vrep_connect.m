rotationX = @(t) [1 0 0; 0 cosd(t) -sind(t); 0 sind(t) cosd(t)] ;
rotationY = @(t) [cosd(t) 0 sind(t); 0 1 0; -sind(t) 0 cosd(t)] ;
rotationZ = @(t) [cosd(t) -sind(t) 0; sind(t) cosd(t) 0; 0 0 1] ;
% toa_do_tron = [0 0.80 0.350];
% toa_do_sao = [-0.088 0.711 0.350];
% toa_do_tim = [0.093 0.709 0.350];
% toa_do_tamgiac = [-0.088 0.887 0.350];
% toa_do_vuong = [0.089 0.889 0.350];
height_of_robot_base = 0.5;

printTimeInterval=0.006; %correct time interval for printing variables in CoppeliaSim
disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
    disp('Connected to remote API server');
    % first call for all signal
    [er, gripper_open] = sim.simxGetInt32Signal(clientID, 'RG2_open', sim.simx_opmode_streaming);
    [er, done] = sim.simxGetInt32Signal(clientID, 'Done', sim.simx_opmode_streaming);
    [~, realPosition] = sim.simxGetStringSignal(clientID, 'realPosition', sim.simx_opmode_streaming);
    for i = 1 : 6
        [r,joint(i)]=sim.simxGetObjectHandle(clientID, strcat('UR10_joint', int2str(i)) , sim.simx_opmode_blocking);
    end
    jointAnglesHome = [0; 0; 0; 0; 0; 0];
    toolOrientationHome = [0 90 -90];
    homepoint = [1.18437 0.44429 -0.00688];
    type = 'line';
    orientations_home = [0 pi/2 -pi/2];
    orientations_target = [0 -pi -pi/2];
    force = [0 0 0 0 0 0];
    % includeOrientation = true;
%% tròn
    getPosition_fromCamera;
    [q, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, 'curve', 'pick');
    q_circle = q;
    pd = makedist('Normal','mu',0,'sigma',0.5);
    r = random(pd,[1,30]);
    r = sort(r,"descend","ComparisonMethod","abs");
    r(length(q)) = 0;
    r = r*pi/180;
    q = q+r;
    qq = q(:); qq(2:end+1) = qq;
%     dqq = qd(:); dqq(2:end+1) = dqq;
%     ddqq = qdd(:); ddqq(2:end+1) =ddqq;
    trajectory = sim.simxPackFloats(qq');
%     velocity = sim.simxPackFloats(dqq');
%     accelerator = sim.simxPackFloats(ddqq');
    sim.simxSetStringSignal(clientID, 'trajectory', trajectory, sim.simx_opmode_oneshot);
%     sim.simxSetStringSignal(clientID, 'velocity', velocity, sim.simx_opmode_oneshot);
%     sim.simxSetStringSignal(clientID, 'accelerator', accelerator, sim.simx_opmode_oneshot);
    pause(2);
    
    sim.simxSetIntegerSignal(clientID, 'LetMove', 1, sim.simx_opmode_blocking);
    pause(printTimeInterval);
    sim.simxSetIntegerSignal(clientID, 'LetMove', 0, sim.simx_opmode_blocking);
    % Tín hiệu đã gắp vật
    q_force_circle = zeros(6,1);
    while(true)
        for i = 1 : 6
            [~,force(i)]=sim.simxGetJointForce(clientID, joint(i), sim.simx_opmode_blocking);
        end
        q_force_circle(:,end+1) = force;
        [er, gripper_open] = sim.simxGetInt32Signal(clientID, 'RG2_open', sim.simx_opmode_buffer);
        if(gripper_open == 0)
            break
        end
    end

    % Lấy vị trí thực của các khớp
    [~, realPosition] = sim.simxGetStringSignal(clientID, 'realPosition', sim.simx_opmode_buffer);
    realPosition = sim.simxUnpackFloats(realPosition);
    q_circle_real = zeros(6,1);
    for i = 1:length(realPosition)/6
        q_circle_real(1, end+1) = realPosition(6*(i-1)+1);
        q_circle_real(2, end) = realPosition(6*(i-1)+2);
        q_circle_real(3, end) = realPosition(6*(i-1)+3);
        q_circle_real(4, end) = realPosition(6*(i-1)+4);
        q_circle_real(5, end) = realPosition(6*(i-1)+5);
        q_circle_real(6, end) = realPosition(6*(i-1)+6);
    end
    % Đưa vật đến đúng vị trí trên hộp
    targetpoint = [0 0.80 0.38-0.5+0.3];
    [q1, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, type, "None");
    
    targetpoint = [0 0.80 0.38-0.5];
    orientations_target = [0 -pi -pi/2];
    [q2, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, type, "None");
    q = [q1 q2];
    q_circle = [q_circle q]; % Quỹ đạo mong muốn của từng khớp
    pd = makedist('Normal','mu',0,'sigma',0.5);
    r = random(pd,[1,30]);
    r = sort(r,"descend","ComparisonMethod","abs");
    r(length(q)) = 0;
    r = r*pi/180;
    q = q+r;
    qq = q(:); qq(2:end+1) = qq;
    trajectory = sim.simxPackFloats(qq');
    sim.simxSetStringSignal(clientID, 'trajectory', trajectory, sim.simx_opmode_oneshot);
    pause(2);
    
    sim.simxSetIntegerSignal(clientID, 'Drop', 1, sim.simx_opmode_blocking);
    pause(printTimeInterval);
    sim.simxSetIntegerSignal(clientID, 'Drop', 0, sim.simx_opmode_blocking);
    % Tín hiệu đã thả vật
    while(true)
        for i = 1 : 6
            [~,force(i)]=sim.simxGetJointForce(clientID, joint(i), sim.simx_opmode_blocking);
        end
        q_force_circle(:,end+1) = force;
        [er, done] = sim.simxGetInt32Signal(clientID, 'Done', sim.simx_opmode_buffer);
        if(done == 1)
            sim.simxSetInt32Signal(clientID, 'Done', 0, sim.simx_opmode_blocking);
            break
        end
    end
    % Lấy vị trí thực của các khớp
    [~, realPosition] = sim.simxGetStringSignal(clientID, 'realPosition', sim.simx_opmode_buffer);
    realPosition = sim.simxUnpackFloats(realPosition);
    for i = 1:length(realPosition)/6
        q_circle_real(1, end+1) = realPosition(6*(i-1)+1);
        q_circle_real(2, end) = realPosition(6*(i-1)+2);
        q_circle_real(3, end) = realPosition(6*(i-1)+3);
        q_circle_real(4, end) = realPosition(6*(i-1)+4);
        q_circle_real(5, end) = realPosition(6*(i-1)+5);
        q_circle_real(6, end) = realPosition(6*(i-1)+6);
    end
    % Vẽ đồ thị so sánh giữa quỹ đạo thực và quỹ đạo mong muốn

    
    %% sao
    getPosition_fromCamera;
    [q, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, 'curve', 'pick');
    q_star = q;
    pd = makedist('Normal','mu',0,'sigma',0.5);
    r = random(pd,[1,30]);
    r = sort(r,"descend","ComparisonMethod","abs");
    r(length(q)) = 0;
    r = r*pi/180;
    q = q+r;
    qq = q(:); qq(2:end+1) = qq;
    trajectory = sim.simxPackFloats(qq');
    sim.simxSetStringSignal(clientID, 'trajectory', trajectory, sim.simx_opmode_oneshot);
    pause(2);
    
    sim.simxSetIntegerSignal(clientID, 'LetMove', 1, sim.simx_opmode_blocking);
    pause(printTimeInterval);
    sim.simxSetIntegerSignal(clientID, 'LetMove', 0, sim.simx_opmode_blocking);    
    q_force_star = zeros(6,1);
    while(true)
        for i = 1 : 6
            [~,force(i)]=sim.simxGetJointForce(clientID, joint(i), sim.simx_opmode_blocking);
        end
        q_force_star(:,end+1) = force;
        [er, gripper_open] = sim.simxGetInt32Signal(clientID, 'RG2_open', sim.simx_opmode_buffer);
        if(gripper_open == 0)
            break
        end
    end
    % Lấy vị trí thực của các khớp
    [~, realPosition] = sim.simxGetStringSignal(clientID, 'realPosition', sim.simx_opmode_buffer);
    realPosition = sim.simxUnpackFloats(realPosition);
    q_star_real = zeros(6,1);
    for i = 1:length(realPosition)/6
        q_star_real(1, end+1) = realPosition(6*(i-1)+1);
        q_star_real(2, end) = realPosition(6*(i-1)+2);
        q_star_real(3, end) = realPosition(6*(i-1)+3);
        q_star_real(4, end) = realPosition(6*(i-1)+4);
        q_star_real(5, end) = realPosition(6*(i-1)+5);
        q_star_real(6, end) = realPosition(6*(i-1)+6);
    end
    % Đưa vật đến đúng vị trí trên hộp
    targetpoint = [-0.088 0.701 0.380-0.5+0.3];
    [q1, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, type, "None");
    
    targetpoint = [-0.088 0.701 0.380-0.5];
    orientations_target = [-pi/18 -pi -pi/2];
    [q2, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, type, "None");
    q = [q1 q2];
    q_star = [q_star q]; % Quỹ đạo mong muốn của từng khớp
    pd = makedist('Normal','mu',0,'sigma',0.5);
    r = random(pd,[1,30]);
    r = sort(r,"descend","ComparisonMethod","abs");
    r(length(q)) = 0;
    r = r*pi/180;
    q = q+r;
    qq = q(:); qq(2:end+1) = qq;
    trajectory = sim.simxPackFloats(qq');
    sim.simxSetStringSignal(clientID, 'trajectory', trajectory, sim.simx_opmode_oneshot);
    pause(2);
    
    sim.simxSetIntegerSignal(clientID, 'Drop', 1, sim.simx_opmode_blocking);
    pause(printTimeInterval);
    sim.simxSetIntegerSignal(clientID, 'Drop', 0, sim.simx_opmode_blocking);
    while(true)
        for i = 1 : 6
            [~,force(i)]=sim.simxGetJointForce(clientID, joint(i), sim.simx_opmode_blocking);
        end
        q_force_star(:,end+1) = force;
        [er, done] = sim.simxGetInt32Signal(clientID, 'Done', sim.simx_opmode_buffer);
        if(done == 1)
            sim.simxSetInt32Signal(clientID, 'Done', 0, sim.simx_opmode_blocking);
            break
        end
    end
    % Lấy vị trí thực của các khớp
    [~, realPosition] = sim.simxGetStringSignal(clientID, 'realPosition', sim.simx_opmode_buffer);
    realPosition = sim.simxUnpackFloats(realPosition);
    for i = 1:length(realPosition)/6
        q_star_real(1, end+1) = realPosition(6*(i-1)+1);
        q_star_real(2, end) = realPosition(6*(i-1)+2);
        q_star_real(3, end) = realPosition(6*(i-1)+3);
        q_star_real(4, end) = realPosition(6*(i-1)+4);
        q_star_real(5, end) = realPosition(6*(i-1)+5);
        q_star_real(6, end) = realPosition(6*(i-1)+6);
    end
    % Vẽ đồ thị so sánh giữa quỹ đạo thực và quỹ đạo mong muốn

    %% tim
    getPosition_fromCamera;
    orientations_target = [0 -pi -pi/2];
    [q, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, 'curve', 'pick');
    q_heart = q;
    pd = makedist('Normal','mu',0,'sigma',0.5);
    r = random(pd,[1,30]);
    r = sort(r,"descend","ComparisonMethod","abs");
    r(length(q)) = 0;
    r = r*pi/180;
    q = q+r;
    qq = q(:); qq(2:end+1) = qq;
    trajectory = sim.simxPackFloats(qq');
    sim.simxSetStringSignal(clientID, 'trajectory', trajectory, sim.simx_opmode_oneshot);
    pause(2);
    
    sim.simxSetIntegerSignal(clientID, 'LetMove', 1, sim.simx_opmode_blocking);
    pause(printTimeInterval);
    sim.simxSetIntegerSignal(clientID, 'LetMove', 0, sim.simx_opmode_blocking);    
    q_force_heart = zeros(6,1);
    while(true)
        for i = 1 : 6
            [~,force(i)]=sim.simxGetJointForce(clientID, joint(i), sim.simx_opmode_blocking);
        end
        q_force_heart(:,end+1) = force;
        [er, gripper_open] = sim.simxGetInt32Signal(clientID, 'RG2_open', sim.simx_opmode_buffer);
        if(gripper_open == 0)
            break
        end
    end
    % Lấy vị trí thực của các khớp
    [~, realPosition] = sim.simxGetStringSignal(clientID, 'realPosition', sim.simx_opmode_buffer);
    realPosition = sim.simxUnpackFloats(realPosition);
    q_heart_real = zeros(6,1);
    for i = 1:length(realPosition)/6
        q_heart_real(1, end+1) = realPosition(6*(i-1)+1);
        q_heart_real(2, end) = realPosition(6*(i-1)+2);
        q_heart_real(3, end) = realPosition(6*(i-1)+3);
        q_heart_real(4, end) = realPosition(6*(i-1)+4);
        q_heart_real(5, end) = realPosition(6*(i-1)+5);
        q_heart_real(6, end) = realPosition(6*(i-1)+6);
    end
    % Đưa vật đến đúng vị trí trên hộp
    targetpoint = [0.093 0.709 0.380-0.5+0.3];
    [q1, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, type, "None");
    
%     targetpoint = [0.065 0.875 0.36-0.5];
    targetpoint = [0.093 0.709 0.380-0.5];
    orientations_target = [-pi/4 -pi -pi/2];
    [q2, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, type, "None");
    q = [q1 q2];
    q_heart = [q_heart q]; % Quỹ đạo mong muốn của từng khớp
    pd = makedist('Normal','mu',0,'sigma',0.5);
    r = random(pd,[1,30]);
    r = sort(r,"descend","ComparisonMethod","abs");
    r(length(q)) = 0;
    r = r*pi/180;
    q = q+r;
    qq = q(:); qq(2:end+1) = qq;
    trajectory = sim.simxPackFloats(qq');
    sim.simxSetStringSignal(clientID, 'trajectory', trajectory, sim.simx_opmode_oneshot);
    pause(2);
    
    sim.simxSetIntegerSignal(clientID, 'Drop', 1, sim.simx_opmode_blocking);
    pause(printTimeInterval);
    sim.simxSetIntegerSignal(clientID, 'Drop', 0, sim.simx_opmode_blocking);
    while(true)
        for i = 1 : 6
            [~,force(i)]=sim.simxGetJointForce(clientID, joint(i), sim.simx_opmode_blocking);
        end
        q_force_heart(:,end+1) = force;
        [er, done] = sim.simxGetInt32Signal(clientID, 'Done', sim.simx_opmode_buffer);
        if(done == 1)
            sim.simxSetInt32Signal(clientID, 'Done', 0, sim.simx_opmode_blocking);
            break
        end
    end
    % Lấy vị trí thực của các khớp
    [~, realPosition] = sim.simxGetStringSignal(clientID, 'realPosition', sim.simx_opmode_buffer);
    realPosition = sim.simxUnpackFloats(realPosition);
    for i = 1:length(realPosition)/6
        q_heart_real(1, end+1) = realPosition(6*(i-1)+1);
        q_heart_real(2, end) = realPosition(6*(i-1)+2);
        q_heart_real(3, end) = realPosition(6*(i-1)+3);
        q_heart_real(4, end) = realPosition(6*(i-1)+4);
        q_heart_real(5, end) = realPosition(6*(i-1)+5);
        q_heart_real(6, end) = realPosition(6*(i-1)+6);
    end
    % Vẽ đồ thị so sánh giữa quỹ đạo thực và quỹ đạo mong muốn
    %% tamgiac
    getPosition_fromCamera;
    orientations_target = [0 -pi -pi/2];
    [q, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, 'curve', 'pick');
    q_triangle = q;
    pd = makedist('Normal','mu',0,'sigma',0.5);
    r = random(pd,[1,30]);
    r = sort(r,"descend","ComparisonMethod","abs");
    r(length(q)) = 0;
    r = r*pi/180;
    q = q+r;
    qq = q(:); qq(2:end+1) = qq;
    trajectory = sim.simxPackFloats(qq');
    sim.simxSetStringSignal(clientID, 'trajectory', trajectory, sim.simx_opmode_oneshot);
    pause(2);
    
    sim.simxSetIntegerSignal(clientID, 'LetMove', 1, sim.simx_opmode_blocking);
    pause(printTimeInterval);
    sim.simxSetIntegerSignal(clientID, 'LetMove', 0, sim.simx_opmode_blocking);    
    q_force_triangle = zeros(6,1);
    while(true)
        for i = 1 : 6
            [~,force(i)]=sim.simxGetJointForce(clientID, joint(i), sim.simx_opmode_blocking);
        end
        q_force_triangle(:,end+1) = force;
        [er, gripper_open] = sim.simxGetInt32Signal(clientID, 'RG2_open', sim.simx_opmode_buffer);
        if(gripper_open == 0)
            break
        end
    end
    % Lấy vị trí thực của các khớp
    [~, realPosition] = sim.simxGetStringSignal(clientID, 'realPosition', sim.simx_opmode_buffer);
    realPosition = sim.simxUnpackFloats(realPosition);
    q_triangle_real = zeros(6,1);
    for i = 1:length(realPosition)/6
        q_triangle_real(1, end+1) = realPosition(6*(i-1)+1);
        q_triangle_real(2, end) = realPosition(6*(i-1)+2);
        q_triangle_real(3, end) = realPosition(6*(i-1)+3);
        q_triangle_real(4, end) = realPosition(6*(i-1)+4);
        q_triangle_real(5, end) = realPosition(6*(i-1)+5);
        q_triangle_real(6, end) = realPosition(6*(i-1)+6);
    end
    % Đưa vật đến đúng vị trí trên hộp
    targetpoint = [-0.095 0.887 0.380-0.5+0.3];
    [q1, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, type, "None");
   
    targetpoint = [-0.095 0.887 0.380-0.5];
    orientations_target = [5*pi/18 -pi -pi/2];
    [q2, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, type, "None");
    q = [q1 q2];
    q_triangle = [q_triangle q]; % Quỹ đạo mong muốn của từng khớp
    pd = makedist('Normal','mu',0,'sigma',0.5);
    r = random(pd,[1,30]);
    r = sort(r,"descend","ComparisonMethod","abs");
    r(length(q)) = 0;
    r = r*pi/180;
    q = q+r;
    qq = q(:); qq(2:end+1) = qq;
    trajectory = sim.simxPackFloats(qq');
    sim.simxSetStringSignal(clientID, 'trajectory', trajectory, sim.simx_opmode_oneshot);
    pause(2);
    
    sim.simxSetIntegerSignal(clientID, 'Drop', 1, sim.simx_opmode_blocking);
    pause(printTimeInterval);
    sim.simxSetIntegerSignal(clientID, 'Drop', 0, sim.simx_opmode_blocking);
     while(true)
        for i = 1 : 6
            [~,force(i)]=sim.simxGetJointForce(clientID, joint(i), sim.simx_opmode_blocking);
        end
        q_force_triangle(:,end+1) = force;
        [er, done] = sim.simxGetInt32Signal(clientID, 'Done', sim.simx_opmode_buffer);
        if(done == 1)
            sim.simxSetInt32Signal(clientID, 'Done', 0, sim.simx_opmode_blocking);
            break
        end
     end
     % Lấy vị trí thực của các khớp
    [~, realPosition] = sim.simxGetStringSignal(clientID, 'realPosition', sim.simx_opmode_buffer);
    realPosition = sim.simxUnpackFloats(realPosition);
    for i = 1:length(realPosition)/6
        q_triangle_real(1, end+1) = realPosition(6*(i-1)+1);
        q_triangle_real(2, end) = realPosition(6*(i-1)+2);
        q_triangle_real(3, end) = realPosition(6*(i-1)+3);
        q_triangle_real(4, end) = realPosition(6*(i-1)+4);
        q_triangle_real(5, end) = realPosition(6*(i-1)+5);
        q_triangle_real(6, end) = realPosition(6*(i-1)+6);
    end
    % Vẽ đồ thị so sánh giữa quỹ đạo thực và quỹ đạo mong muốn
     %% vuông
    getPosition_fromCamera;
    orientations_target = [0 -pi -pi/2];
    [q, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, 'curve', 'pick');
    q_square = q;
    pd = makedist('Normal','mu',0,'sigma',0.5);
    r = random(pd,[1,30]);
    r = sort(r,"descend","ComparisonMethod","abs");
    r(length(q)) = 0;
    r = r*pi/180;
    q = q+r;
    qq = q(:); qq(2:end+1) = qq;
    trajectory = sim.simxPackFloats(qq');
    sim.simxSetStringSignal(clientID, 'trajectory', trajectory, sim.simx_opmode_oneshot);
    pause(2);
    
    sim.simxSetIntegerSignal(clientID, 'LetMove', 1, sim.simx_opmode_blocking);
    pause(printTimeInterval);
    sim.simxSetIntegerSignal(clientID, 'LetMove', 0, sim.simx_opmode_blocking); 
    q_force_square = zeros(6,1);
    while(true)
        for i = 1 : 6
            [~,force(i)]=sim.simxGetJointForce(clientID, joint(i), sim.simx_opmode_blocking);
        end
        q_force_square(:,end+1) = force;
        [er, gripper_open] = sim.simxGetInt32Signal(clientID, 'RG2_open', sim.simx_opmode_buffer);
        if(gripper_open == 0)
            break
        end
    end
    % Lấy vị trí thực của các khớp
    [~, realPosition] = sim.simxGetStringSignal(clientID, 'realPosition', sim.simx_opmode_buffer);
    realPosition = sim.simxUnpackFloats(realPosition);
    q_square_real = zeros(6,1);
    for i = 1:length(realPosition)/6
        q_square_real(1, end+1) = realPosition(6*(i-1)+1);
        q_square_real(2, end) = realPosition(6*(i-1)+2);
        q_square_real(3, end) = realPosition(6*(i-1)+3);
        q_square_real(4, end) = realPosition(6*(i-1)+4);
        q_square_real(5, end) = realPosition(6*(i-1)+5);
        q_square_real(6, end) = realPosition(6*(i-1)+6);
    end
    % Đưa vật đến đúng vị trí trên hộp
    targetpoint = [0.089 0.889 0.380-0.5+0.3];
    [q1, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, type, "None");
    targetpoint = [0.089 0.889 0.380-0.5];
    orientations_target = [pi/4 -pi -pi/2];
    [q2, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, type, "None");
    q = [q1 q2];
    q_square = [q_square q]; % Quỹ đạo mong muốn của từng khớp
    pd = makedist('Normal','mu',0,'sigma',0.5);
    r = random(pd,[1,30]);
    r = sort(r,"descend","ComparisonMethod","abs");
    r(length(q)) = 0;
    r = r*pi/180;
    q = q+r;
    qq = q(:); qq(2:end+1) = qq;
    trajectory = sim.simxPackFloats(qq');
    sim.simxSetStringSignal(clientID, 'trajectory', trajectory, sim.simx_opmode_oneshot);
    pause(2);
    
    sim.simxSetIntegerSignal(clientID, 'Drop', 1, sim.simx_opmode_blocking);
    pause(printTimeInterval);
    sim.simxSetIntegerSignal(clientID, 'Drop', 0, sim.simx_opmode_blocking);
    while(true)
        for i = 1 : 6
            [~,force(i)]=sim.simxGetJointForce(clientID, joint(i), sim.simx_opmode_blocking);
        end
        q_force_square(:,end+1) = force;
        [er, done] = sim.simxGetInt32Signal(clientID, 'Done', sim.simx_opmode_buffer);
        if(done == 1)
            sim.simxSetInt32Signal(clientID, 'Done', 0, sim.simx_opmode_blocking);
            break
        end
     end
     % Lấy vị trí thực của các khớp
    [~, realPosition] = sim.simxGetStringSignal(clientID, 'realPosition', sim.simx_opmode_buffer);
    realPosition = sim.simxUnpackFloats(realPosition);
    for i = 1:length(realPosition)/6
        q_square_real(1, end+1) = realPosition(6*(i-1)+1);
        q_square_real(2, end) = realPosition(6*(i-1)+2);
        q_square_real(3, end) = realPosition(6*(i-1)+3);
        q_square_real(4, end) = realPosition(6*(i-1)+4);
        q_square_real(5, end) = realPosition(6*(i-1)+5);
        q_square_real(6, end) = realPosition(6*(i-1)+6);
    end
    % Vẽ đồ thị so sánh giữa quỹ đạo thực và quỹ đạo mong muốn
    %% Đưa tay máy về vị trí ban đầu
    targetpoint = [1.18437 0.44429 -0.00688];
    orientations_target = [0 -pi -pi/2]; clf;
    [q, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, true, 'curve', "None");
    qq = q(:); qq(2:end+1) = qq;
    trajectory = sim.simxPackFloats(qq');
    sim.simxSetStringSignal(clientID, 'trajectory', trajectory, sim.simx_opmode_oneshot);
    pause(2);
    
    sim.simxSetIntegerSignal(clientID, 'LetMove', 1, sim.simx_opmode_blocking);
    pause(printTimeInterval);
    sim.simxSetIntegerSignal(clientID, 'LetMove', 0, sim.simx_opmode_blocking);
else
    disp('Failed connecting to remote API server');
end
sim.delete();% call the destructor!
disp('Program ended');