% MANIPULATOR TRAJECTORY GENERATION
% Generates Cartesian only (no rotation) trajectories
%
% Copyright 2019 The MathWorks, Inc.

%% Setup
clear, clc, close all

% Define waypoint information
addpath '..\Trajectory\utilities'
toolPositionHome = [-1.0024 -0.2653 0.38823]'; % start point of the pen
heart_waypoint = 0.5*[0 0 0; 0 0.15 0; 0.2 0 0; 0 -0.15 0; 0 0 0]';

F_waypoint = [0 0 0 ; 0 -0.05 0 ; 0.1 -0.05 0 ; 0.05 -0.05 0 ; 0.05 0 0]';
E_waypoint = [0 0.06 0 ; 0 0.01 0 ; 0.1 0.01 0 ; 0.1 0.06 0 ; 0.05 0.01 0 ; 0.05 0.06 0]';
T_waypoint = [0 0.07 0 ; 0 0.12 0 ; 0 0.095 0 ; 0.1 0.095 0]';

% Define IK
% ik = inverseKinematics('RigidBodyTree',gen3);
% ikWeights = [1 1 1 1 1 1];
% ikInitGuess = gen3.homeConfiguration;

% Set up plot
% plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
% show(gen3,jointAnglesHome','Frames','off','PreservePlot',false);
% xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
% hold on

% if plotMode == 1
%     hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
% end
% plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);

%% Generate trajectory
myTrajectory = 'FET';
switch myTrajectory
    case 'heart'
        % heart trajectory
        [waypoints, waypointTimes, trajTimes, waypointVels, waypointAccels, waypointAccelTimes] = createMyWaypointData(toolPositionHome, heart_waypoint, 'heart');
        [q_heart,~,~] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
                    'VelocityBoundaryCondition',waypointVels);
        qqq = q_heart;
    case 'FET'
        % F trajectory
        [waypoints, waypointTimes, trajTimes, waypointVels, waypointAccels, waypointAccelTimes] = createMyWaypointData(toolPositionHome, F_waypoint, 'FET');
        [qF,dqF,~] = trapveltraj(waypoints,numel(trajTimes), ...
                    'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
                    'EndTime',repmat(diff(waypointTimes),[3 1]));
        % Trajectory from F to E, using cubic trajectory
        cubic_1 = [F_waypoint(:,end), E_waypoint(:,1)];
        [waypoints, waypointTimes, trajTimes, waypointVels, waypointAccels, waypointAccelTimes] = createMyWaypointData(toolPositionHome, cubic_1, 'FET');
        [q_mid_1,dq_mid_1,~] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
                    'VelocityBoundaryCondition',waypointVels);
        % E trajectory
        [waypoints, waypointTimes, trajTimes, waypointVels, waypointAccels, waypointAccelTimes] = createMyWaypointData(toolPositionHome, E_waypoint(:,1:4), 'FET');
        [qE_1,dqE_1,~] = trapveltraj(waypoints,numel(trajTimes), ...
                    'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
                    'EndTime',repmat(diff(waypointTimes),[3 1]));
                
        [waypoints, waypointTimes, trajTimes, waypointVels, waypointAccels, waypointAccelTimes] = createMyWaypointData(toolPositionHome, E_waypoint(:,4:5), 'FET');
        [qE_mid,dqE_mid,~] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
                    'VelocityBoundaryCondition',waypointVels);
                
        [waypoints, waypointTimes, trajTimes, waypointVels, waypointAccels, waypointAccelTimes] = createMyWaypointData(toolPositionHome, E_waypoint(:,5:end), 'FET');
        [qE_2,dqE_2,~] = trapveltraj(waypoints,numel(trajTimes), ...
                    'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
                    'EndTime',repmat(diff(waypointTimes),[3 1]));
        qE = [qE_1, qE_mid, qE_2];
        dqE = [dqE_1, dqE_mid, dqE_2];
        % Trajectory from E to T, using cubic trajectory
        cubic_2 = [E_waypoint(:,end), T_waypoint(:,1)];
        [waypoints, waypointTimes, trajTimes, waypointVels, waypointAccels, waypointAccelTimes] = createMyWaypointData(toolPositionHome, cubic_2, 'FET');
        [q_mid_2,dq_mid_2,~] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
                    'VelocityBoundaryCondition',waypointVels);
        % T trajectory
        [waypoints, waypointTimes, trajTimes, waypointVels, waypointAccels, waypointAccelTimes] = createMyWaypointData(toolPositionHome, T_waypoint, 'FET');
        [q_T,dq_T,~] = trapveltraj(waypoints,numel(trajTimes), ...
                    'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
                    'EndTime',repmat(diff(waypointTimes),[3 1]));
        qqq=[qF, q_mid_1, qE, q_mid_2, q_T];
end
% Cartesian Motion only
% trajType = 'trap';
% switch trajType
%     case 'trap'
%         [q,qd,qdd] = trapveltraj(waypoints,numel(trajTimes), ...
%             'AccelTime',repmat(waypointAccelTimes,[3 1]), ... 
%             'EndTime',repmat(diff(waypointTimes),[3 1]));
%                             
%     case 'cubic'
%         [q,qd,qdd] = cubicpolytraj(waypoints,waypointTimes,trajTimes, ... 
%             'VelocityBoundaryCondition',waypointVels);
%         
%     case 'quintic'
%         [q,qd,qdd] = quinticpolytraj(waypoints,waypointTimes,trajTimes, ... 
%             'VelocityBoundaryCondition',waypointVels, ...
%             'AccelerationBoundaryCondition',waypointAccels);
%         
%     case 'bspline'
%         ctrlpoints = waypoints; % Can adapt this as needed
%         [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
%         
%     otherwise
%         error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
% end

% Show the full trajectory with the rigid body tree
% if plotMode == 1
%     set(hTraj,'xdata',q(1,:),'ydata',q(2,:),'zdata',q(3,:));
% elseif plotMode == 2
%     plotTransforms(q',repmat([1 0 0 0],[size(q,2) 1]),'FrameSize',0.05);
% end
% To visualize the trajectory, run the following line
% plotTrajectory(trajTimes,q,qd,qdd,'Names',["X","Y","Z"],'WaypointTimes',waypointTimes)

%% Trajectory following loop
% for idx = 1:numel(trajTimes) 
    % Solve IK
%     tgtPose = trvec2tform(q(:,idx)');
%     [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
%     ikInitGuess = config;

%     % Show the robot
%     show(gen3,config,'Frames','off','PreservePlot',false);
%     title(['Trajectory at t = ' num2str(trajTimes(idx))])
%     drawnow    
% end

% dqqq=[dqF, dq_mid_1, dqE, dq_mid_2, dq_T];

%% Draw trajectory
draw_trajectory;