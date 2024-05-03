offset = 0.19488;
rotationX = @(t) [1 0 0; 0 cosd(t) -sind(t); 0 sind(t) cosd(t)] ;
rotationY = @(t) [cosd(t) 0 sind(t); 0 1 0; -sind(t) 0 cosd(t)] ;
rotationZ = @(t) [cosd(t) -sind(t) 0; sind(t) cosd(t) 0; 0 0 1] ;
    jointAnglesHome = [0; 0; 0; 0; 0; 0];
    toolOrientationHome = [0 0 0];
    toolPositionHome = [1.18437 0.44429 -0.00688];
    targetpoint_offset = [0.9 0.65 0.1];
    targetpoint = [0.5 0.85-0.2 0.182-0.5];
    
    orientations_home = [0 -pi -pi/2];
    orientations_target_offset = [0 -pi -pi/2];
    orientations_target = [0 -pi -pi/2];

%     orientations_home = [0 0 0];
%     orientations_target_offset = [0 0 0];
%     orientations_target = [0 0 0];
    
    load UR10_;
    eeName = 'mid_point';
    
    maxWaypoints = 5;
    
    waypoints = [toolPositionHome' targetpoint_offset' targetpoint'];           
    orientations = [orientations_home' orientations_target_offset' orientations_target'];            
                
    % Array of waypoint times
    waypointTimes = 0:4:4*(length(waypoints(1,:))-1);
    
    % Trajectory sample time
    ts = 0.1;
    trajTimes = 0:ts:waypointTimes(end);
    
    %% Additional parameters
    
    % Boundary conditions (for polynomial trajectories)
    % Velocity (cubic and quintic)
    waypointVels = 0.1 *[ 0  1  0;
                         -1  0  0;
                          0 -1  0;
                          1  0  0;
                          0  1  0]';
    
    % Acceleration (quintic only)
    waypointAccels = zeros(size(waypointVels));
    
    % Acceleration times (trapezoidal only)
    waypointAccelTimes = diff(waypointTimes)/4;
    
    ik = inverseKinematics('RigidBodyTree', UR10_);
    ikWeights = [1 1 1 1 1 1];
    ikInitGuess = jointAnglesHome';
    
    % Set up plot
    plotMode = 1; % 0 = None, 1 = Trajectory, 2 = Coordinate Frames
    show(UR10_,UR10_.homeConfiguration,'Frames','off','PreservePlot',false);
    xlim([-2 2]), ylim([-2 2]), zlim([-1 2])
    hold on
    if plotMode == 1
        hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'b.-');
    end
    plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'ro','LineWidth',2);
    
%% Generate and follow trajectory
% Loop through segments one at a time
trajType = 'trap';
numWaypoints = size(waypoints,2);
for w = 1:numWaypoints-1
    % Get the initial and final rotations and times for the segment
    R0 = eul2quat(orientations(:,w)');
    Rf = eul2quat(orientations(:,w+1)');
    timeInterval = waypointTimes(w:w+1);
    trajTimes = timeInterval(1):ts:timeInterval(2);

    % Cartesian Motion only
    switch trajType
        case 'trap'
            [q,qd,qdd] = trapveltraj(waypoints(:,w:w+1),numel(trajTimes), ...
                'AccelTime',waypointAccelTimes(w), ... 
                'EndTime',diff(waypointTimes(w:w+1)));

        case 'cubic'
            [q,qd,qdd] = cubicpolytraj(waypoints(:,w:w+1),waypointTimes(w:w+1),trajTimes, ... 
                'VelocityBoundaryCondition',waypointVels(:,w:w+1));

        case 'quintic'
            [q,qd,qdd] = quinticpolytraj(waypoints(:,w:w+1),waypointTimes(w:w+1),trajTimes, ... 
                'VelocityBoundaryCondition',waypointVels(:,w:w+1), ...
                'AccelerationBoundaryCondition',waypointAccels(:,w:w+1));

        case 'bspline'
            ctrlpoints = waypoints(:,idx:idx+1); % Can adapt this as needed
            [q,qd,qdd] = bsplinepolytraj(ctrlpoints,timeInterval,trajTimes);

        otherwise
            error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
    end
        
    % Find the quaternions from trajectory generation
    [R, omega, alpha] = rottraj(R0, Rf, timeInterval, trajTimes);    
    
    % Plot trajectory
    if plotMode == 1
        set(hTraj,'xdata',q(1,:),'ydata',q(2,:),'zdata',q(3,:));
    elseif plotMode == 2
        plotTransforms(q',R','FrameSize',0.05)
    end
    numJoints = numel(UR10_.homeConfiguration);
    jointWaypoints = zeros(numJoints, length(trajTimes));
    % Trajectory following loop
    for idx = 1:numel(trajTimes) 
        % Solve IK
        tgtPose = trvec2tform(q(:,idx)') * quat2tform(R(:,idx)');
        [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
        ikInitGuess = config;
        jointWaypoints(:,idx) = config';

        % Show the robot
        show(UR10_,config,'Frames','off','PreservePlot',false);
        title(['Trajectory at t = ' num2str(trajTimes(idx))])
        drawnow    
    end
    
    
end





