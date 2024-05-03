offset = 0.19488;
rotationX = @(t) [1 0 0; 0 cosd(t) -sind(t); 0 sind(t) cosd(t)] ;
rotationY = @(t) [cosd(t) 0 sind(t); 0 1 0; -sind(t) 0 cosd(t)] ;
rotationZ = @(t) [cosd(t) -sind(t) 0; sind(t) cosd(t) 0; 0 0 1] ;
    jointAnglesHome = [0; 0; 0; 0; 0; 0];
    toolOrientationHome = [0 90 -90];
    toolPositionHome = [1.18437 0.44429 -0.00688];
    targetpoint = [0.5 0.85-0.2 0.182-0.5];
    
    orientations_home = [0 pi/2 -pi/2];
    orientations_target_offset = [-pi/2 pi -2*pi];
    orientations_target = [0 -pi -pi/2];
    
    load UR10_;
    eeName = 'mid_point';
    
    maxWaypoints = 5;
    
    waypoints = [toolPositionHome' targetpoint'];           
    orientations = [orientations_home' orientations_target'];            
                
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
    
    %% Solve IK for all waypoints
    includeOrientation = true; % Set this to use zero vs. nonzero orientations
    
    numWaypoints = size(waypoints,2);
    numJoints = numel(UR10_.homeConfiguration);
    jointWaypoints = zeros(numJoints, numWaypoints);
    
    for idx = 1:numWaypoints
        if includeOrientation
            tgtPose = trvec2tform(waypoints(:,idx)') * eul2tform(orientations(:,idx)');
        else
            tgtPose = trvec2tform(waypoints(:,idx)');
        end
        [config,info] = ik(eeName,tgtPose,ikWeights,ikInitGuess);
        jointWaypoints(:,idx) = config';
    end
    
    %% Generate trajectory on joint space
    trajType = 'trap';
    switch trajType
        case 'trap'
            [q,qd,qdd] = trapveltraj(jointWaypoints,numel(trajTimes), ...
                'AccelTime',repmat(waypointAccelTimes,[numJoints 1]), ... 
                'EndTime',repmat(diff(waypointTimes),[numJoints 1]));
                                
        case 'cubic'
            [q,qd,qdd] = cubicpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
                'VelocityBoundaryCondition',zeros(numJoints,numWaypoints));
            
        case 'quintic'
            [q,qd,qdd] = quinticpolytraj(jointWaypoints,waypointTimes,trajTimes, ... 
                'VelocityBoundaryCondition',zeros(numJoints,numWaypoints), ...
                'AccelerationBoundaryCondition',zeros(numJoints,numWaypoints));
            
        case 'bspline'
            ctrlpoints = jointWaypoints; % Can adapt this as needed
            [q,qd,qdd] = bsplinepolytraj(ctrlpoints,waypointTimes([1 end]),trajTimes);
            
        otherwise
            error('Invalid trajectory type! Use ''trap'', ''cubic'', ''quintic'', or ''bspline''');
    end
    
    % To visualize the trajectory, run the following line
%     plotTrajectory(trajTimes,q,qd,qdd,'Names',"Joint " + string(1:numJoints),'WaypointTimes',waypointTimes)
    
    %% Trajectory following loop
    for idx = 1:numel(trajTimes)  
    
        config = q(:,idx)';
        
        % Find Cartesian points for visualization
        eeTform = getTransform(UR10_,config,eeName);
        if plotMode == 1
            eePos = tform2trvec(eeTform);
            set(hTraj,'xdata',[hTraj.XData eePos(1)], ...
                      'ydata',[hTraj.YData eePos(2)], ...
                      'zdata',[hTraj.ZData eePos(3)]);
        elseif plotMode == 2
            plotTransforms(tform2trvec(eeTform),tform2quat(eeTform),'FrameSize',0.05);
        end
    
        % Show the robot
        show(UR10_,config,'Frames','off','PreservePlot',false);
        title(['Trajectory at t = ' num2str(trajTimes(idx))])
        drawnow    
        axis([-1 1.5 -1 1.5 -1 1])
    end
%% draw trajectory
pause(2);
pos = ["northwest" "north" "northeast" "southwest" "south" "southeast"];
for i = 1:6
    figure("Name","Đồ thị vị trí và vận tốc khớp thứ " + string(i),"Position",[0 0 450 280]);
    movegui(pos(i));
    subplot(2,1,1);
    plot(trajTimes,q(i,:),"LineWidth",1.5);
    xlabel('time(s)'); ylabel('rad');
    subplot(2,1,2);
    plot(trajTimes,qd(i,:),"LineWidth",1.5);
%     legend('Quỹ đạo mong muốn','Quỹ đạo thực tế');
%     title('Đồ thị vị trí khớp thứ ' + string(i) + ' khi gắp vật hình tròn')
    xlabel('time(s)'); ylabel('rad/s');
end





