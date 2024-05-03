function [q, jointAnglesHome, homepoint, orientations_home] = get_trajectory(jointAnglesHome, homepoint, targetpoint, orientations_home, orientations_target, includeOrientation, type, pick_or_drop)

    %jointAnglesHome = [0; 0; 0; 0; 0; 0];
    % toolOrientationHome = [0 90 -90];
    % toolPositionHome = [1.18437 0.24941 -0.00688];
    % targetpoint = [-0.6 0.695 0.025-0.5+offset];
    
    %orientations_home = [0 pi/2 -pi/2];
    %orientations_target_offset = [-pi/2 pi -2*pi];
    %orientations_target = [0 0 -pi];
    
%     UR10_ = loadrobot("universalUR10");
%     UR10_.DataFormat = 'row';
    load UR10_
    eeName = 'mid_point';
    
    maxWaypoints = 5;
    switch pick_or_drop
        case "pick"
            mid_point = targetpoint;
            mid_point(3) = mid_point(3) + 0.2;
            waypoints = [homepoint' mid_point' targetpoint'];           
            orientations = [orientations_home' orientations_target' orientations_target']; 
        case "drop"
            mid_point = targetpoint;
            mid_point(3) = mid_point(3) - 0.25;
            waypoints = [homepoint' mid_point' targetpoint'];           
            orientations = [orientations_home' orientations_target' orientations_target']; 
        otherwise
            waypoints = [homepoint' targetpoint'];           
            orientations = [orientations_home' orientations_target']; 
    end
                
    % Array of waypoint times
    waypointTimes = 0:2:2*(length(waypoints(1,:))-1);
    
    % Trajectory sample time
    ts = 0.05;
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
    
    if type == "curve"
        %% Solve IK for all waypoints
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
    %     
    %     Trajectory following loop
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
        end
    else
        if type == "line"
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
            
%                     Show the robot
                    show(UR10_,config,'Frames','off','PreservePlot',false);
                    title(['Trajectory at t = ' num2str(trajTimes(idx))])
                    drawnow    
                end
%                 clf;
                q = jointWaypoints;
            end
        end
    end

    %% Get parameter for the next trajectory
    jointAnglesHome = config';
    homepoint = targetpoint;
    orientations_home = orientations_target;

end





