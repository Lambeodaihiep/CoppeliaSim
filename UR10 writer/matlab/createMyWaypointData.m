% Create sample waypoint data for trajectory generation
% NOTE: Modify this script with your own rigid body tree and 
%       trajectory reference points
%       (We recommend saving a copy of this file)
%
% Copyright 2019 The MathWorks, Inc.
function [waypoints, waypointTimes, trajTimes, waypointVels, waypointAccels, waypointAccelTimes] = createMyWaypointData(toolPositionHome, character_waypoint, yourTrajectory) 
    %% Common parameters
    % Rigid Body Tree information
    % load gen3
    % load gen3positions
    % eeName = 'Gripper';
    % numJoints = numel(gen3.homeConfiguration);
    % ikInitGuess = gen3.homeConfiguration;

    % Maximum number of waypoints (for Simulink)
    maxWaypoints = 20;

    % Positions (X Y Z)
    % toolPositionHome = [-1.0024 -0.2653 0.38823];
    % waypoints = toolPositionHome' + ... 
    %             [0 0 0 ; 0 -0.05 0 ; 0.1 -0.05 0 ; 0.05 -0.05 0 ; 0.05 0 0]'; % character F
    % waypoints = toolPositionHome' + ... 
    %             [0 0 0 ; 0 -0.05 0 ; 0.1 -0.05 0 ; 0.05 -0.05 0 ; 0.05 0 0;
    %              0 0.06 0 ; 0 0.01 0 ; 0.1 0.01 0 ; 0.1 0.06 0 ; 0.05 0.01 0 ; 0.05 0.06 0;
    %              0 0.07 0 ; 0 0.12 0 ; 0 0.095 0 ; 0.1 0.095 0]'; %FET
    % waypoints = toolPositionHome' + ... 
    %             [0.05 0 0; 0 0.06 0]'; % from end of F to start of E
    waypoints = toolPositionHome + character_waypoint;      
    % Euler Angles (Z Y X) relative to the home orientation    
    % Don't care about this
    orientations = [0     pi/2    0;
                    0     pi/2    0; 
                    0     pi/2    0;
                    0     pi/2    0;
                    0     pi/2    0]';   

    % Array of waypoint times
    waypointTimes = 0:4:4*(length(character_waypoint(1,:))-1);

    % Trajectory sample time
    ts = 0.05;
    trajTimes = 0:ts:waypointTimes(end);

    %% Additional parameters

    % Boundary conditions (for polynomial trajectories)
    % Velocity (cubic and quintic)
    % waypointVels = 0.1 *[ 0  1  0;
    %                      -1  0  0;
    %                       0 -1  0;
    %                       1  0  0;
    %                       0  1  0]';
    switch yourTrajectory
        case 'FET'
            waypointVels = 0.1 *[ 0  0  0.5;
                                  0  0  0]';
        case 'heart'
            waypointVels = 0.1 *[ -0.8  0  0;
                             0.5  0  0;
                             0  0  0;
                             -0.5  0  0;
                             0.8  0  0]';
    end
    % Acceleration (quintic only)
    waypointAccels = zeros(size(waypointVels));

    % Acceleration times (trapezoidal only)
    waypointAccelTimes = diff(waypointTimes)/4;

end