close all  % close all opening figure windows
clear % Clear all variables in workspace
clc
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
dataFinal = [];
if (clientID>-1)
    disp('Connected');
    [~,Robot]=sim.simxGetObjectHandle(clientID,'Robot',sim.simx_opmode_blocking );
    [~,left_Motor]=sim.simxGetObjectHandle(clientID,'left_motor',sim.simx_opmode_blocking );
    [~,right_Motor]=sim.simxGetObjectHandle(clientID,'right_motor',sim.simx_opmode_blocking );
    [~,center]=sim.simxGetObjectHandle(clientID,'center',sim.simx_opmode_blocking);
    [~,original]=sim.simxGetObjectHandle(clientID,'original',sim.simx_opmode_blocking);
    kp = 6;
    ki = 0.01;
    kd = 0.06;
    speed = 0.1; %m/s
    deltaT = 0.01;
    sum = 0;
    prev = 0;
    n = 0;
    arrayAngle = [];
    arraySetPoint =[];
    arrayTimes = [];
    count = 1;
    delta = 5;
    t0 = 0;
    tn = 2;
    deltaTn = 0.1;
    angleList = computeAngleList(3, 5, -3, deltaTn, t0, tn);
    steps = deltaTn / deltaT;  
    setPoint = angleList(1);
    countSetpoint = 1;
    while true
        [~, euler] = sim.simxGetObjectOrientation(clientID, center, original, sim.simx_opmode_streaming);
        angle = euler(3)*180/pi;
        figure(2);
        hold on
        grid on
        [~, position] = sim.simxGetObjectPosition(clientID, center, original, sim.simx_opmode_streaming );
        plot(position(1), position(2), "r.");
        if angle < 0
            angle = 360 + angle;
        end
        arrayAngle = [arrayAngle, angle];
        arraySetPoint = [arraySetPoint, setPoint];
        arrayTimes = [arrayTimes, n];
        error = computeErrorNew(angle, setPoint);
        
        [omega, sum, prev] = PIDrespone(error, kp, ki, kd, sum, deltaT, prev);
        [vLeft, vRight] = computeSpeed(speed, omega);
        sim.simxSetJointTargetVelocity( clientID,left_Motor,vLeft,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity( clientID,right_Motor,vRight,sim.simx_opmode_blocking);
        [~, position] = sim.simxGetObjectPosition(clientID, center, original, sim.simx_opmode_streaming );
        n = n + 1;
        if  mod(n,steps) == 0
            figure(1);
            hold on
            grid on
            plot(arrayTimes, arrayAngle, "r", 'LineWidth', 1);
            plot(arrayTimes, arraySetPoint, "g", 'LineWidth' , 1);
            arrayAngle = [];
            arraySetPoint =[];
            arrayTimes = [];
            countSetpoint = countSetpoint + 1;
            setPoint = angleList(countSetpoint);
            sum = 0;
            prev = 0;
            if setPoint < 0
                setPoint = setPoint + 360;
            end
            if countSetpoint >= length(angleList)
                break;
            end
            count = count + 1;
        end
        
    end
else
    disp('Failed connecting to remote API server');
end
sim.delete();% call the destructor!
disp('Program ended');