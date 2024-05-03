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
    goStraight(sim, clientID, left_Motor, right_Motor, 0.05, 1, 0.01);
    [~, euler] = sim.simxGetObjectOrientation(clientID, center, original, sim.simx_opmode_streaming);
    kp = 6;
    ki = 0.005;
    kd = 0.06;
    speed = 0.1; %m/s
    deltaT = 0.01;
    setPoint = 0;
    sum = 0;
    prev = 0;
    n = 0;
    arrayAngle = [];
    arraySetPoint =[];
    arrayError = [];
    arrayTimes = [];
    while true
        [~, euler] = sim.simxGetObjectOrientation(clientID, center, original, sim.simx_opmode_streaming);
        angle = euler(3)*180/pi;
        arrayAngle = [arrayAngle, angle];
        arraySetPoint = [arraySetPoint, setPoint];
        
        arrayTimes = [arrayTimes, n];
        if angle < 0
            angle = 360 + angle;
        end
        error = computeErrorNew(angle, setPoint);
        arrayError = [arrayError, abs(error)];
        [omega, sum, prev] = PIDrespone(error, kp, ki, kd, sum, deltaT, prev);
        [vLeft, vRight] = computeSpeed(speed, omega);
        sim.simxSetJointTargetVelocity( clientID,left_Motor,vLeft,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity( clientID,right_Motor,vRight,sim.simx_opmode_blocking);
        n = n + 1;
        if  mod(n,300) == 0
            figure(1);
            hold on
            grid on
            plot(arrayTimes, arrayAngle, "r", 'LineWidth', 1);
            plot(arrayTimes, arraySetPoint, "g", 'LineWidth' , 1);
            title("PID response")
            legend("setPoint", "Response");
            break;
        end
        
    end
else
    disp('Failed connecting to remote API server');
end
sim.delete();% call the destructor!
disp('Program ended');