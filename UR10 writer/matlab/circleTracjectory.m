function [Xd, dXd] = circleTracjectory(t, height, centerX, centerY, firstX, firstY)
Xd = zeros(1, 3);
dXd = zeros(1, 3);
radius = sqrt((firstX - centerX)^2 + (firstY - centerY)^2);
%% Quy dao hinh tron
Xd(1) = centerX + radius*cos(t);
Xd(2) = centerY + radius*sin(t);
Xd(3) = height;

dXd(1) = radius*sin(t);
dXd(2) = radius*cos(t);
dXd(3) = 0;
end