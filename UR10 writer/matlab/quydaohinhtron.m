firstX = 0.2653;
firstY = -1.0024;
centerX = 0.3;
centerY = -0.95;
radius = sqrt((firstX - centerX)^2 + (firstY - centerY)^2);
%for t=0:pi/50:7*pi
t=0:pi/200:7*pi;
    xx=centerX+radius.*sin(t); %m
    yy=centerY+radius.*cos(t); %m
    zz=0.38823;
min = 10000;
for i=1:1:length(xx)
    if min > (abs(firstY-yy(i)) + abs(firstX-xx(i)))
        min = abs(firstY-yy(i)) + abs(firstX-xx(i))
        start = i;
    end
end
start;
%     plot3(x,y,z,'g.')
%     grid on
%     hold on
%end
%plot3(firstX,firstY,z,'b.')
