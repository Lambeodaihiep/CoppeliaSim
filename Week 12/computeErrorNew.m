function error = computeErrorNew(angle, setPoint)
error = angle - setPoint;
if error > 180
    error = error - 360 ;
elseif error < -180
    error = 360 + error;
end
end