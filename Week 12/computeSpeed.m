function [vLeft, vRight] = computeSpeed(speed, omega)
r = 0.085/2;
b = 0.2084;
matrix = [1/r, b/(2*r); 1/r, -b/(2*r)];
vector = [speed; omega];
result = matrix * vector;
vLeft = result(1);
vRight = result(2);
end