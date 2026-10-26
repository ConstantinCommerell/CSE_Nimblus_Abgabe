function x = buildCircleTrajectory(R, tStep, v0)
%% INPUT:
% R: Radius in m
% tStep: time interval between two
%   discrete trajectory points in ms
% v0: initial/continous velocity in m/s
%
%% OUTPUT:
% x = [t, x, y, psi, xdot, ydot, psidot]
% in world coordinates and meters and radians and seconds

tStep = tStep / 1000; % for calculation in seconds
T = 2*pi*R/v0;
discretePoints = ceil(T/tStep);

x = zeros(discretePoints, 7);

x(1,1) = 0;
x(1,2) = 0;
x(1,3) = 0;
x(1,4) = pi/2;
x(1,5) = 0;
x(1,6) = v0;
x(1,7) = v0/R;

for i = 2:discretePoints+1
  x(i,1) = (i-1) * tStep;
  x(i,2) = R*cos((i-1)*2*pi/discretePoints)-R;
  x(i,3) = R*sin((i-1)*2*pi/discretePoints);
  x(i,4) = pi/2+(i-1)*2*pi/discretePoints;
  x(i,5) = (x(i,2) - x(i-1,2)) / tStep;
  x(i,6) = (x(i,3) - x(i-1,3)) / tStep;
  x(i,7) = (x(i,4) - x(i-1,4)) / tStep;
end

end
