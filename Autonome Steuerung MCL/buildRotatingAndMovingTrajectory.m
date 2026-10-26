function x = buildRotatingAndMovingTrajectory( ...
  T, maxAngleDeg, distance, nofStages, tStep)
%% INPUT:
% T: period time for one single circle
% maxAngle: max angle to one side in degrees
% distance: traveled distance in y direction in meters
% nofStages: number of full stages/cirlces (2*maxAngle)
%   car should rotate
% tStep: time interval between two discrete
%   trajectory points in ms
%
%% OUTPUT:
% x = [t, x, y, psi, xdot, ydot, psidot]
% in world coordinates and meters and radians and seconds

tStep = tStep/1000;
t = 0:tStep:T*nofStages;
angleVec = linspace(0, 2*pi*nofStages, length(t));
maxAngleRad = maxAngleDeg/180*pi;
psidotMax = 2 * pi/T * maxAngleRad;
distanceT = distance / length(t);
v = distance / t(end);

x(1,1) = 0;
x(1,2) = 0;
x(1,3) = 0;
x(1,4) = pi/2;
x(1,5) = 0;
x(1,6) = v;
x(1,7) = psidotMax;

for i = 2:length(t)
  x(i,1) = t(i);
  x(i,2) = 0;
  x(i,3) = x(i-1,3) + distanceT;
  x(i,4) = pi/2 + maxAngleRad*sin(angleVec(i));
  x(i,5) = 0;
  x(i,6) = v;
  x(i,7) = psidotMax * cos(angleVec(i));
end

end
