function x = buildEndPointRotating2( ...
    actualRot,endRot,tStep)
%% INPUT:
% T: period time for one single circle
% maxAngle: max angle to one side in degrees
% nofStages: number of full stages/cirlces (2*maxAngle)
%   car should rotate
% tStep: time interval between
%   two discrete trajectory points in ms
%
%% OUTPUT:
% x = [t, x, y, psi, xdot, ydot, psidot]
% in world coordinates and meters and radians and seconds
T= 2;
tStep = tStep/1000;
t = 0:tStep:T;
%angleVec = linspace(actualRot, endRot, length(t));
%maxAngleRad = maxAngleDeg/180*pi;
%psidotMax = 2 * pi/T * maxAngleRad;

deltaRot = atan2(sin(endRot - actualRot), cos(endRot - actualRot));
if(actualRot - endRot) < 0
    psidot = 0.6;
else
    psidot = -0.6;
end 

if abs(actualRot-endRot) > pi
    psidot = -psidot;
end

x = zeros(length(t),7); 

% x(1,1) = 0;
% x(1,2) = 0;
% x(1,3) = 0;
% x(1,4) = actualRot;
% x(1,5) = 0;
% x(1,6) = 0;
% x(1,7) = psidotMax;

for i = 1:length(t)
  s = (i-1)/(length(t)-1); 

  x(i,1) = t(i);
  x(i,2) = 0;
  x(i,3) = 0;
  x(i,4) = actualRot + s * deltaRot; %maxAngleRad*sin(angleVec(i));
  x(i,5) = 0;
  x(i,6) = 0;
  x(i,7) = psidot; %psidotMax * cos(angleVec(i));
end

end