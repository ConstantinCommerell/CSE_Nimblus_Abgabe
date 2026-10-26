function [dx, dy] = odometry8DOFtoRelPose( ...
  delta, n, psiInDeg, psidotInDeg, deltaTinS)
%% INPUT
% delta in deg (steering angles)
% n in rounds per second
% psiInDeg in deg
% psidotInDeg in deg/s
% deltaTinS in seconds
%
%% OUTPUT
% x-velocity of geometrical center in world coordinates in m
% y-velocity of geometrical center in world coordinates in m

% predefinition of car params
tw = 0.55; % track_width = 0.55;   % Spurweite in m
wb = 0.86; % wheel_base = 0.86;    % Radstand in m
r = 0.096; % wheelRadius = 0.095;  % Radradius in m

%% From here everything in car coodinates
% psi and psidot in rad
psiInRad = psiInDeg * pi / 180;
psidotInRad = psidotInDeg * pi / 180;
delta = delta * pi / 180;

% definition of functions
v_Sx(1) = 2*pi*n(1) * r * cos(delta(1)) + psidotInRad * tw/2;
v_Sx(2) = 2*pi*n(2) * r * cos(delta(2)) - psidotInRad * tw/2;
v_Sx(3) = 2*pi*n(3) * r * cos(delta(3)) + psidotInRad * tw/2;
v_Sx(4) = 2*pi*n(4) * r * cos(delta(4)) - psidotInRad * tw/2;
v_Sy(1) = 2*pi*n(1) * r * sin(delta(1)) - psidotInRad * wb/2;
v_Sy(2) = 2*pi*n(2) * r * sin(delta(2)) - psidotInRad * wb/2;
v_Sy(3) = 2*pi*n(3) * r * sin(delta(3)) + psidotInRad * wb/2;
v_Sy(4) = 2*pi*n(4) * r * sin(delta(4)) + psidotInRad * wb/2;

%% From here everything in world coodinates
vSWorldSys = zeros(2,4);
rotMat = [cos(psiInRad) -sin(psiInRad); ...
          sin(psiInRad) cos(psiInRad)];
for i = 1:4
  vSWorldSys(:,i) = rotMat * [v_Sx(i); v_Sy(i)];
end

% vX = median(v_Sx);
% vY = median(v_Sy);
% 
% dx = deltaTinS * vX;
% dy = deltaTinS * vY;

vXWorld = median(vSWorldSys(1,:));
vYWorld = median(vSWorldSys(2,:));

dx = deltaTinS * vXWorld;
dy = deltaTinS * vYWorld;

end
