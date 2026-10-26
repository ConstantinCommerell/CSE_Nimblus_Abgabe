function res = ...
    trajectoryTo8DOF(psi, xdot_s, ydot_s, psidot, deltaLast)
    
% input in world coordinates and output in vehicle coordinates
% psi_input in rad
%
% deltaLast: last 4 steering angles [VL VR HL HR] in degree
%
% output
% res = [delta_fl omega_fl;
%        delta_fr omega_fr;
%        delta_rl omega_rl;
%        delta_rr omega_rr]
% unit: [deg      rounds/s]

% ------------------------------------------------------------
% vectors that point into the direction in which the wheel is
% steered in vehicle coordinates
% deltaVecs = [deltaVLxV, deltaVRxV, deltaHLxV, deltaHRxV;
%              deltaVLyV, deltaVRyV, deltaHLyV, deltaHRyV]

% x = [xdot_fl xdot_fr xdot_rl xdot_rr;
%      ydot_fl ydot_fr yfot_rl ydot_rr]
% unit: [m/s  m/s]


% predefinition of car params
track_width = 0.55;   % Spurweite in m
wheel_base = 0.86;    % Radstand in m
wheelRadius = 0.095;  % Radradius in m

% build rotational matrix
rotMat_V_in_W = [cos(psi) -sin(psi) 0;
                 sin(psi)  cos(psi) 0;
                 0                0 1];
% vWorld = [vFl_W vFr_W vRl_W vRr_W]
% vVehicle = [vFl_V vFr_V vRl_V vRr_V]
vWorld_s(:,1) = [xdot_s; ydot_s; 0];
vVehicle_s(:,1) = rotMat_V_in_W' * vWorld_s(:,1);

% convert lastdeltas to rad
deltaLast = deltaLast * pi/180;

% 
x(1,1) = vVehicle_s(1) - track_width/2 * psidot;
x(2,1) = vVehicle_s(2) + wheel_base/2 * psidot;
res(1,1) = findNearestTan(x(1,1),x(2,1), deltaLast(1));
deltaVecs(1:2,1) = [cos(res(1,1)), sin(res(1,1))];

x(1,2) = vVehicle_s(1) + track_width/2 * psidot;
x(2,2) = vVehicle_s(2) + wheel_base/2 * psidot;
res(2,1) = findNearestTan(x(1,2),x(2,2), deltaLast(2));
deltaVecs(1:2,2) = [cos(res(2,1)), sin(res(2,1))];

x(1,3) = vVehicle_s(1) - track_width/2 * psidot;
x(2,3) = vVehicle_s(2) - wheel_base/2 * psidot;
res(3,1) = findNearestTan(x(1,3),x(2,3), deltaLast(3));
deltaVecs(1:2,3) = [cos(res(3,1)), sin(res(3,1))];

x(1,4) = vVehicle_s(1) + track_width/2 * psidot;
x(2,4) = vVehicle_s(2) - wheel_base/2 * psidot;
res(4,1) = findNearestTan(x(1,4),x(2,4), deltaLast(4));
deltaVecs(1:2,4) = [cos(res(4,1)), sin(res(4,1))];

% convert angles from rad to degree
res(:, 1) = res(:, 1) .* 180/pi;
res(:, 1) = mod(res(:,1), 360);        

% sign and magnitude of rotational rate
res(1,2) = sign(deltaVecs(:,1)'*[x(1,1); x(2,1)]) * ...
  sqrt(x(1,1)^2+x(2,1)^2) / (wheelRadius * 2 * pi);
res(2,2) = sign(deltaVecs(:,2)'*[x(1,2); x(2,2)]) * ...
  sqrt(x(1,2)^2+x(2,2)^2) / (wheelRadius * 2 * pi);
res(3,2) = sign(deltaVecs(:,3)'*[x(1,3); x(2,3)]) * ...
  sqrt(x(1,3)^2+x(2,3)^2) / (wheelRadius * 2 * pi);
res(4,2) = sign(deltaVecs(:,4)'*[x(1,4); x(2,4)]) * ...
  sqrt(x(1,4)^2+x(2,4)^2) / (wheelRadius * 2 * pi);

% for loop only for making all steering angles possible
% not necessary in unconstrained environment
for i = 1:4
  if res(i, 1) >= 90 && res(i, 1) <= 180
    res(i,1) = interp1([90,180],[-90 0], res(i,1));
    res(i,2) = res(i,2) * (-1);
  end
  if res(i, 1) >= 180 && res(i, 1) <= 270
    res(i,1) = interp1([180,270],[90 0], res(i,1));
    res(i,2) = res(i,2) * (-1);
  end
  if res(i, 1) >= 270 && res(i, 1) <= 360
    res(i,1) = interp1([270, 360],[-90 0], res(i,1));
  end
end

end
