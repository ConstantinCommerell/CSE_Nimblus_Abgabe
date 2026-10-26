function x = buildStraightTrajectory(tStep, distance)
%% INPUT:
% tStep: time interval between two discrete trajectory points [ms]
% distance: total distance to travel [m]
%
%% OUTPUT:
% x = [t, x, y, psi, xdot, ydot, psidot]
% world coordinates, meters, radians, seconds

tStep = tStep / 1000;   % convert to seconds

% Number of points (including start)
discretePoints = ceil(distance / tStep) + 1;

% Total time
T = (discretePoints - 1) * tStep;

% Constant velocity computed from distance and time
v = distance / T;

x = zeros(discretePoints, 7);

% Initial state
x(1,1) = 0;        % time
x(1,2) = 0;        % x-position
x(1,3) = 0;        % y-position
x(1,4) = pi/2;    % heading (phi)
x(1,5) = 0;        % xdot
x(1,6) = v;        % ydot
x(1,7) = 0;        % psidot

for i = 2:discretePoints
    x(i,1) = (i-1) * tStep;
    x(i,2) = 0;
    x(i,3) = v * x(i,1);
    x(i,4) = pi/2;

    x(i,5) = 0;
    x(i,6) = v;
    x(i,7) = 0;
end

end
