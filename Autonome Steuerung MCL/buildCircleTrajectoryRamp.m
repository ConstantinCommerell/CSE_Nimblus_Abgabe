function x = buildCircleTrajectoryRamp(R, tStep, v0)
%% OUTPUT:
% x = [t, x, y, psi, xdot, ydot, psidot]

tStep = tStep/1000;   % original time step (only used for initial sampling)
T_geo = 2*pi*R;       % geometric length of the circle (arc length)

% Number of points
discretePoints = ceil(T_geo/(v0*tStep));

% Preallocate
x = zeros(discretePoints, 7);

%% ----- 1) Geometry stays EXACTLY the same -----
theta = linspace(0, 2*pi, discretePoints);
x(:,2) = R*cos(theta) - R;
x(:,3) = R*sin(theta);
x(:,4) = pi/2 + theta;

%% ----- 2) Build linear ramp speed profile -----
N = discretePoints;
ramp = zeros(N,1);

for i = 1:N
    if i <= N/3
        ramp(i) = (i-1)/(N/3);         % 0 → 1
    elseif i <= 2*N/3
        ramp(i) = 1;                  % constant
    else
        ramp(i) = 1 - (i - 2*N/3)/(N/3);   % 1 → 0
    end
end

%% ----- 3) Normalize ramp so that total distance is correct -----
meanRamp = mean(ramp);
v_scaled = v0 / meanRamp;     % ensures correct final pose

speedProfile = v_scaled * ramp;   % final linear ramp speed

%% ----- 4) Compute time stamps from speed profile -----
% Arc length per point:
ds = T_geo/(N-1);

t = zeros(N,1);
for i = 2:N
    t(i) = t(i-1) + ds / speedProfile(i);
end

x(:,1) = t;

%% ----- 5) Compute velocities from position derivatives -----
for i = 2:N
    dt = t(i)-t(i-1);

    x(i,5) = (x(i,2)-x(i-1,2)) / dt;         % xdot
    x(i,6) = (x(i,3)-x(i-1,3)) / dt;         % ydot
    x(i,7) = (x(i,4)-x(i-1,4)) / dt;         % psidot
end

end