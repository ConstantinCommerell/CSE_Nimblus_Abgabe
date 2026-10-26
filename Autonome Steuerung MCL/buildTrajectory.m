function traj = buildTrajectory(tStep, v, s, R, d2, d)
%% INPUT:
% tStep : time step [ms]
% v     : constant velocity [m/s]
% s     : first straight distance [m]
% R     : radius for all circles [m]
% d2    : middle straight distance [m]
% d     : final straight distance [m]
%
%% OUTPUT:
% traj = [t, x, y, psi, xdot, ydot, psidot]

tStep = tStep / 1000;   % seconds

traj = [];
state = [0 0 0 pi/2];   % [t x y psi]

%% 1. Straight line (distance s)
traj = [traj; straightSegment(state, s, v, tStep)];
state = traj(end,1:4);

%% 2. Quarter circle RIGHT
traj = [traj; circleSegment(state, R, -pi/2, v, tStep)];
state = traj(end,1:4);

%% 3. Quarter circle LEFT
traj = [traj; circleSegment(state, R,  pi/2, v, tStep)];
state = traj(end,1:4);

%% 4. Straight line (distance d2)
traj = [traj; straightSegment(state, d2, v, tStep)];
state = traj(end,1:4);

%% 5. Quarter circle LEFT
traj = [traj; circleSegment(state, R,  pi/2, v, tStep)];
state = traj(end,1:4);

%% 6. Quarter circle RIGHT
traj = [traj; circleSegment(state, R, -pi/2, v, tStep)];
state = traj(end,1:4);

%% 7. Straight line (distance d)
traj = [traj; straightSegment(state, d, v, tStep)];

end



function seg = straightSegment(state, distance, v, dt)
t0 = state(1); x0 = state(2); y0 = state(3); psi = state(4);

T = distance / v;
N = ceil(T / dt);

seg = zeros(N,7);

for i = 1:N
    t = t0 + i*dt;
    x = x0 + v*cos(psi)*(i*dt);
    y = y0 + v*sin(psi)*(i*dt);

    seg(i,:) = [t x y psi ...
                v*cos(psi) v*sin(psi) 0];
end
end


function seg = circleSegment(state, R, dPsi, v, dt)
t0 = state(1); x0 = state(2); y0 = state(3); psi0 = state(4);

omega = v / R;
T = abs(dPsi) / omega;
N = ceil(T / dt);

seg = zeros(N,7);

% Center of rotation
cx = x0 - R*sin(psi0)*sign(dPsi);
cy = y0 + R*cos(psi0)*sign(dPsi);

for i = 1:N
    psi = psi0 + sign(dPsi)*omega*i*dt;

    x = cx + R*sin(psi)*sign(dPsi);
    y = cy - R*cos(psi)*sign(dPsi);

    seg(i,:) = [t0 + i*dt, x, y, psi, ...
                v*cos(psi), v*sin(psi), sign(dPsi)*omega];
end
end
