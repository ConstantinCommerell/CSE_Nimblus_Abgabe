function traj = buildTrajectorywithAcceleration(tStep, v, s, R, d2, d, a)
%% INPUT:
% tStep : time step [ms]
% v     : max velocity [m/s]
% s     : first straight distance [m]
% R     : radius for all circles [m]
% d2    : middle straight distance [m]
% d     : final straight distance [m]
% a     : acceleration [m/s^2]
%
%% OUTPUT:
% traj = [t, x, y, psi, xdot, ydot, psidot]

tStep = tStep / 1000;   % seconds

traj = [];
state = [0 0 0 pi/2];   % [t x y psi]

%% 1. Straight line
traj = [traj; straightSegment(state, s, v, a, tStep)];
state = traj(end,1:4);

%% 2. Quarter circle RIGHT
traj = [traj; circleSegment(state, R, -pi/2, v, a, tStep)];
state = traj(end,1:4);

%% 3. Quarter circle LEFT
traj = [traj; circleSegment(state, R,  pi/2, v, a, tStep)];
state = traj(end,1:4);

%% 4. Straight line
traj = [traj; straightSegment(state, d2, v, a, tStep)];
state = traj(end,1:4);

%% 5. Quarter circle LEFT
traj = [traj; circleSegment(state, R,  pi/2, v, a, tStep)];
state = traj(end,1:4);

%% 6. Quarter circle RIGHT
traj = [traj; circleSegment(state, R, -pi/2, v, a, tStep)];
state = traj(end,1:4);

%% 7. Straight line
traj = [traj; straightSegment(state, d, v, a, tStep)];

end


function seg = straightSegment(state, distance, vMax, a, dt)

t0 = state(1); x0 = state(2); y0 = state(3); psi = state(4);

% Time to reach max velocity
tRamp = vMax / a;
sRamp = 0.5 * a * tRamp^2;

if 2*sRamp > distance
    % Triangle profile (never reaches vMax)
    tRamp = sqrt(distance/a);
    T = 2*tRamp;
else
    % Trapezoidal profile
    sConst = distance - 2*sRamp;
    tConst = sConst / vMax;
    T = 2*tRamp + tConst;
end

N = ceil(T/dt);
seg = zeros(N,7);

x = x0; y = y0;
s = 0;

for i = 1:N
    t = i*dt;

    if t <= tRamp
        v = a*t;
    elseif t <= (T - tRamp)
        v = vMax;
    else
        v = a*(T - t);
    end

    ds = v*dt;
    s = s + ds;

    x = x + ds*cos(psi);
    y = y + ds*sin(psi);

    seg(i,:) = [t0 + t, x, y, psi, ...
                v*cos(psi), v*sin(psi), 0];
end
end


function seg = circleSegment(state, R, dPsi, vMax, a, dt)

t0 = state(1); x0 = state(2); y0 = state(3); psi0 = state(4);

arcLength = abs(R*dPsi);

tRamp = vMax / a;
sRamp = 0.5 * a * tRamp^2;

if 2*sRamp > arcLength
    tRamp = sqrt(arcLength/a);
    T = 2*tRamp;
else
    sConst = arcLength - 2*sRamp;
    tConst = sConst / vMax;
    T = 2*tRamp + tConst;
end

N = ceil(T/dt);
seg = zeros(N,7);

psi = psi0;
signTurn = sign(dPsi);

% Center of rotation
cx = x0 - R*sin(psi0)*signTurn;
cy = y0 + R*cos(psi0)*signTurn;

for i = 1:N
    t = i*dt;

    if t <= tRamp
        v = a*t;
    elseif t <= (T - tRamp)
        v = vMax;
    else
        v = a*(T - t);
    end

    omega = v / R;
    dpsi = signTurn * omega * dt;
    psi = psi + dpsi;

    x = cx + R*sin(psi)*signTurn;
    y = cy - R*cos(psi)*signTurn;

    seg(i,:) = [t0 + t, x, y, psi, ...
                v*cos(psi), v*sin(psi), signTurn*omega];
end
end
