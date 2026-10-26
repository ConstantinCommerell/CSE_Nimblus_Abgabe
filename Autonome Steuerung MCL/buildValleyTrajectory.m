function traj = buildValleyTrajectory(tStep, vStraight, vCurve, R, pauseTime)
%BUILDVALLEYTRAJECTORY Creates the complete valley-shaped trajectory.
%
% INPUT:
%   tStep      : time step between trajectory points [ms]
%   vStraight  : translational velocity on straight segments [m/s]
%   vCurve     : translational velocity in curves [m/s]
%   R          : radius of the four 90-degree curves [m]
%   pauseTime  : pause after every straight and every curve [s]
%
% OUTPUT:
%   traj = [t, x, y, psi, xdot, ydot, psidot]
%
% Sign convention:
%   positive dPsi  -> left turn
%   negative dPsi  -> right turn
%
% Example:
%   tStep      = 20;
%   vStraight  = 0.10;
%   vCurve     = 0.04;
%   R          = 0.60;
%   pauseTime  = 0.80;
%
%   traj = buildValleyTrajectory( ...
%       tStep, vStraight, vCurve, R, pauseTime);

   arguments
       tStep      (1,1) double {mustBePositive}
       vStraight  (1,1) double {mustBePositive}
       vCurve     (1,1) double {mustBePositive}
       R          (1,1) double {mustBePositive}
       pauseTime  (1,1) double {mustBeNonnegative}
   end

   dt = tStep / 1000;

   % State:
   % [time, x, y, psi]
   %
   % psi = pi:
   % Start on the right, pointing to the left.
   state = [0, 0, 0, 0];

   % Initial point: vehicle standing still.
   traj = stateToTrajectoryRow(state, 0, 0);

   sequence = {
       'straight',  1
       'turn',       pi/2
       'straight',  0.5
       'turn',      -pi/2
       'straight',  0.5
       'turn',      -pi/2
       'straight',  0.5
       'turn',       pi/2
       'straight',  0.5
   };

   for k = 1:size(sequence,1)

       segmentType = sequence{k,1};
       value       = sequence{k,2};

       switch segmentType

           case 'straight'
               seg = straightSegment( ...
                   state, value, vStraight, dt);

           case 'turn'
               seg = circleSegment( ...
                   state, R, value, vCurve, dt);

           otherwise
               error('Unknown segment type: %s', segmentType);
       end

       % Add movement segment.
       traj = [traj; seg]; %#ok<AGROW>

       % New state is the last pose of the segment.
       state = traj(end,1:4);

       % Add an explicit stop segment after every straight
       % and every curve.
       %
       % The control loop can therefore continue sending
       % zero velocities to the Arduino/VESC.
       if pauseTime > 0

           pauseSeg = pauseSegment( ...
               state, pauseTime, dt);

           traj = [traj; pauseSeg]; %#ok<AGROW>

           state = traj(end,1:4);
       end
   end
end


function row = stateToTrajectoryRow(state, v, omega)
%STATETOTRAJECTORYROW Converts a state into one trajectory row.

   psi = state(4);

   row = [
       state(1), ...
       state(2), ...
       state(3), ...
       psi, ...
       v*cos(psi), ...
       v*sin(psi), ...
       omega
   ];
end


function seg = straightSegment(state, distance, v, dt)
%STRAIGHTSEGMENT Creates one straight trajectory segment.

   t0   = state(1);
   x0   = state(2);
   y0   = state(3);
   psi0 = state(4);

   % Required duration for the straight segment.
   T = distance / v;

   sampleTimes = makeSampleTimes(T, dt);
   N = numel(sampleTimes);

   seg = zeros(N,7);

   for i = 1:N

       tau = sampleTimes(i);

       x = x0 + v*cos(psi0)*tau;
       y = y0 + v*sin(psi0)*tau;

       seg(i,:) = [
           t0 + tau, ...
           x, ...
           y, ...
           psi0, ...
           v*cos(psi0), ...
           v*sin(psi0), ...
           0
       ];
   end

   % Force the exact endpoint.
   seg(end,2) = x0 + distance*cos(psi0);
   seg(end,3) = y0 + distance*sin(psi0);
end


function seg = circleSegment(state, R, dPsi, v, dt)
%CIRCLESEGMENT Creates one circular trajectory segment.
%
% dPsi > 0: left turn
% dPsi < 0: right turn

   t0   = state(1);
   x0   = state(2);
   y0   = state(3);
   psi0 = state(4);

   turnSign = sign(dPsi);

   % Constant angular velocity in the curve.
   omega = turnSign * v / R;

   % Required duration for the angular change.
   T = abs(dPsi) / abs(omega);

   sampleTimes = makeSampleTimes(T, dt);
   N = numel(sampleTimes);

   seg = zeros(N,7);

   % Circle centre.
   cx = x0 - turnSign*R*sin(psi0);
   cy = y0 + turnSign*R*cos(psi0);

   for i = 1:N

       tau = sampleTimes(i);
       psi = psi0 + omega*tau;

       x = cx + turnSign*R*sin(psi);
       y = cy - turnSign*R*cos(psi);

       seg(i,:) = [
           t0 + tau, ...
           x, ...
           y, ...
           psi, ...
           v*cos(psi), ...
           v*sin(psi), ...
           omega
       ];
   end

   % Force the exact endpoint and final orientation.
   psiEnd = psi0 + dPsi;

   seg(end,2) = cx + turnSign*R*sin(psiEnd);
   seg(end,3) = cy - turnSign*R*cos(psiEnd);
   seg(end,4) = psiEnd;
   seg(end,5) = v*cos(psiEnd);
   seg(end,6) = v*sin(psiEnd);
   seg(end,7) = omega;
end


function seg = pauseSegment(state, pauseTime, dt)
%PAUSESEGMENT Keeps the vehicle at its current pose.
%
% During the pause:
%   xdot   = 0
%   ydot   = 0
%   psidot = 0
%
% Multiple trajectory samples are generated, so the controller can
% continue transmitting explicit stop commands.

   t0   = state(1);
   x0   = state(2);
   y0   = state(3);
   psi0 = state(4);

   sampleTimes = makeSampleTimes(pauseTime, dt);
   N = numel(sampleTimes);

   seg = zeros(N,7);

   for i = 1:N

       seg(i,:) = [
           t0 + sampleTimes(i), ...
           x0, ...
           y0, ...
           psi0, ...
           0, ...
           0, ...
           0
       ];
   end
end


function sampleTimes = makeSampleTimes(T, dt)
%MAKESAMPLETIMES Creates sample times without exceeding T.

   if T <= 0
       sampleTimes = [];
       return;
   end

   sampleTimes = dt:dt:T;

   % Add the exact endpoint if T is not a multiple of dt.
   if isempty(sampleTimes) || ...
           abs(sampleTimes(end)-T) > 10*eps(T)

       sampleTimes = [sampleTimes, T];
   end
end