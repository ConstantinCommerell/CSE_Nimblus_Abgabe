function traj = buildEndPointRotating( ...
    actualRot,endRot, maxAngleDeg,tStep)

% BUILDENDPOINTROTATING
%
% Inputs:
%   actualRot   [rad]
%   endRot      [rad]
%   maxAngleDeg [deg/s]
%   tStep       [ms]
%
% Output:
%   traj = [t, x, y, psi, xdot, ydot, psidot]

    %% Parameters
    dt = tStep/1000;                % ms -> s
    omegaMax = deg2rad(maxAngleDeg);

    % Ramp time (adjust as desired)
    tRamp = 0.5;                    % [s]

    % Angular acceleration
    alpha = omegaMax/tRamp;         % [rad/s²]

    %% Required rotation
    dPsi = wrapToPi(endRot-actualRot);
    dir = sign(dPsi);

    if abs(dPsi) < 1e-9
        traj = [0 0 0 actualRot 0 0 0];
        return;
    end

    theta = abs(dPsi);

    %% Determine trapezoidal or triangular profile
    thetaRamp = 0.5*alpha*tRamp^2;

    if theta <= 2*thetaRamp
        % TRIANGULAR profile
        tRamp = sqrt(theta/alpha);
        omegaPeak = alpha*tRamp;

        tTotal = 2*tRamp;

        t = (0:dt:tTotal)';
        if t(end) < tTotal
            t = [t; tTotal];
        end

        omega = zeros(size(t));
        psiRel = zeros(size(t));

        for k = 1:length(t)
            tk = t(k);

            if tk <= tRamp
                omega(k) = alpha*tk;
                psiRel(k) = 0.5*alpha*tk^2;
            else
                td = tk-tRamp;
                omega(k) = omegaPeak-alpha*td;
                psiRel(k) = theta/2 + ...
                    omegaPeak*td - 0.5*alpha*td^2;
            end
        end

    else
        % TRAPEZOIDAL profile
        tConst = (theta-2*thetaRamp)/omegaMax;
        tTotal = 2*tRamp+tConst;

        t = (0:dt:tTotal)';
        if t(end) < tTotal
            t = [t; tTotal];
        end

        omega = zeros(size(t));
        psiRel = zeros(size(t));

        for k = 1:length(t)
            tk = t(k);

            if tk <= tRamp
                % acceleration
                omega(k) = alpha*tk;
                psiRel(k) = 0.5*alpha*tk^2;

            elseif tk <= tRamp+tConst
                % constant velocity
                tc = tk-tRamp;
                omega(k) = omegaMax;
                psiRel(k) = thetaRamp + omegaMax*tc;

            else
                % deceleration
                td = tk-(tRamp+tConst);
                omega(k) = omegaMax-alpha*td;

                psiRel(k) = thetaRamp + ...
                            omegaMax*tConst + ...
                            omegaMax*td - ...
                            0.5*alpha*td^2;
            end
        end
    end

    %% Apply direction
    psi = actualRot + dir*psiRel;
    psi(end) = endRot;

    psidot = dir*omega;
    psidot(end) = 0;

    %% Fixed position
    x = zeros(size(t));
    y = zeros(size(t));
    xdot = zeros(size(t));
    ydot = zeros(size(t));

    traj = [t x y psi xdot ydot psidot];
end