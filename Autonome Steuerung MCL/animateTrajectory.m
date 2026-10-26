function animateTrajectory(traj)
% animateTrajectory  Animates vehicle trajectory
%
% INPUT:
% traj = [t, x, y, psi, xdot, ydot, psidot]

t   = traj(:,1);
px  = traj(:,2);
py  = traj(:,3);
psi = traj(:,4);

dt = mean(diff(t));

% Vehicle dimensions (meters)
L = 1.0;   % length
W = 0.5;   % width

% Precompute trajectory bounds
margin = 1;
xmin = min(px) - margin;
xmax = max(px) + margin;
ymin = min(py) - margin;
ymax = max(py) + margin;

figure;
axis equal;
axis([xmin xmax ymin ymax]);
grid on;
hold on;

xlabel('x [m]');
ylabel('y [m]');
title('Vehicle Trajectory Animation');

% Plot full path (background)
plot(px, py, '--k', 'LineWidth', 1);

% Vehicle body (rectangle)
carPatch = patch('XData', [], 'YData', [], ...
                 'FaceColor', [0.2 0.6 0.8]);

% Heading arrow
headingArrow = quiver(0,0,0,0, ...
    'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);

for k = 1:length(t)
    % Rotation matrix
    R = [cos(psi(k)) -sin(psi(k));
         sin(psi(k))  cos(psi(k))];

    % Rectangle corners (vehicle frame)
    corners = [ L/2  L/2 -L/2 -L/2;
                W/2 -W/2 -W/2  W/2 ];

    % Rotate and translate
    worldCorners = R * corners + [px(k); py(k)];

    % Update car body
    set(carPatch, ...
        'XData', worldCorners(1,:), ...
        'YData', worldCorners(2,:));

    % Update heading arrow
    set(headingArrow, ...
        'XData', px(k), ...
        'YData', py(k), ...
        'UData', cos(psi(k))*L, ...
        'VData', sin(psi(k))*L);

    drawnow;
    pause(dt);
end

end