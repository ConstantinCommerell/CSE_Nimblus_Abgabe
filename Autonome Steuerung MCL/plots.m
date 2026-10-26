%% create images from data
close all
clear all

%% 

load Testfahrt_Circle_1.mat

%% Plots

%% Calculated world poses

figure;
plot(carPoses(1:poseIdx-1,2), carPoses(1:poseIdx-1,3), ...
    'r-', 'LineWidth', 1.5);

axis equal;
grid on;

title('$\mathrm{Calculated\ World\ Pose}$', ...
    'Interpreter', 'latex');

xlabel('$x\,[\mathrm{m}]$', 'Interpreter', 'latex');
ylabel('$y\,[\mathrm{m}]$', 'Interpreter', 'latex');

legend('$\mathrm{Odometry}$', ...
    'Interpreter', 'latex', ...
    'Location', 'best');

%% SLAM results

if useLidar

    figure;

    plot(estimatedPoses(1:poseIdx-1,1), ...
         estimatedPoses(1:poseIdx-1,2), ...
         'r-', 'LineWidth', 1.5);

    axis equal;
    grid on;

    title('$\mathrm{SLAM\ Estimated\ Pose}$', ...
        'Interpreter', 'latex');

    xlabel('$x\,[\mathrm{m}]$', 'Interpreter', 'latex');
    ylabel('$y\,[\mathrm{m}]$', 'Interpreter', 'latex');

    legend('$\mathrm{SLAM}$', ...
        'Interpreter', 'latex', ...
        'Location', 'best');

end


%% Plot occupancy map

figure;

show(occMap);
hold on;

% SLAM / LiDAR estimated position
h1 = plot(estimatedPoses(:,1), estimatedPoses(:,2), ...
    'r-', 'LineWidth', 1.5);

% Odometry position
h2 = plot(carPoses(:,2), carPoses(:,3), ...
    'b-', 'LineWidth', 1.5);

% Reference trajectory
h3 = plot(trajectory(:,2), trajectory(:,3), ...
    'k--', 'LineWidth', 1.5);

title('$\mathrm{Occupancy\ Map\ and\ Vehicle\ Trajectories}$', ...
    'Interpreter', 'latex');

xlabel('$x\,[\mathrm{m}]$', ...
    'Interpreter', 'latex');

ylabel('$y\,[\mathrm{m}]$', ...
    'Interpreter', 'latex');

legend([h1, h2, h3], ...
    {'$\mathrm{LiDAR}$', ...
     '$\mathrm{Odometry}$', ...
     '$\mathrm{Reference\ Trajectory}$'}, ...
    'Interpreter', 'latex', ...
    'Location', 'best');

axis equal;
grid on;

hold off;