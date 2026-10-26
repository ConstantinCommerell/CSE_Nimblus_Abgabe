function [pathToFollow, fig] = pathPlanningHybridAStar( ...
  trajectoryName, tStep, vMax, x0, xEnd)
%% INPUT
% trajectoryName: filename of occupancy map
% tStep: time interval between two trajectory points
% vMax: maximum speed in m/s
% x0: start point in [m m rad]
% xEnd: end point in in [m m rad]
% x0 and xEnd relative to lower left corner of occMap, not
% to coordinates stored in occMap, as they get lost during
% conversion to binary occupancy map
%
%% OUTPUT
% pathToFollow = [t, x, y, psi, xdot, ydot, psidot]
% fig = the plot of the planned trajectory - can be used to
%   plot inverse trajectory from odometry

load(trajectoryName, 'occMap');
occupancy = getOccupancy(occMap);
occThreshold = 0.005;
occupancy(occupancy > occThreshold) = 1;
occupancy(occupancy <= occThreshold) = 0;
resolution = occMap.Resolution;
map = binaryOccupancyMap(occupancy, resolution);

% Create a validatorOccupancyMap state validator 
validator = validatorOccupancyMap(stateSpaceSE2, map=map);
validator.ValidationDistance = 0.05;

% Initialize the plannerHybridAStar object
%planner = plannerHybridAStar(validator, ...
 % MinTurningRadius=0.4, MotionPrimitiveLength=0.48);
planner = plannerHybridAStar(validator, ...
  MinTurningRadius=0.4, MotionPrimitiveLength=0.48);

% Set default random number for repeatability
rng("default");

% path planing
refPath = plan(planner, x0, xEnd, SearchMode="exhaustive");
path = refPath.States;

options = optimizePathOptions;

% trajectory parameters
% options.MinTurningRadius = 0.02; % meters
options.MaxVelocity = vMax; % m/s
% options.MaxAcceleration = 0.15; % m/s^2
options.ReferenceDeltaTime = tStep/1000; % seconds

options.MaxPathStates = 200;


% obstacle parameters
options.ObstacleSafetyMargin = 0.5; % meters, 0.5
% options.ObstacleInclusionDistance = 0.75; % meters
% options.ObstacleCutOffDistance = 2.5; %i meters

% solver parameters
% options.NumIteration = 5; 
% options.MaxSolverIteration = 15;

% weights
% options.WeightTime = 10;
% options.WeightSmoothness = 10;
% options.WeightMinTurningRadius = 1;
% options.WeightVelocity = 100;
% options.WeightObstacles = 1000;

% path optimization
[optimizedPath, kineticInfo] = ...
  optimizePath(path, map, options);

%%
timeDiff = tStep/1000;
timestamps = 0:timeDiff:kineticInfo.TimeStamps(end);
interpVelocities = interp1(kineticInfo.TimeStamps(2:end), ...
  kineticInfo.Velocity, timestamps(2:end));
interpVelocities(1) = 0;

pathToFollow = zeros(length(timestamps),7);
% pathToFollow = [t, x, y, psi, xdot, ydot, psidot]
pathToFollow(:,1) = timestamps;
pathToFollow(:,2) = interp1(kineticInfo.TimeStamps, ...
  optimizedPath(:,1), timestamps);
pathToFollow(:,3) = interp1(kineticInfo.TimeStamps, ...
  optimizedPath(:,2), timestamps);
pathToFollow(:,4) = interp1(kineticInfo.TimeStamps, ...
  optimizedPath(:,3), timestamps);
pathToFollow(2:end,5) = ...
  cos(pathToFollow(2:end,4)) .* interpVelocities';
pathToFollow(2:end,6) = ...
  sin(pathToFollow(2:end,4)) .* interpVelocities';
pathToFollow(2:end,7) = ...
  interp1(kineticInfo.TimeStamps(2:end), ...
  kineticInfo.AngularVelocity, timestamps(2:end));
pathToFollow(2,7) = 0;

fig = figure;
show(planner, Tree="off");
hold on
plot(pathToFollow(:,2), pathToFollow(:,3), ...
  ':b', 'LineWidth', 1.5, ...
  'DisplayName', 'Optimized Path');

legend(Location="southeast");
title("Planned Path and Optimized Path")
hold off

end
