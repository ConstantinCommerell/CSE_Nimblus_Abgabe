clear
close all
clc
format short

% CHANGE: save 'carPoses', 'odometries' and 'trajectoryFig'?
saveData = false;

% CHANGE: choose trajectory to follow
%   "circle", "rot" and "rotAndMove" are precomputed test
%   trajectories. If non of these, trajectoryName must be a
%   valid filename of a .mat object containing an occupancy
%   map object called "occMap". According parameters for each
%   trajectory can be set further below
trajectoryName = "circle";
%trajectoryName = "rot";
%trajectoryName = "rotAndMove";
%trajectoryName = "occMap_test.mat";
%trajectoryName = "occMap1.mat";

%% Arduino and controller setup
disp("Serialport Connection procedure started")
% CHANGE: set COM and baud rate
arduMega = serialport("COM4", 115200);
tic;
f = waitbar(0, "Waiting to set serial connection to Arduino");
% CHANGE: if connection to ardu fails, increase value
initTime = 10;
while toc < initTime
  waitbar(toc/initTime, f, ...
    "Waiting to setup serial connection to Arduino");
end
delete(f) % Close the waitbar
disp("Serialport Connection procedure finished")

%% calculate trajectory
disp("calc trajectory for " + trajectoryName)

% CHANGE: adjust parameters for chosen trajectory
tStep = 100; % in ms ATTENTION must be same as on Arduino
switch trajectoryName
  case "circle"
    R = 0.8;    % radius in meters
    v0 = 0.5; % speed in m/s

    trajectory = buildCircleTrajectory(R, tStep, v0);
  case "rot"
    pendulumT = 3;    % period time of rotation in s
    maxAngleDeg = 45; % angle to one side in degree
    nofStages = 2;    % how many complete cycles

    trajectory = buildRotatingTrajectory( ...
      pendulumT, maxAngleDeg, nofStages, tStep);
  case "rotAndMove"
    pendulumT = 3;    % period time of rotation in s
    maxAngleDeg = 45; % angle to one side in degree
    nofStages = 2;    % how many complete cycles
    distance = 10;    % distance forward in meters

    trajectory = buildRotatingAndMovingTrajectory( ...
      pendulumT, maxAngleDeg, distance, nofStages, tStep);
  otherwise
    % safeguard against non existing files
    if ~exist(trajectoryName, "file")
      error("filename " + trajectoryName + " does not exist!")
    end
    variableInfo = who('-file', trajectoryName);
    if ~ismember('occMap', variableInfo)
      error(trajectoryName + " does not contain an" + ...
        " occupancy  map called 'occMap'!")
    end

    vMax = 2;               % speed in m/s
    x0 = [6 13 0];          % in [m m rad]
    xEnd = [14 11 -pi/2]; % in [m m rad]

    [trajectory, trajectoryFig] = pathPlanningHybridAStar(trajectoryName, tStep, vMax, x0, xEnd);
end
disp("trajectory calculation finished")

sv = zeros(4, 2, length(trajectory));
deltaLast = zeros(4,1);

disp("convert to 8 values for car")
for i = 1:length(trajectory)
  sv(:, :, i) = trajectoryTo8DOF( ...
    trajectory(i,4), trajectory(i,5), trajectory(i,6), ...
    trajectory(i,7), deltaLast);
  deltaLast = sv(:, 1, i);
end

disp("send to initial steering angles")
str = sprintf("<%.0f, %.0f, %.0f, %.0f, 0, 0, 0, 0>", ...
  sv(1,1,1), sv(2,1,1), sv(3,1,1), sv(4,1,1));
write(arduMega, str, "string");
pause(5);

%% preallocation
currentSteeringAngles = zeros(4,1);
currentRotRates = zeros(4,1);
carPoses = zeros(length(trajectory), 4);
carPoses(1,:) = trajectory(1,1:4);
lastPsiInDeg = 0;
lastTimestamp = 0;
odometries = zeros(2,4,length(trajectory));

%% communication with Arduino
disp("start trajectory following")
i = 2;
flush(arduMega, "input");
while i <= length(trajectory)
  % send values when asked for
  % read serial data from arduino if available
  if arduMega.NumBytesAvailable > 0
    % send next trajectory value
    str = sprintf("<%.0f, %.0f, %.0f, %.0f, " + ...
                   "%.2f, %.2f, %.2f, %.2f>", ...
      sv(1,1,i), sv(2,1,i), sv(3,1,i), sv(4,1,i), ...
      sv(1,2,i), sv(2,2,i), sv(3,2,i), sv(4,2,i));
    write(arduMega, str, "string");

    data = readline(arduMega);    % read data from arduino
    data = extractBetween(data, '<', '>');
    dataComponents = split(data, ',');

    % save incoming data
    timestamp = str2double(dataComponents{1});
    psiInDeg = mod(str2double(dataComponents{2}), 360);
    currentSteeringAngles(1) = str2double(dataComponents{3});
    currentSteeringAngles(2) = str2double(dataComponents{4});
    currentSteeringAngles(3) = str2double(dataComponents{5});
    currentSteeringAngles(4) = str2double(dataComponents{6});
    currentRotRates(1) = str2double(dataComponents{7});
    currentRotRates(2) = str2double(dataComponents{8});
    currentRotRates(3) = str2double(dataComponents{9});
    currentRotRates(4) = str2double(dataComponents{10});
    odometries(:, :, i) = ...
      [currentSteeringAngles'; currentRotRates'];

    deltaTinS = (timestamp - lastTimestamp) / 1000;
    deltaPsi = psiInDeg - lastPsiInDeg;
    if deltaPsi > 180
      deltaPsi = deltaPsi - 360;
    elseif deltaPsi < -180
      deltaPsi = deltaPsi + 360;
    end
    psidotInDeg = deltaPsi / deltaTinS;
    lastTimestamp = timestamp;
    lastPsiInDeg = psiInDeg;

    [dx, dy] = odometry8DOFtoRelPose( ...
      currentSteeringAngles, currentRotRates, ...
      psiInDeg, psidotInDeg, deltaTinS);

    carPoses(i,1) = timestamp;
    carPoses(i,2) = carPoses(i-1,2) + dx;
    carPoses(i,3) = carPoses(i-1,3) + dy;
    carPoses(i,4) = psiInDeg;
    i = i + 1;
  end
end

%% leave program clean
write(arduMega, "<0,0,0,0, 0,0,0,0>", "string");
pause(2);

delete(arduMega)
disp("Serial Port Object Arduino Mega cleared!")

% Save data to file
if saveData
  % Optionally save the data to a file
  fileTimestamp = datetime("now");
  fileTimestamp.Format = "yyyymmdd_HHMMSS";
  saveFileName = sprintf('data_run_%s.mat', ...
    string(fileTimestamp));
  save(saveFileName, 'carPoses', 'odometries');
  if exist('trajectoryFig', 'var')
    save(saveFileName, 'trajectoryFig', '-append');
  end
  disp("Data saved to " + saveFileName);
end

disp("Program finished successfully.");
