clear
close all
clc
format short

% CHANGE: save 'carPoses', 'odometries' and 'trajectoryFig'?
saveData = false;

% CHANGE: lidar false for simple driving, true for mapping
useLidar = false;

% CHANGE: choose trajectory to follow
%   "circle", "rot" and "rotAndMove" are precomputed test
%   trajectories. If non of these, trajectoryName must be a
%   valid filename of a .mat object containing an occupancy
%   map object called "occMap". According parameters for each
%   trajectory can be set further below
%trajectoryName = "circle";
%trajectoryName = "rot";
trajectoryName = "rot2EndRot";
%trajectoryName = "rotAndMove";
%trajectoryName = "projektTrajectory";
%trajectoryName = "occMap_test.mat";
%trajectoryName = "occMap1.mat";
%trajectoryName = "zero";
mapFile = "recordedMap.mat";
defaultSize = 1000;
estimatePoses = zeros(defaultSize,4);
slamInterval = 5; % Time between SLAM updates in seconds

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


%% Variables for Lidar and SLAM
if useLidar
    
    
    MaxLidarRange = 10;
    lastPose = [0 0 0];


    load(mapFile,"occMap");
    mcl = monteCarloLocalization;
    mcl.UseLidarScan = true;
    mcl.ParticleLimits = [500 10000];
    mcl.UpdateThresholds = [0.05 0.05 0.05];

    sensorModel = likelihoodFieldSensorModel;
    sensorModel.Map = occMap;
    sensorModel.SensorLimits = [0.2 MaxLidarRange];
    sensorModel.SensorPose = [0 0 0];
    mcl.SensorModel = sensorModel;
    mcl.InitialPose = [0 0 0];
    mcl.InitialCovariance = diag([0.01 0.01 0.01]);

    


  % Setup and start Velodyne Lidar
  lidar = velodynelidar('VLP32C');
  start(lidar);
  while toc < slamInterval % ensure one lidar scan at start
    waitbar(toc/slamInterval, f, 'Waiting to setup lidar');
  end
  delete(f)
  disp("Lidar and parallel pool ready");
end

%% calculate trajectory
disp("calc trajectory for " + trajectoryName)

% CHANGE: adjust parameters for chosen trajectory
tStep = 100; % in ms ATTENTION must be same as on Arduino
switch trajectoryName
  case "circle"
    R = 0.8;    % radius in meters
    v0 = 0.4; % speed in m/s

    trajectory = buildCircleTrajectory(R, tStep, v0);
  case "rot"
    pendulumT = 3;    % period time of rotation in s
    maxAngleDeg = 45; % angle to one side in degree
    nofStages = 2;    % how many complete cycles

    trajectory = buildRotatingTrajectory( ...
      pendulumT, maxAngleDeg, nofStages, tStep);
  case "rot2EndRot"
    actualRot = deg2rad(336);
    endRot = deg2rad(0);
    maxAngleDeg = 45; % angle to one side in degree

    %trajectory = buildEndPointRotating(actualRot,endRot,maxAngleDeg,tStep);
    trajectory = buildEndPointRotating2(actualRot,endRot,tStep);

  case "projektTrajectory"
        v = 0.5; % constant velocity [m/s]
        s = 1; % first straight distance [m]
        R = 1; % radius for all circles [m]
        d2 = 1; % middle straight distance [m]
        d = 1; % final straight distance [m]
        a = 0.5; % acceleration in [m/s^2]
        trajectory = buildTrajectorywithAcceleration(tStep, v, s, R, d2, d,a);
  case "rotAndMove"
    pendulumT = 3;    % period time of rotation in s
    maxAngleDeg = 45; % angle to one side in degree
    nofStages = 2;    % how many complete cycles
    distance = 10;    % distance forward in meters

    trajectory = buildRotatingAndMovingTrajectory( ...
      pendulumT, maxAngleDeg, distance, nofStages, tStep);
case "zero"
    totalTime = 5; % End Time 
    dt = tStep / 1000;   % convert to seconds
    N  = ceil(totalTime / dt);
    
    % Initial pose (same as your trajectory generator)
    x0 = 0;
    y0 = 0;
    psi0 = pi/2;
    
    trajectory = zeros(N,7);
    
    for i = 1:N
        t = i*dt;
    
        trajectory(i,:) = [t, ...
                     x0, ...
                     y0, ...
                     psi0, ...
                     0, ...   % xdot
                     0, ...   % ydot
                     0];      % psidot
    end
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

%%
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
carPoses(1,4) = rad2deg(carPoses(1,4));
lastPsiInDeg = 0;
lastTimestamp = 0;
odometries = zeros(2,4,length(trajectory));
estimatedPoses = zeros(defaultSize, 3);
endRot_deg = rad2deg(endRot);
%% communication with Arduino
disp("start trajectory following")
i = 2;
flush(arduMega, "input");
delta_deg = 5;
delta_rad = deg2rad(delta_deg);
psiInRad = 0;
if endRot_deg < 0
    endRot_deg = 360 + endRot_deg;
end

while abs((carPoses(i-1,4)+rad2deg(actualRot)) - endRot_deg) > delta_deg && i < 100
  % send values when asked for
  % read serial data from arduino if available
  if arduMega.NumBytesAvailable > 0
    % send next trajectory value
    str = sprintf("<%.0f, %.0f, %.0f, %.0f, " + ...
                   "%.2f, %.2f, %.2f, %.2f>", ...
      sv(1,1,1), sv(2,1,1), sv(3,1,1), sv(4,1,1), ...
      sv(1,2,1), sv(2,2,1), sv(3,2,1), sv(4,2,1));
    write(arduMega, str, "string");

    data = readline(arduMega);    % read data from arduino
    data = extractBetween(data, '<', '>');
    dataComponents = split(data, ',');

    % save incoming data
    timestamp = str2double(dataComponents{1});
    psiInRad = deg2rad(str2double(dataComponents{2}));
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

if useLidar
  [frame, ~] = read(lidar, 'latest');
  scan = pc2scan(frame);

  odomPose = carPoses(i, [2,3,4]);
  odomPose(3) = deg2rad(odomPose(3));

  [isUpdated, estimatedPose, covariance] = mcl(odomPose,scan);
   
   estimatedPoses(i,:) = estimatedPose;
  end
    i = i + 1;
  end
end

%% End point correction
% write(arduMega, "<0,0,0,0, 0,0,0,0>", "string");
% 
% poseIdx = i-1;
% plannedEndPose = trajectory(end, 2:4);
% 
% actualEndPose = carPoses(poseIdx, 2:4);
% actualEndPose(3) = deg2rad(actualEndPose(3));
% 
% correctionTrajectory = buildEndPointCorrectionTrajectory( ...
%     actualEndPose, plannedEndPose, tStep, 0.5);
% 
% if ~isempty(correctionTrajectory)
% 
%     disp("Starting end point correction...");
% 
%     correctionControlValues = zeros(size(correctionTrajectory,1), 8);
% 
%     deltaLast = zeros(1,4);
% 
%     for k = 1:size(correctionTrajectory,1)
% 
%         res = trajectoryTo8DOF( ...
%             correctionTrajectory(k,4), ...
%             correctionTrajectory(k,5), ...
%             correctionTrajectory(k,6), ...
%             correctionTrajectory(k,7), ...
%             deltaLast);
% 
%         correctionControlValues(k,:) = [res(:,1)' res(:,2)'];
%         deltaLast = res(:,1)';
% 
%     end
% pause(1);
%  sv = correctionControlValues(1,:);
%  disp("send to initial steering angles")
% str = sprintf("<%.0f, %.0f, %.0f, %.0f, 0, 0, 0, 0>", ...
%   sv(1), sv(2), sv(3), sv(4));
% write(arduMega, str, "string");
% pause(1);
% 
%     for k = 1:size(correctionControlValues,1)
%     if arduMega.NumBytesAvailable > 0
%         sv = correctionControlValues(k,:);
% 
%         str = sprintf( ...
%             "<%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f>", ...
%             sv(1), sv(2), sv(3), sv(4), ...
%             sv(5), sv(6), sv(7), sv(8));
% 
%     write(arduMega, str, "string");
%     data = readline(arduMega);    % read data from arduino
%     data = extractBetween(data, '<', '>');
%     dataComponents = split(data, ',');
% 
%     % save incoming data
%     timestamp = str2double(dataComponents{1});
%     psiInDeg = mod(str2double(dataComponents{2}), 360);
%     currentSteeringAngles(1) = str2double(dataComponents{3});
%     currentSteeringAngles(2) = str2double(dataComponents{4});
%     currentSteeringAngles(3) = str2double(dataComponents{5});
%     currentSteeringAngles(4) = str2double(dataComponents{6});
%     currentRotRates(1) = str2double(dataComponents{7});
%     currentRotRates(2) = str2double(dataComponents{8});
%     currentRotRates(3) = str2double(dataComponents{9});
%     currentRotRates(4) = str2double(dataComponents{10});
% 
%     odometries(:, :, poseIdx) = ...
%       [currentSteeringAngles'; currentRotRates'];
% 
%     deltaTinS = (timestamp - lastTimestamp) / 1000;
%     deltaPsi = psiInDeg - lastPsiInDeg;
%     if deltaPsi > 180
%       deltaPsi = deltaPsi - 360;
%     elseif deltaPsi < -180
%       deltaPsi = deltaPsi + 360;
%     end
%     psidotInDeg = deltaPsi / deltaTinS;
%     lastTimestamp = timestamp;
%     lastPsiInDeg = psiInDeg;
% 
%     [dx, dy] = odometry8DOFtoRelPose( ...
%       currentSteeringAngles, currentRotRates, ...
%       psiInDeg, psidotInDeg, deltaTinS);
%     poseIdx = poseIdx+1;
%     carPoses(poseIdx,1) = timestamp;
%     carPoses(poseIdx,2) = carPoses(poseIdx-1,2) + dx;
%     carPoses(poseIdx,3) = carPoses(poseIdx-1,3) + dy;
%     carPoses(poseIdx,4) = psiInDeg;
%     pause(tStep / 1000);
%     end
%     end
% 
%     disp("End point correction finished.");
% 
% end


%% leave program clean
write(arduMega, "<0,0,0,0, 0,0,0,0>", "string");
pause(2);

delete(arduMega)
disp("Serial Port Object Arduino Mega cleared!")
% Stop Lidar and close SLAM processing
if useLidar
  disp('Stopping Lidar and SLAM processing.');
  stop(lidar); % Stop the Lidar sensor
  clear lidar; % Release resources associated with Lidar

end
%%  Plot
% remove preallocated memory
%dataTable(i:end, :) = [];
%CarPoses(i:end, :) = [];
estimatedPoses(i:end, :) = [];
% set first angle as zero position
% carPoses(:,1) = carPoses(:,1) - carPoses(1,1);

%% plot calculated world poses
% figure;
% axis("equal");
% for n = 1:i+k-1
%   plot(carPoses(n,2),carPoses(n,3), 'ro');
%   drawnow;
%   hold on;
% end
% xlabel('x', Interpreter='latex');
% ylabel('y', Interpreter='latex');
% hold off;

%% plot slam results
if useLidar
  
figure;
axis("equal");
for n = 1:i-1
  plot(estimatedPoses(n,1),estimatedPoses(n,2), 'ro');
  drawnow;
  hold on;
end
xlabel('x', Interpreter='latex');
ylabel('y', Interpreter='latex');
hold off;
 %% plot occupancy map
  figure;
  show(occMap);
  hold on;
  %plot(estimatedPoses(:,1),estimatedPoses(:,2));
  %legend("lidar");
  plot(carPoses(:,2)-R,carPoses(:,3)-R);
  plot(trajectory(:,2),trajectory(:,3))
  hold off;

end


%% Save data to file
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
