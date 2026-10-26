clear
close all
clc
format short
warning('off', 'all'); % Disable warnings (use with caution)
% Disable lidar warnings at every scan

% all poses are computed in world coordinate system except for
% the relative poses for the slam algorithm denoted as
% 'relPoseCS'


%% Control flags
% lidarSLAM from navigation toolbox:
%   https://de.mathworks.com/help/nav/ref/lidarslam.html
% CHANGE: lidar false for simple driving, true for mapping
useLidar = true;
% CHANGE: save 'dataTable', 'poses' and 'slamObj'?
saveData = false;

mode = "localization";%"mapping";
mapFile = "recordedMap.mat";

%% Xbox controller button and axis definitions
% Define button mappings for an Xbox controller
buttonA = 1;
buttonB = 2;
buttonX = 3;
buttonY = 4;
buttonLB = 5;
buttonRB = 6;
buttonCopy = 7; % Left small button
buttonSettings = 8; % Right small button
buttonLeftJoystick = 9;
buttonRightJoystick = 10;
buttonPower = 11;
buttonUpload = 12; % Middle small button

% Define joystick axis mappings
% joysticks horizontal: -1: left, +1: right
% joysticks vertical: -1: up, +1 down
axisLeftHozizontal = 1;  % Left joystick horizontal
axisLeftVertical = 2;    % Left joystick vertical
axisLTRT = 3;            % Triggers (-1: left, +1: right)
axisRightHorizontal = 4; % Right joystick horizontal
axisRightVertical = 5;   % Right joystick vertical

% Command states
NO = 0;
YES = 1;
TURN = 2;
POS = 3;
NEG = 4;
END = 5;

%% Arduino and controller setup
disp("Serialport Connection procedure started")
% CHANGE: set COM and baud rate
arduMega = serialport("COM4", 115200);
tic;
f = waitbar(0, 'Waiting to set serial connection to Arduino');
% CHANGE: if connection to ardu fails, increase value
initTime = 10;
while toc < initTime
  waitbar(toc/initTime, f, ...
    'Waiting to setup serial connection to Arduino');
end
delete(f) % Close the waitbar
disp("Serialport Connection procedure finished")

% Connect to Xbox controller
disp("Searching Controller Connection")
xBoxC = vrjoystick(1);
disp("Controller Connection established")

% Initialize control variables
steeringCommand = NO;
isInTurnOnSpot = false;
steeringAngle = 0;
rotRate = 0;
rotDir = 1;
currentSteeringAngle = 0;

%% Preallocate memory for data storage
defaultSize = 1000;
dataTable = zeros(defaultSize,4); % Store odometrie data
sentCount = 2; % Counter for data records
poses = zeros(defaultSize, 4); % Store poses
estimatedPoses = zeros(defaultSize, 3);
n=1;

%% Variables for Lidar and SLAM
if useLidar
  disp("Setup Lidar and parallel pool");
  tic; % Start timer for Lidar
  slamInterval = 5; % Time between SLAM updates in seconds

  % Start a parallel pool for SLAM processing
  f = waitbar(toc/slamInterval, ...
    'Waiting to start parallel pool');
  pool = gcp('nocreate');
  if isempty(pool)
    % Create a parallel pool if none exists
    pool = parpool('Processes', 1);
  end
  future = parallel.FevalFuture;


  % SLAM parameters
  MapResolution = 20;
  MaxLidarRange = 10;

  lastPose = [0 0 0];
  if mode == "mapping"

  slamObj = lidarSLAM(MapResolution, MaxLidarRange);
  slamObj.LoopClosureThreshold = 100;
  slamObj.LoopClosureSearchRadius = 8;
  slamObj.LoopClosureMaxAttempts = 1;
  slamObj.OptimizationInterval = 1;
  slamObj.MovementThreshold = [0 0];
  slamObj.TranslationSearchRange = ...
    [slamObj.MaxLidarRange/2, slamObj.MaxLidarRange/2];
  slamObj.RotationSearchRange = pi;

  elseif mode == "localization" 
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

    estimatePoses = zeros(defaultSize,4);

  end

  % Setup and start Velodyne Lidar
  lidar = velodynelidar('VLP32C');
  start(lidar);
  while toc < slamInterval % ensure one lidar scan at start
    waitbar(toc/slamInterval, f, 'Waiting to setup lidar');
  end
  delete(f)
  disp("Lidar and parallel pool ready");
end

%% Control loop
% Flush any previous input from Arduino
flush(arduMega, "input");

while true
  % % Lidar scan and SLAM processing
  % if useLidar && toc >= slamInterval
  %   % Send "no movement" command to Arduino
  %   str = sprintf("<%d, %0.f, %.2f>", NO, 0, 0);
  %   write(arduMega, str, "string");
  % 
  %   % Get latest point cloud from Lidar
  %   [frame, ~] = read(lidar, 'latest');
  % 
  %   % Fetch SLAM results if the previous task is done
  %   if strcmp(future.State, 'unavailable') == false
  %     % Retrieve updated SLAM object
  %     [slamObj, lastPose] = fetchOutputs(future);
  %   end
  % 
  %   % Start parallel processing for SLAM
  %   currentPose = poses(sentCount-1, [2,3,4]);
  %   future = parfeval(@performSlam, 2, frame, slamObj, ...
  %     currentPose, lastPose);
  %   tic; % Reset the SLAM interval timer
  % end

  % [frame, ~] = read(lidar, 'latest');
  % scan = pc2scan(frame);
  % 
  % odomPose = poses(sentCount-1, [2,3,4]);
  % 
  % [isUpdated, estimatedPose, covariance] = mcl(odomPose,scan);
  % 
  % if isUpdated
  %    estimatedPoses(n,:) = estimatedPose;
  %    n=n+1;
  % else
  %     estimatedPoses(n,:) = estimatedPose;
  %     n=n+1;
  % end


  % Read data from Arduino
  if arduMega.NumBytesAvailable > 0
    data = readline(arduMega);

    data = extractBetween(data, '<', '>');
    dataComponents = split(data, ',');

    timestamp = str2double(dataComponents{1});
    currentSteeringAngle = str2double(dataComponents{2});
    currentRotRate = str2double(dataComponents{3});
    currentPsiInDeg = ... % Normalize angle to [0, 359]
      mod(str2double(dataComponents{4}), 360);

    % Update data table with new values
    dataTable(sentCount,:) = ...
      [timestamp, currentSteeringAngle, ...
      currentRotRate, currentPsiInDeg];

    % Perform inverse kinematics and update poses
    lastTimestamp = dataTable(sentCount-1, 1);
    lastPsi = dataTable(sentCount-1, 4);
    [dx, dy] = odometryToRelPose(timestamp, ...
      currentSteeringAngle, currentRotRate, lastTimestamp, ...
      lastPsi, isInTurnOnSpot);
    poses(sentCount, 1:3) = poses(sentCount-1, 1:3) + ...
      [timestamp, dx, dy];
    poses(sentCount, 4) = currentPsiInDeg;
    

    [frame, ~] = read(lidar, 'latest');
  scan = pc2scan(frame);

  odomPose = poses(sentCount, [2,3,4]);
  odomPose(3) = deg2rad(odomPose(3));

  [isUpdated, estimatedPose, covariance] = mcl(odomPose,scan);
   
   estimatedPoses(sentCount,:) = estimatedPose;
    sentCount = sentCount + 1;
  end

  % Xbox controller input handling
  % order of statements matter, most importantly button 
  % Y/RB/LB before A and normal steering so
  % turning on spot does not get affected by other commands
  if button(xBoxC, buttonX)
    % Exit program with button X
    str = sprintf("<%d, 0, 0>", END); % Send end command
    write(arduMega, str, "string");
    pause(0.01);
    disp("Matlab Program was stopped via the button X")
    break;
  elseif button(xBoxC, buttonB) % Stop the vehicle
    % ATTENTION breaking is not implemented on VESCs
    % Stop of the vehicle is accomplished by blocking wheels
    % against each other i.e. enter "turn on the spot" mode
    if isInTurnOnSpot
      steeringCommand = NO;
      steerigAngle = 0;
      rotRate = 0;
    else
      steeringCommand = TURN;
      steerigAngle = 0;
      rotRate = 0;
      isInTurnOnSpot = true;
    end
  elseif button(xBoxC, buttonY)
    % Enable "turn on the spot" mode
    steeringCommand = TURN;
    steeringAngle = 0;
    rotRate = 0;
    isInTurnOnSpot = true;
  elseif button(xBoxC, buttonRB) && isInTurnOnSpot
    % Rotate clockwise
    steeringCommand = NEG;
    steerigAngle = 0;
    rotRate = 0;
  elseif button(xBoxC, buttonLB) && isInTurnOnSpot
    % Rotate counterclockwise
    steeringCommand = POS;
    steeringAngle = 0;
    rotRate = 0;
  elseif button(xBoxC, buttonA)
    % Reset steering to neutral
    steeringCommand = YES;
    steeringAngle = 0;
    rotRate = 0;
    isInTurnOnSpot = false;
  elseif isInTurnOnSpot
    % Stay idle if no input while in "turn on the spot" mode
    steeringCommand = NO;
    steeringAngle = 0;
    rotRate = 0;
  else
    % Handle joystick for steering and speed control
    steeringX = axis(xBoxC, axisLeftHozizontal);
    steeringY = axis(xBoxC, axisLeftVertical);

    if norm([steeringX steeringY]) < 1
      % Avoid steering if joystick is not in outer circle
      steeringCommand = NO;
      steeringAngle = 0;
    else
      steeringCommand = YES;
      % Convert joystick input to steering angle
      steeringAngle = atan2(steeringX, steeringY) * 180 / pi;
      if abs(steeringAngle) > 90 % upper half with joystick
        rotDir = 1; % drive forwards
        if steeringAngle < 0 % left upper quarter
          steeringAngle = steeringAngle + 180;
        else % right upper quarter
          steeringAngle = steeringAngle - 180;
        end
      else
        rotDir = -1;
      end
      if abs(steeringAngle - currentSteeringAngle) < 1.5
        steeringCommand = NO;
      end
    end

    % rotRate: mapped from 0.15 to 1.3
    %   slower is not possible, faster not desirable
    % multiply with the former sign to keep direction
    % if no steering in inner joystick
    speedInput = axis(xBoxC, axisLTRT);
    rotRate = (0.15 + abs(speedInput) * 1.15) * rotDir;
  end

  % Send control command to Arduino
  str = sprintf("<%d, %.0f, %.2f>", ...
    steeringCommand, steeringAngle, rotRate);
  write(arduMega, str, "string");
  pause(0.1);
end

% Close the serial connection to Arduino
disp('Closing serial connection to Arduino.');
delete(arduMega); % Close the serial connection
disp('Serial connection closed.');

% Stop Lidar and close SLAM processing
if useLidar
  disp('Stopping Lidar and SLAM processing.');
  stop(lidar); % Stop the Lidar sensor
  clear lidar; % Release resources associated with Lidar

  % Fetch the last SLAM output
  if strcmp(future.State, 'unavailable') == false
    [slamObj, lastPose] = fetchOutputs(future);
    disp('Final SLAM results fetched successfully.');
  end
end

%% Trajectory plotting
% remove preallocated memory
dataTable(sentCount:end, :) = [];
poses(sentCount:end, :) = [];
estimatedPoses(sentCount:end, :) = [];
% set first angle as zero position
poses(:,1) = poses(:,1) - poses(1,1);

% plot calculated world poses
figure;
axis("equal");
for i = 1:sentCount-1
  plot(poses(i,2),poses(i,3), 'ro');
  drawnow;
  hold on;
end
xlabel('x', Interpreter='latex');
ylabel('y', Interpreter='latex');
hold off;

%% plot slam results
if useLidar
  % poseGraph = slamObj.PoseGraph;
  % [scans, poses] = scansAndPoses(slamObj);
  % occMap = buildMap(scans, poses, ...
  %   slamObj.MapResolution, slamObj.MaxLidarRange);
  % 
  % % plot map with trajectory
  % figure;
  % show(slamObj);
  % title("Umgebungskarte mit Trajektorie");
  % 
  % % plot pose graphs with and without IDs
  % figure;
  % subplot(1,2,1);
  % show(poseGraph);
  % 
  % subplot(1,2,2);
  % show(poseGraph, IDs="off");
  % sgtitle("PoseGraph mit und ohne IDs");
%%
  figure;
axis("equal");
for i = 1:sentCount-1
  plot(estimatedPoses(i,1),estimatedPoses(i,2), 'ro');
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
  plot(estimatedPoses(:,1),estimatedPoses(:,2)); 
  plot(poses(:,2),poses(:,3));
  hold off;
end

% Save data to file
if saveData && sentCount > 2
  % Optionally save the data to a file
  fileTimestamp = datetime("now");
  fileTimestamp.Format = "yyyymmdd_HHMMSS";
  saveFileName = sprintf('data_run_%s.mat', ...
    string(fileTimestamp));
  save(saveFileName, 'dataTable', 'poses');
  if exist('slamObj', 'var')
    save(saveFileName, 'slamObj', '-append');
  end
  disp("Data saved to " + saveFileName);
end

disp('Program finished successfully.');

%% define function for parallel worker
% The performSlam function handles the SLAM processing in
% parallel. It takes the current Lidar scan, processes it, and
% updates the SLAM object. This function is executed
% asynchronously in a parallel worker.

function [slamObj, lastPose] = ...
  performSlam(frame, slamObj, currentPose, lastPose)

  % CHANGE: experiment with greater/smaller values

  % height in meters over optical center
  removeHeight = 0.05; % default = 0.05

  % just keep x% of it points for faster processing
  downsamplePercentage = 0.1; % default = 0.1

  % number of neighbors to compute mean distance
  % low -> sensitive to noise, high -> more computation
  denoiseNeighbors = 4; % default = 4
  % By default, the threshold is one standard deviation from
  % the mean of the average distance to neighbors of all
  % points. A point is considered to be an outlier if the
  % average distance to its k-nearest neighbors is above the
  % specified threshold.
  denoiseThreshold = 0.75; % default = 0.75

  %% Step 1:
  %   Remove points above 'removeHeight' m over optical center
  locations = frame.Location;
  z_coordinates = locations(:, :, 3);
  z_coordinates(z_coordinates > removeHeight) = nan;
  locations(:, :, 3) = z_coordinates;
  frame = pointCloud(locations);

  %% Step 2: Downsample and Denoise
  %   Downsample the point cloud to reduce the number of
  %   points for faster computation.
  frame = pcdownsample(frame, 'random', downsamplePercentage);

  % Remove noisy points using a denoising filter
  frame = pcdenoise(frame, ...
    'NumNeighbors', denoiseNeighbors, ...
    'Threshold', denoiseThreshold);

  %% Step 3: Segment out the ground
  [~, frame, ~] = segmentGroundSMRF(frame, ...
    MaxWindowRadius = 5, ElevationThreshold = 0.1, ...
    ElevationScale = 0.25, SlopeThreshold = 0.15);

  %% Step 4: Convert 3D point cloud to 2D scan
  %   apply simple parallel projection
  loc = frame.Location; % Get 3D locations of points
  cart = double(loc(:, 1:2)); % Project points onto xy-plane
  scan = lidarScan(cart); % Create a 2D Lidar scan

  %% Step 5: Update SLAM with new scan
  % relative pose is centered at last pose so first subtract
  % last pose from current pose and then rotate according to
  % last psi

  % Convert the current vehicle pose angle from deg to rad
  currentPose(3) = currentPose(3) * pi / 180;

  psi = lastPose(3); % Extract last pose angle (rad)
  rotMatrix = [cos(psi), sin(psi); -sin(psi), cos(psi)];

  relPoseCS = currentPose - lastPose;
  relPoseCS(1:2) = rotMatrix * relPoseCS(1:2)';

  % Add the new scan and pose to the SLAM object
  addScan(slamObj, scan, ...
    [relPoseCS(1), relPoseCS(2), relPoseCS(3)]);

  % Update the last pose to current pose for next iteration
  lastPose = currentPose;
end
