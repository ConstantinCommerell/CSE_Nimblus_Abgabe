% Map parameters
mapSize = 10;
resolution = 10;  % cells per meter

occMap = binaryOccupancyMap(mapSize, mapSize, resolution);
trajectoryName = "occMap.mat";
% Rectangular obstacles: [x y width height]
obstacles = [
    2 2 2 1;
    6 3 1.5 3;
    3 7 4 1
];

for i = 1:size(obstacles,1)
    obs = obstacles(i,:);
    
    % Generate points inside rectangle
    xVals = obs(1) : 1/resolution : obs(1) + obs(3);
    yVals = obs(2) : 1/resolution : obs(2) + obs(4);
    
    [X, Y] = meshgrid(xVals, yVals);
    pts = [X(:), Y(:)];
    
    setOccupancy(occMap, pts, 1);
end


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
    xEnd = [12.5 10 -pi/2]; % in [m m rad]


% Show map
figure;
show(occMap);
title('10x10 m Occupancy Map');
grid on;


