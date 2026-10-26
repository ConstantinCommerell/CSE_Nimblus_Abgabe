function  [deltaX, deltaY] = odometryToRelPose( ...
  time, delta, rotRate, t_before, psi_before, turn_on_spot)
% ODOMETRYTORELPOSE compute relative position change of car
% in world coordinate system by odometrie data from sensors
%
% Inputs:
%   time: timestamp by arduino in milliseconds
%   delta: steering angle relative to the car in degrees
%   rotRate: rounds per seconds given by VESCs
%   t_before: last timestamp for integration in milliseconds
%   psi_before: car orientation in world coordinates at last
%               timestamp in degrees
%   turn_on_spot: flag for "turn on the spot" mode
%
% Outputs:
%   deltaX: change in x coords in world sys in meters
%   deltaY: change in y coords in world sys in meters

  r = 0.10; % wheel radius in meters
  timeDiff = (time - t_before) / 1000; % timeDiff in seconds
  if isnan(rotRate) % error in VESCs, ignore values
    deltaX = 0;
    deltaY = 0;
  else
    if turn_on_spot
      % only turning on spot, no spacial difference
      deltaX = 0;
      deltaY = 0;
    else % only driving translatorily
      N = rotRate * timeDiff; % integrated rotRate phi
      alpha = psi_before + delta;
      deltaX = cos(alpha/180*pi)* 2 * pi * N * r;
      deltaY = sin(alpha/180*pi)* 2 * pi * N * r;
    end
  end
end
