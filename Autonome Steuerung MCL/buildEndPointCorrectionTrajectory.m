function correctionTrajectory = buildEndPointCorrectionTrajectory( ...
    actualPose, targetPose, tStep, vCorrection)
 
%% buildEndPointCorrectionTrajectory
% Builds a straight correction trajectory from actualPose to targetPose.
%
% INPUT:
% actualPose  = [x, y, psi] current vehicle pose
% targetPose  = [x, y, psi] desired end pose
% tStep       = time step in ms
% vCorrection = correction velocity in m/s
%
% IMPORTANT:
% psi must be in radians.
%
% OUTPUT:
% correctionTrajectory = [t, x, y, psi, xdot, ydot, psidot]
 
  tStepSec = tStep / 1000;
 
  x0 = actualPose(1);
  y0 = actualPose(2);
 
  xEnd = targetPose(1);
  yEnd = targetPose(2);
 
  dxTotal = xEnd - x0;
  dyTotal = yEnd - y0;
 
  distance = sqrt(dxTotal^2 + dyTotal^2);
 
  % If vehicle is already close enough, no correction is needed
  tolerance = 0.05; % 5 cm
  if distance < tolerance
    correctionTrajectory = [];
    return;
  end
 
  % Heading of the straight correction line
  psiCorrection = actualPose(3);
  travelDirection = atan2(dyTotal,dxTotal);

 
  % Number of discrete trajectory points
  T = distance / vCorrection;
  discretePoints = max(2, ceil(T / tStepSec) + 1);
 
  correctionTrajectory = zeros(discretePoints, 7);
 
  for i = 1:discretePoints
    s = (i-1) / (discretePoints-1);
 
    correctionTrajectory(i,1) = (i-1) * tStepSec;
    correctionTrajectory(i,2) = x0 + s * dxTotal;
    correctionTrajectory(i,3) = y0 + s * dyTotal;
    correctionTrajectory(i,4) = psiCorrection;
 
    correctionTrajectory(i,5) = vCorrection * cos(travelDirection);
    correctionTrajectory(i,6) = vCorrection * sin(travelDirection);
    correctionTrajectory(i,7) = 0;
  end
 
end