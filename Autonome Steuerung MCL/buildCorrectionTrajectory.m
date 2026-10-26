function correctionTrajectory = buildCorrectionTrajectory(actualPose, targetPose, tStep, vMax)

 % actualPose = [x y psi]
 % targetPose = [x y psi]
 % psi in rad or deg? Falls carPoses psi in deg ist: vorher umwandeln!

 tStepSec = tStep / 1000;

 x0 = actualPose(1);
 y0 = actualPose(2);
 psi0 = actualPose(3);

 xEnd = targetPose(1);
 yEnd = targetPose(2);
 psiEnd = targetPose(3);

 distance = norm([xEnd - x0, yEnd - y0]);

 if distance < 0.05
   correctionTrajectory = [];
   return;
 end

 T = distance / vMax;
 nSteps = max(2, ceil(T / tStepSec));

 t = (0:nSteps-1)' * tStepSec;
 s = linspace(0, 1, nSteps)';

 x = x0 + s * (xEnd - x0);
 y = y0 + s * (yEnd - y0);
 psi = psi0 + s * (psiEnd - psi0);

 xdot = gradient(x, tStepSec);
 ydot = gradient(y, tStepSec);
 psidot = gradient(psi, tStepSec);

 correctionTrajectory = [t, x, y, psi, xdot, ydot, psidot];
end