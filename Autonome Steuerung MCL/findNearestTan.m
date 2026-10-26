function delta = findNearestTan(xdot, ydot, deltaLast)
% find nearest tangens respective to periodicity of tan

deltaAkt = atan(ydot/(xdot+eps));

possibleDeltas = ((-10:10) * pi) + deltaAkt;

diff = 2*pi;
idx = 1;
for j = 1:length(possibleDeltas)
  diffNew = abs(possibleDeltas(j) - deltaLast);
  if diffNew < diff
    diff = diffNew;
    idx = j;
  end
end
delta = possibleDeltas(idx);

end
