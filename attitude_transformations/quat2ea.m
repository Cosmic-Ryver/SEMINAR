function ea = quat2ea(quat)
% quat2ea convert quaternion represented transformation to 3-2-1
%   transformation Euler angles
%
% Gus Buonviri, 1/28/18
% Mississippi State University
%
% INPUTS:
%
%   quat = [4 x 1] {numeric, column vector} quaternion represented
%       transformation, where the vector precedes the scalar.
%
% OUTPUTS:
%
%   ea = [3 x 1] {numeric, column vector} 3-2-1 transformation Euler angles
%       in radians. Order is roll-pitch-yaw.
%
%

ea = CTM2ea(quat2CTM(quat));

end