function quat = ea2quat(ea)
% ea2quat convert 3-2-1 transformation Euler angles to a quaternion 
%   represented transformation
%
% Gus Buonviri, 1/28/18
% Mississippi State University
%
% INPUTS:
%
%   ea = [3 x 1] {numeric, column vector} 3-2-1 transformation Euler angles
%       in radians. Order is roll-pitch-yaw.
%
% OUTPUTS:
%
%   quat = [4 x 1] {numeric, column vector} quaternion represented
%       transformation, where the vector precedes the scalar.
%
%

quat = CTM2quat(ea2CTM(ea));

end