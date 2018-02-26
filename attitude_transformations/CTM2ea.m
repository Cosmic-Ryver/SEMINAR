function ea = CTM2ea(CTM)
% CTM2ea convert coordinate transformation matrix to 3-2-1 transformation 
%   Euler angles
%
% Gus Buonviri, 1/28/18
% Mississippi State University
%
% INPUTS:
%
%   CTM = [3 x 3] {numeric, array} coordinate transformation matrix.
%
% OUTPUTS:
%
%   ea = [3 x 1] {column vector, numeric} 3-2-1 transformation Euler angles
%       in radians. Order is roll-pitch-yaw.
%

% Converting to quaternion eliminates singularity at ea = +/-(pi/2)
ea = quat2ea(CTM2quat(CTM));

end