function quat = CTM2quat(CTM)
% CTM2quat convert coordinate transformation matrix to quaternion
%   represented transformation
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
%   quat = [4 x 1] {numeric, column vector} quaternion represented
%       transformation, where the vector precedes the scalar.
%
%

quat = zeros(4,1);

quat(4) = 0.5 * sqrt(1 + trace(CTM));

quat(1) = sign(CTM(3,2) - CTM(2,3)) * 0.5 * ...
    sqrt(1 + CTM(1,1) - CTM(2,2) - CTM(3,3));

quat(2) = sign(CTM(1,3) - CTM(3,1)) * 0.5 * ...
    sqrt(1 - CTM(1,1) + CTM(2,2) - CTM(3,3));

quat(3) = sign(CTM(2,1) - CTM(1,2)) * 0.5 * ...
    sqrt(1 - CTM(1,1) - CTM(2,2) + CTM(3,3));

quat = quat/norm(quat);

end