function CTM = quat2CTM(quat)
% quat2CTM convert quaternion represented transformation to coordinate
%   transformation matrix
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
%   CTM = [3 x 3] {numeric, array} coordinate transformation matrix.
%
%

x2q1p2 = 2 * quat(1)^2;
x2q2p2 = 2 * quat(2)^2;
x2q3p2 = 2 * quat(3)^2;
x2q1q2 = 2 * quat(1) * quat(2);
x2q1q3 = 2 * quat(1) * quat(3);
x2q2q3 = 2 * quat(2) * quat(3);
x2q4q1 = 2 * quat(4) * quat(1);
x2q4q2 = 2 * quat(4) * quat(2);
x2q4q3 = 2 * quat(4) * quat(3);

CTM = [1 - x2q2p2 - x2q3p2,     x2q1q2 - x2q4q3,     x2q1q3 + x2q4q2;
           x2q1q2 + x2q4q3, 1 - x2q1p2 - x2q3p2,     x2q2q3 - x2q4q1;
           x2q1q3 - x2q4q2,     x2q2q3 + x2q4q1, 1 - x2q1p2 - x2q2p2];

CTM(abs(CTM)<1e-15) = 0;
       
end