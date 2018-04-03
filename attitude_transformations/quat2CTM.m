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

x2q1q2 = 2 * quat(1) * quat(2);
x2q1q3 = 2 * quat(1) * quat(3);
x2q2q3 = 2 * quat(2) * quat(3);
x2q1q4 = 2 * quat(4) * quat(1);
x2q2q4 = 2 * quat(4) * quat(2);
x2q3q4 = 2 * quat(4) * quat(3);
q1p2   = quat(1)^2;
q2p2   = quat(2)^2;
q3p2   = quat(3)^2;
q4p2   = quat(4)^2;

CTM = [q1p2-q2p2-q3p2+q4p2,      x2q1q2 + x2q3q4,      x2q1q3 - x2q2q4;
           x2q1q2 - x2q3q4, -q1p2+q2p2-q3p2+q4p2,      x2q2q3 + x2q1q4;
           x2q1q3 + x2q2q4,      x2q2q3 - x2q1q4, -q1p2-q2p2+q3p2+q4p2];
       
end