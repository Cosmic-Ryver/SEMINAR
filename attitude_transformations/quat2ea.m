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


q1 = quat(1);
q2 = quat(2);
q3 = quat(3);
q4 = quat(4);
q2Sqr = q2^2;

ea = zeros(3,1);

ea(2) = asin(2*q1*q3 - 2*q4*q2);

if abs(ea(2) - pi/2) < 1e-15     % singularity @ pitch = pi/2
    ea(1) = 0;
    ea(3) = -2 * atan2(q1,q4);
elseif abs(ea(2) + pi/2) < 1e-15 % singularity @ pitch = -pi/2
    ea(1) = 0;
    ea(3) = 2 * atan2(q1,q4);
else                             % no singularity
    ea(1) = atan2(-2*q2*q3 - 2*q4*q1, 1 - 2*q1^2 - 2*q2Sqr);
    ea(3) = atan2(-2*q1*q2 - 2*q4*q3, 1 - 2*q2Sqr - 2*q3^2);
end

end