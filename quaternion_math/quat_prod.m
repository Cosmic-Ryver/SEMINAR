function [ qxp ] = quat_prod( q, p )
% quat_prod Quaternion multiplication
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   q = [4 x 1] {numeric, column vector} first quaternion to multiply,
%       where the vector precedes the scalar.
%
%   p = [4 x 1] {numeric, column vector} second quaternion to multiply,
%       where the vector precedes the scalar.
%
% OUTPUTS:
%
%   qxp = [4 x 1] {numeric, column vector} quaternion product, where the
%       vector precedes the scalar.
%
% NOTE: For input quaternions that represent successive rotations, the
%   product represents the rotation from the intitial frame to the final
%   frame. Ex: if q represents the rotation from a frame A to a frame B and
%   p represents the rotation from a frame B to a frame C, then qxp
%   represents the rotation from frame A to frame C.
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 37. Eq 2.82a.
%
%

qxp = zeros(4,1);

qxp(1:3) = p(4) * q(1:3) + q(4) * p(1:3) - cross(q(1:3),p(1:3));
qxp(4)   = q(4) * p(4) - dot(q(1:3),p(1:3));

end

