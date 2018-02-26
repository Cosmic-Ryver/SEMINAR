function [ qs ] = quat_conj( q )
% quat_conj Quaternion conjugate
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   q = [4 x 1] {numeric, column vector} generic quaternion, where the 
%       vector precedes the scalar.
%
% OUTPUTS:
%
%   qs = [4 x 1] {numeric, column vector} quaternion conjugate, where the
%       vector precedes the scalar.
%
% NOTE: This function is for the generic quaternion conjugate. For finding
%   quaternions representing an inverse rotation, quat_inv should be used.
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 39. Eq 2.91.
%
%

qs = q;
qs(1:3) = -1 * qs(1:3);

end

