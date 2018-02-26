function [ q_inv ] = quat_inv( q )
% quat_inv Quaternion inverse
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
%   q_inv = [4 x 1] {numeric, column vector} inverse quaternion, where the
%       vector precedes the scalar.
%
% NOTE: For quaternions representing a rotation, quat_inv gives the
%   inverse rotation. Ex: if q represents a rotation from a frame A to a
%   frame B, q_inv represents the rotation from frame B to frame A.
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 39. Eq 2.95.
%
%

q_inv = quat_conj(q) / norm(q)^2;

end

