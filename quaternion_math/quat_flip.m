function [ qp ] = quat_flip( q )
% quat_flip Flip a quaternion to the other side of the quaternion sphere
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   q = [4 x 1] {column vector, numeric} quaternion to flip, where the 
%       vector precedes the scalar
%
% OUTPUTS:
%
%   qp = [4 x 1] {column vector, numeric} flipped quaternion, where the 
%       vector precedes the scalar
%
%

qp = -1 * q;

end