function [ qp ] = quat_pq4( q )
% quat_pq4 Restrict a quaternion to the hemisphere associated with
%   possitive values of q4
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
%   qp = [4 x 1] {numeric, column vector} hemisphere-restricted quaternion,
%       where the vector precedes the scalar.
%
% NOTE: This function should not be used on quaternions that are being 
%   integrated or derived, because it inevitably results in discontinuities
%   as the quaternion jumps around the edge of the quaternion sphere 
%   hemisphere associated with possitive values of q4.
%
%

if q(4) < 0
    qp = quat_flip(q);
else
    qp = q;
end

end

