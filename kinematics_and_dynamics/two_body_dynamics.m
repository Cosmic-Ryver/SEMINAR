function [ dxdt ] = two_body_dynamics( t, x, mu )
% two_body_dynamics Two body orbital dynamics
%
% Gus Buonviri, 2/26/18
% Mississippi State University
%
% INPUTS:
%
%   t = {scalar, numeric} time value (s)
%
%   x = [nX x 1] {column vector, numeric} state vector, where (1:3) is the
%       position vector (L), (4:6) is the velocity vector (L/s), (7:10) 
%       is the quaternion represented attitude of the body frame wrt the 
%       inertial frame, (11:13) is the angular rotation rate vector of the
%       body frame wrt the inertial frame expressed in the body frame 
%       (rad/s), and (14:nX) is the angular momentum vector of the 
%       spacecraft's (nX - 13)/3 reaction wheels expressed in the body
%       frame (N*m*s).
%
%   mu = {scalar, numeric} earth gravitational parameter L^3/s^2
%
% OUTPUTS:
%
%   dxdt = [nX x 1] {column vector, numeric} state vector time derivative
%

% Initialize state vector time derivative
dxdt = zeros(length(x),1);

% Velocity
dxdt(1:3) = x(4:6);

% Acceleration
dxdt(4:6) = -x(1:3) * mu / norm(x(1:3))^3;

end

