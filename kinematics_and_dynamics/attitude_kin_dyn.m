function [ dxdt ] = attitude_kin_dyn( t, x, J, L_int_w, L_ext )
% attitude_kin_dyn Basic rigid-body attitude dynamics
%s
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
%   J = [3 x 3] {array, numeric} inertia tensor (kg*m^2).
%
%   L_int_w = [3 x nW] {array, numeric} internal torques acting on 
%       the spacecraft in the body frame (N*m). nW is the number of
%       reaction wheels within the spacecraft, which is equivalent to 
%       (nX - 13)/3.
%
%   L_ext = [3 x 1] {array, numeric} external torque torques acting on 
%       the spacecraft in the body frame (N*m).
%
% OUTPUTS:
%
%   dxdt = [nX x 1] {column vector, numeric} state vector time derivative
%

% Initialize state vector time derivative
dxdt = zeros(length(x),1);

% Get number of reaction wheels
nW = (length(x) - 13)/3;

if nW > 0
    % Wheel momentum rates of change
    dxdt(14:end) = reshape(L_int_w,3*nW,1);

    % Total current wheel momentum
    H = sum(reshape(x(14:end),3,nW), 2);
    
    % Total current internal torque
    L_int = sum(L_int_w, 2);
else
    % No wheels = no momentum or internal torques
    H     = zeros(3,1);
    L_int = zeros(3,1);
end

% Angular acceleration
dxdt(11:13) = J\(L_ext - L_int - cross(x(11:13), J*x(11:13) + H));

% Quaternion rate of change
dxdt(7:10) = 0.5*quat_prod([x(11:13); 0], x(7:10));

end

