function [ dxdt ] = third_body_perturbations( t, x, mu3, r3_func, func_args )
% third_body_perturbations Calculate acceleration due to gravity of
%   additional bodies
%
% Gus Buonviri, 3/6/18
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
%   mu3 = [N x 1] {column vector, numeric} gravitational parameters of 
%       additional bodies (L^3/s^2), where N is number of additional bodies
%
%   r3_func = {N x 1} {cell array, column vector, function handle} function
%       handle for the ECI position vector of the additional bodies. Must
%       take arguments of (t, x, func_args{i}{:}). It is recomended that
%       these handles reference an anonymous function that serves as an
%       interface to a traditional matlab function.
%
%   func_args = {N x 1} {nested cell array, column vector} Each element is
%       a vectorized cell array of the additional arguments needed for the
%       corresponding function handle in r3_func.
%
% OUTPUTS:
%
%   dxdt = [nX x 1] {column vector, numeric} state vector time derivative
%
%

N = length(r3_func);
dxdt = zeros(length(x),1);
r = x(4:6);

for i = 1:N
    % get position vector of 3rd body
    func_i     = r3_func{i};
    args_i     = func_args{i};
    r3_i       = func_i(t,x,args_i{:});
    
    % get acceleration due to 3rd body
    mu3_i      = mu3(i);
    r3_rel     = r - r3_i;
    mag_r3_rel = norm(r3_rel);
    u          = r3_rel/mag_r3_rel;
    a3_i       = -mu3_i*(r - 3*dot(u,r)*u)/mag_r3_rel^3;
    
    % add acceleration 
    dxdt(4:6)  = dxdt(4:6) + a3_i;
end
end