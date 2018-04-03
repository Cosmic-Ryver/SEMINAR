function [ dxdt ] = kin_dyn_superposition( t, x, m, J, L_int, ...
    force_funcs, force_args, kin_dyn_funcs, kin_dyn_args )
% DYN_KIN_SUPERPOSITION Calculate state vector time derivative
%
% Gus Buonviri, 2/26/18
% Mississippi State University
%
% INPUTS:
%
%   t = {scalar, numeric} time value (s)
%
%   x = [nX x 1] {column vector, numeric} state vector, where nX is the 
%       number of independent state variables.
%
%   m = {scalar, numeric} mass of spacecraft
%
%   J = [3 x 3] {array, numeric} inertia tensor of spacecraft
%
%   L_int = [3 x 1] {column vector, numeric} control torque vector of 
%       spacecraft
%
%   force_funcs = {nFunc x 1} {cell array, column vector, function handles}
%       array of function handles for calculating forces and torques, where 
%       nFunc is the number of unique functions.
%
%   force_args = {nFunc x 1} {cell array, column vector, cell arrays}
%       nested array of function inputs. Each element contains a column 
%       vector cell array of the inputs following (t, x) of the 
%       corresponding force_funcs function.
%
%   kin_dyn_funcs = {nFunc x 1} {cell array, column vector, function handles}
%       array of function handles for calculating terms of dxdt, where 
%       nFunc is the number of unique functions.
%
%   kin_dyn_args = {nFunc x 1} {cell array, column vector, cell arrays}
%       nested array of function inputs. Each element contains a column 
%       vector cell array of the inputs following (t, x) of the 
%       corresponding kin_dyn_funcs function.
%
% OUTPUTS:
%
%   dxdt = [nX x 1] {column vector, numeric} state vector time derivative.
%

% preallocate
F     = zeros(3,1);
L_ext = zeros(3,1);
dxdt  = zeros(length(x),1);

% enforce unit quaternion
x(7:10) = x(7:10)/norm(x(7:10));

% allow for common arguments
if length(force_args) == 1
    isArgsCommon = true;
else
    isArgsCommon = false;
end

% get forces and external torques
N = length(force_funcs);
for i = 1:N
    funci    = force_funcs{i};
    if isArgsCommon
        argsi = force_args{1};
    else
        argsi    = force_args{i};
    end
    [Fi, Li] = funci(t,x,argsi{:});
    F        = F + Fi;
    L_ext    = L_ext + Li;
end

% apply forces and torques
dxdt(4:6) = dxdt(4:6) + F/m;
dxdt      = dxdt      + attitude_kin_dyn(t,x,J,L_int,L_ext);

% allow for common arguments
if length(kin_dyn_args) == 1
    isArgsCommon = true;
else
    isArgsCommon = false;
end

% apply other kinematics and dynamics
N = length(kin_dyn_funcs);
for i = 1:N
    funci = kin_dyn_funcs{i};
    if isArgsCommon
        argsi = kin_dyn_args{1};
    else
        argsi = kin_dyn_args{i};
    end
    dxdti = funci(t,x,argsi{:});
    dxdt  = dxdt + dxdti;
end

end

