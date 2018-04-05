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

% enforce unit quaternion
x(7:10) = x(7:10)/norm(x(7:10));

% allow for common arguments
if length(force_args) == 1
    isFArgsCommon = true;
else
    isFArgsCommon = false;
end

% allow for common arguments
if length(kin_dyn_args) == 1
    isKDArgsCommon = true;
else
    isKDArgsCommon = false;
end

% get forces and external torques
N1 = length(force_funcs);
N2 = length(kin_dyn_funcs);
funcs = { force_funcs{:}, kin_dyn_funcs{:} };
args  = { force_args{:}, kin_dyn_args{:} };
N12 = N1 + N2;
Fn = zeros(3,N12);
Ln = zeros(3,N12);
dxdtn = zeros(length(x),N12);
for i = 1 : N12
    if i <= N1
        funci    = funcs{i};
        if isFArgsCommon
            argsi = args{1};
        else
            argsi    = args{i};
        end
        [Fi, Li] = funci(t,x,argsi{:});
        Fn(:,i)    = Fi;
        Ln(:,i)    = Li;
    elseif i > N1
        funci = funcs{i};
        if isKDArgsCommon
            if isFArgsCommon
                argsi = args{2};
            else
                argsi = args{1 + N1};                
            end
        else
            argsi = args{i};
        end
        dxdti = funci(t,x,argsi{:});
        dxdtn(:,i) = dxdti;
    end
end

% sum results of parfor
F     = sum(Fn,2);
L_ext = sum(Ln,2);
dxdt  = sum(dxdtn,2);

% apply forces and torques
dxdt(4:6) = dxdt(4:6) + F/m;
dxdt      = dxdt      + attitude_kin_dyn(t,x,J,L_int,L_ext);

end

