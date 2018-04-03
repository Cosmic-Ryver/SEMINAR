function [ mp, Pp ] = G_EKF_update( mm, Pm, z, R, h, Hx, Hr, varargin )
% G_EKF_update Update step for the first order Gaussian extended Kalman 
%   filter for non-additive noise processes
%
% Gus Buonviri, 3/6/18
% Mississippi State University
%
% INPUTS:
%
%   mm = [nX x 1] {column vector, numeric} propagated Gaussian mean, where
%       nX is the number of state variables
%
%   Pm = [nX x nX] {array, numeric} propagated Gaussian covariance matrix.
%
%   z = [nZ x 1] {column vector, numeric} measurement, where nZ is the
%    number of measurement variables.
%
%   h = [1 x 1] {scalar, function handle} non-linear measuremnet model.
%       Must accept a state vector and varargin as its inputs and return an
%       [nZ x 1] measurement vector.
%
%   Hx = [1 x 1] {scalar, function handle} Jacobian of h with respect to
%       the state vector x. Must accept a state vector and varargin as its
%       inputs and return an [nZ x nX] Jacobian matrix.
%
%   Hr = [1 x 1] {scalar, function handle} Jacobian of h with respect to
%       the measurement noise vector r. Must accept a state vector and
%       varargin as its inputs and return an [nZ x nX] Jacobian matrix.
%
%   varargin (optional) = {nvarargin x 1} {cell array} additional arguments
%       of h.
%
% OUTPUTS:
%
%   mp = [nX x 1] {column vector, numeric} updated Gaussian mean.
%
%   Pp = [nX x nX] {array, numeric} updated Gaussian covariance matrix.
%
%

v   = z - h(mm,varargin{:});
Hxm = Hx(mm,varargin{:});               % Jacobian of h w.r.t. x [nZ x nX]
Hrm = Hr(mm,varargin{:});               % Jacobian of h w.r.t. r [nZ x nX]
% S   = Hxm * Pm * Hxm' + Hrm * R * Hrm'; % Innovation  [nZ x nZ]
% K   = Pm * Hxm' / S;                    % Kalman gain [nX x nZ]
K   = Pm * Hxm' / (Hxm * Pm * Hxm' + Hrm * R* Hrm');                    % Kalman gain [nX x nZ]
mp  = mm + K * v;                       % Updated mean [nX x 1];
Pp  = (eye(length(mm)) - K * Hxm) * Pm;                  % Updated covariance [nX x nX]

end