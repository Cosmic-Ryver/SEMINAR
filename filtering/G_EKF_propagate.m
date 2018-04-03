function [ mm, Pm ] = G_EKF_propagate( m, P, Q, f, Fx, Fq, varargin )
% GM_EKF_propagate Prediction step for the first order Gaussian extended
%   Kalman filter for non-additive noise processes
%
% Gus Buonviri, 3/6/18
% Mississippi State University
%
% INPUTS:
%
%   m = [nX x 1] {column vector, numeric} Gaussian mean, where nX is the
%       length of a state vector.
%
%   P = [nX x nX] {array, numeric} Gaussian covariance matrix.
%
%   Q = [nX x nX] {array, numeric} Process noise covariance.
%
%   f = [1 x 1] {scalar, function handle} non-linear dynamics model.
%       Must accept a state vector and varargin as its inputs and return an
%       [nX x 1] propagated state vector.
%
%   Fx = [1 x 1] {scalar, function handle} Jacobian of f with respect to
%       the state vector x. Must accept a state vector and varargin as its
%       inputs and return an [nX x nX] Jacobian matrix.
%
%   Fq = [1 x 1] {scalar, function handle} Jacobian of f with respect to
%       the process noise vector q. Must accept a state vector and varargin
%       as its inputs and return an [nX x nX] Jacobian matrix.
%
%   varargin (optional) = {nvarargin x 1} {cell array} additional arguments
%       of f.
%
% OUTPUTS:
%
%   mm = [nX x 1] {column vector, numeric} propagated Gaussian mean.
%
%   Pm = [nX x nX] {array, numeric} propagated Gaussian covariance matrix.
%
%

% Update Gaussian mean
mm  = f(m,varargin{:});

% Update Gaussian covariance
Fxm = Fx(m,varargin{:}); % Jacobian of the non-linear dynamics w.r.t. x
Fqm = Fq(m,varargin{:}); % Jacobian of the non-linear dynamics w.r.t. q
Pm  = Fxm * P * Fxm' + Fqm * Q * Fqm';

end