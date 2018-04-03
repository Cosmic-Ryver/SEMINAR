function [ Rx ] = skew( r )
% SKEW Summary of this function goes here
%
% Gus Buonviri, 3/17/18
% Mississippi State University
%
% INPUTS:
%
%   r = [3 x 1]{vector, numeric} vector to skew
%
% OUTPUTS:
%
%   Rx = [3 x 3]{array, numeric} skew symetric matrix of input vector
%
%

if (~isvector(r)) || (length(r) ~= 3)
    error('Input to skew must be a 3 element vector')
end
Rx = [    0, -r(3),  r(2);
       r(3),     0, -r(1);
      -r(2),  r(1),     0];

end

