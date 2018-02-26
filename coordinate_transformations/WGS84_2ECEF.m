function [ r ] = WGS84_2ECEF( gdLat, long, height )
% WGS84_2ECEF find the ECEF position vector from the given WGS84 frame 
%   position
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   gdLat = {scalar, numeric} latitude in the WGS84 frame.
%
%   long = {scalar, numeric} longitude in the WGS84 frame. This is
%       equivalent to longitude in the ECEF frame.
%
%   height = {scalar, numeric} height in the WGS84 frame. (m)
%
% OUTPUTS:
%
%   r = [3 x 1] {column vector, numeric} position in ECEF frame. (m)
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 34-35. Eq 2.71-2.74.
%
%

% Constants
a = 6378137;      % Earth ellipsoid semimajor axis (m)
b = 6356752.3142; % Earth ellipsoid semiminor axis (m)

% Flattening
f = (a - b)/a; % 2.71

% Eccentricity of Earth ellipsoid
e = sqrt(f * (2 - f)); % 2.72
eSqr = e^2;

% Needed sin's and cos's
sLat  = sin(gdLat);
sLong = sin(long);
cLat  = cos(gdLat);
cLong = cos(long);

% Distance from z axis to ellipsoid normal vector
N = a / sqrt(1 - eSqr * sLat^2); % 2.73
Nph = N + height;

% ECEF postion vector
r = zeros(3,1);
r(1) = Nph * cLat * cLong;               % 2.74a
r(2) = Nph * cLat * sLong;               % 2.74b
r(3) = (N * (1 - eSqr) + height) * sLat; % 2.74c

end

