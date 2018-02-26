function [ gdLat, long, height ] = ECEF2WGS84( r )
% ECEF2WGS84 find the WGS84 frame position from the given ECEF position
%   vector
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   r = [3 x 1] {column vector, numeric} position in ECEF frame. (m)
%
% OUTPUTS:
%
%   gdLat = {scalar, numeric} latitude in the WGS84 frame.
%
%   long = {scalar, numeric} longitude in the WGS84 frame. This is
%       equivalent to longitude in the ECEF frame.
%
%   height = {scalar, numeric} height in the WGS84 frame. (m)
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 34-35. Eq 2.77.
%
%

% Constants
a = 6378137;      % Earth ellipsoid semimajor axis (m)
b = 6356752.3142; % Earth ellipsoid semiminor axis (m)

% ECEF position components
x = r(1);
y = r(2);
z = r(3);

% Square of semimajor & semiminor axes
aSqr = a^2;
bSqr = b^2;

% 2.77a
eSqr = 1 - bSqr/aSqr;
epsSqr = aSqr/bSqr - 1;
rho = sqrt(x^2 + y^2);

% 2.77b
p = abs(z)/epsSqr;
s = rho^2/(eSqr * epsSqr);
q = p^2 - bSqr + s;
qSqrt = sqrt(q);

% 2.77c
u = p / qSqrt;
uSqr = u^2
v = bSqr * uSqr / q;
P = 27 * v * s/q;
Q = (sqrt(P + 1) + sqrt(P))^(2/3);

% 2.77d
t = (1 + Q + 1/Q)/6;
c = sqrt(uSqr - 1 + 2*t);
w = (c - u)/2;

% 2.77e
d = sign(z) * qSqrt * (w + sqrt(sqrt(t^2 + v) - u*w - t/2 - 0.25));

% 2.77f
N = a * sqrt(1 + epsSqr * d^2/bSqr);
gdLat = asin((epsSqr + 1) * (d/N));

% 2.77g
height = rho*cos(gdLat) + z*sin(gdLat) - aSqr/N;
long = atan2(y,x);

end