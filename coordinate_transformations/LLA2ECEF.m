function [ r ] = LLA2ECEF( lat, long, alt, rE )
% ECEF2LLA find latitude, longitude, and altitude from ECEF position vector
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   lat = {scalar, numeric} latitude in the ECEF frame.
%
%   long = {scalar, numeric} longitude in the ECEF frame.
%
%   alt = {scalar, numeric} altitude Earth's mean equatorial radius in the 
%       ECEF frame.
%
%   rE = {scalar, numeric} mean equatorial Earth radius. Units of this
%       value determine the units of r.
%
% OUTPUTS:
%
%   r = [3 x 1] {column vector, numeric} position in ECEF frame.
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 34.
%
%

% allow for input as a vector
if nargin == 2
    LLA = lat;
    rE  = long;
    
    lat  = LLA(1);
    long = LLA(2);
    alt  = LLA(3);
end

% Preallocate
r = zeros(3,1);

% Position vector magnitude
rMag = alt + rE;

% Z-axis postion
r(3) = rMag * sin(lat);

% Position vector magnitude in equatorial plane
xyMag = r(3) / tan(lat);

% Tan(long)
tLong = tan(long);

% Y-axis position
r(2) = sign(long) * sqrt(xyMag^2/(1 + (1/tLong^2)));

% X-axis position
r(1) = r(2)/tLong;

end