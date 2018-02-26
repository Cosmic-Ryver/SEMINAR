function [ lat, long, alt ] = ECEF2LLA( r, rE )
% ECEF2LLA find latitude, longitude, and altitude from ECEF position vector
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   r = [3 x 1] {column vector, numeric} position in ECEF frame.
%
%   rE = {scalar, numeric} mean equatorial Earth radius in same distance
%       units as r.
%
% OUTPUTS:
%
%   lat = {scalar, numeric} latitude in the ECEF frame.
%
%   long = {scalar, numeric} longitude in the ECEF frame.
%
%   alt = {scalar, numeric} altitude Earth's mean equatorial radius in the 
%       ECEF frame.
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 34.
%
%

% Position vector magnitude
rMag = norm(r);

% Longitude
long = atan2(r(2),r(1));

% Latitude
lat  = asin(r(3)/rMag);

% Altitude
alt  = rMag - rE;

end