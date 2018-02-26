function [ T ] = GCI2ECEF( jd, varargin )
% GCI2ECEF calculate the CTM for transformations from the GCI frame to the 
%   ECEF frame
%
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   jd = {scalar, numeric} Julian date.
%
% ALTERNATE INPUTS:
%
%   year = {scalar, numeric} Gregorian calendar year, in YYYY format.
%
%   month = {scalar, numeric} Gregorian calendar month.
%
%   day = {scalar, numeric} day of month.
%
%   hour = {scalar, numeric} hour of day.
%
%   min = {scalar, numeric} minute of hour.
%
%   sec = {scalar, numeric} second of minute.
%
% OUTPUTS:
%
%   T = [3 x 3] {array, numeric} coordinate transformation matrix for the
%       transformation from the Geocentric Inertial frame to the Earth-
%       Centered/Earth-Fixed frame.
%
% NOTE: This function and underlying fuctions do not account for leap
%   seconds.
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 32. Eq 2.67.
%
%

% Find sidereal time angle
thetaGMST = ThetaGMST(jd, varargin{:});

% Calculate sin and cos of the angle
cTheta = cos(thetaGMST);
sTheta = sin(thetaGMST);

% Calculate CTM
T = [ cTheta, sTheta, 0;
     -sTheta, cTheta, 0;
           0,      0, 1];

end