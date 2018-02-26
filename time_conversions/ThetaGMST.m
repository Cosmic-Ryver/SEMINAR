function [ thetaGMST ] = ThetaGMST( jd, varargin )
% ThetaGMST Calculate the Greenwich Mean Sidereal Time angle cooresponding
%   to the input date
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
%   thetaGMST = {scalar, numeric} Greenwich Mean Sidereal Time angle in
%       randians.
%
% NOTE: This function does not account for leap seconds.
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 33-34. Eq 2.69.
%
%

if nargin == 1
    
    % Conversion factor
    secPerDay   = 86400;

    % Find Julian date of day's zero hour
    jday = floor(jd - 0.5) + 0.5;

    % Find decimal value of the days in the current day
    daysInDay = jd - jday;

    % Find the number of seconds in the day
    sec = daysInDay * secPerDay;
    
    % Set hours and minutes to zero
    hour = 0;
    min  = 0;

elseif nargin == 6
    
    % Reassign input variables
    year = jd;
    month = varargin{1};
    day   = varargin{2};
    hour  = varargin{3};
    min   = varargin{4};
    sec   = varargin{5};
    
    % Find the Julian day
    jday = JulianDate(year, month, day, 0, 0, 0);

else

    error('Invalid number of input arguments')
    
end

% Find the number of Julian centuries since the J2000 epoch
T0 = (jday - 2451.545)/36.525;

% Find the sidereal time in seconds
secGMST = SecGMST(T0, hour, min, sec);

% Convert sidereal time to angles in degrees
thetaDegGMST = secGMST/240;

% Convert degrees to radians
thetaGMST = thetaDegGMST * pi/180;

end