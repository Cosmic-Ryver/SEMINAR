function [ jd ] = GregDate2JD( year, month, day, hour, min, sec )
% GregDate2JD calculate Julian date from Gregorian calendar date
%
% Gus Buonviri, 2/18/2018
% Mississippi State University
%
% INPUTS:
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
%   jd = {scalar, numeric} Julian date corresponding to input Gregorian
%       calendar date.
%
% REFERENCE:
%
%   Vallado, David A. Fundamentals of astrodynamics and applications. Vol. 
%       12. Springer Science & Business Media, 2001. Algorithm 14. Pg 183.
%
%

% deal with leap seconds
YMDls = getLeapSecData();
secPerDay = 60;
yearChk = YMDls(:,1) == year;
if any(yearChk)
    if YMDls(yearChk,2) == month
        if YMDls(yearChk,3) == day
            secPerDay = 61;
        end
    end
end

% get Julian Date
jd = 367*year...
    -floor(7 * (year + floor((month + 9)/12))/4)...
    +floor(275 * month/9)...
    +day...
    +1721013.5...
    +((sec/secPerDay + min)/60 + hour)/24;

end