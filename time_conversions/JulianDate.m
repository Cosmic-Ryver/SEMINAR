function [ jd ] = JulianDate( year, month, day, hour, min, sec )
% JulianDate calculate Julian date from Gregorian calendar date
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
% NOTE: This function does not account for leap seconds.
%
% REFERENCE:
%
%   Vallado, David A. Fundamentals of astrodynamics and applications. Vol. 
%       12. Springer Science & Business Media, 2001. Algorithm 14. Pg 183.
%
%

jd = 367*year...
    -floor(7 * (year + floor((month + 9)/12))/4)...
    +floor(275 * month/9)...
    +day...
    +1721013.5...
    +((sec/60 + min)/60 + hour)/24;

end