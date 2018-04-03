function [ year, month, day, hour, min, sec ] = JD2GregDate( jd )
% GregDate2JD calculate Gregorian calendar date from Julian date
%
% Gus Buonviri, 2/18/2018
% Mississippi State University
%
% INPUTS:
%
%   jd = {scalar, numeric} Julian date corresponding to input Gregorian
%       calendar date.
%
% OUTPUTS:
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
% REFERENCE:
%
%   Vallado, David A. Fundamentals of astrodynamics and applications. Vol. 
%       12. Springer Science & Business Media, 2001. Algorithm 22. Pg 202.
%
%

T1900 = (jd - 2415019.5)/365.25;
year = 1900 + floor(T1900);
leapYrs = floor((year - 1900 - 1)*0.25);
days = jd - 2415019.5 - (year - 1900)*365 - leapYrs;
if days < 1
    year = year - 1;
    leapYrs = floor((year - 1900 - 1)*0.25);
    days = jd - 2415019.5 - (year - 1900)*365 - leapYrs;
end
lmonth = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
if mod(year,4) == 0
    lmonth(2) = 29;
end
doy = floor(days);
month   = 1;
dtm = lmonth(1);
while dtm < doy
    month = month + 1;
    dtm = dtm + lmonth(month);
end
day = doy - dtm + lmonth(month);
tau = (days - doy)*24;
hour = floor(tau);
min = floor((tau - hour)/60);
sec = (tau - hour - min/60)*3600;

end