function [ mjd ] = ModifiedJulianDate( jd )
% ModifiedJulianDate calculate modified Julian date from standard Julian
%   date
%
% Gus Buonviri, 2/18/2018
% Mississippi State University
%
% INPUTS:
%
%   jd = {scalar, numeric} Julian date.
%
% OUTPUTS:
%
%   mjd = {scalar, numeric} modified Julian date.
%
% REFERENCE:
%
%   Vallado, David A. Fundamentals of astrodynamics and applications. Vol. 
%       12. Springer Science & Business Media, 2001. Pg 183.
%
%

mjd = jd - 2400000.5;

end

