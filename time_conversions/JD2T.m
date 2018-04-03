function [ T ] = JD2T( jd )
% JD2T Julian date to Julian century conversion
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   jd = [1 x 1] {scalar, numeric} Julian date
%
% OUTPUTS:
%
%   T = [1 x 1] {scalar, numeric} Julian century
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 184. Eq 3.42.
%
%

T = (jd - 2451545)/36525;

end

