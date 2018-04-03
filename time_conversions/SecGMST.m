function [ secGMST ] = SecGMST( T0, hour, min, sec )
% ThetaGMST Calculate the Greenwich Mean Sidereal Time angle cooresponding
%   to the input date
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   T0 = {scalar, numeric} number of Julian centuries from the J2000 epoch
%       to 0 hour of the relevant date.
%
%   hour = {scalar, numeric} hour of day.
%
%   min = {scalar, numeric} minute of hour.
%
%   sec = {scalar, numeric} second of minute.
%
% OUTPUTS:
%
%   secGMST = {scalar, numeric} Greenwich Mean Sidereal Time in seconds
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 34. Eq 2.70.
%
%

secInDay = 3600 * hour + 60 * min + sec;

secSinceEpoch =   24110.54841 ...
                + 8640184.812866 * T0 ...
                + 0.093104 * T0^2 ...
                - 6.2e-6 * T0^3 ...
                + 1.002737909350795 * secInDay;
                
secGMST = rem(secSinceEpoch, 86400);

end

