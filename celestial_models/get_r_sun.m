function [ r_ECI_sun ] = get_r_sun( jd_ut1, AU )
% get_r_moon calculate position vector of sun in ECI coordinates
%
% Gus Buonviri, 3/6/18
% Mississippi State University
%
% INPUTS:
%
%   jd_ut1 = [1 x 1] {scalar, numeric} julian date in UT1 time
%
%   AU = [1 x 1] {scalar, numeric} length of 1 AU in the desired distance
%       unit of the output
%
% OUTPUTS:
%
%   r_ECI_sun = [3 x 1] {colummn vector, numeric} position vector of sun
%       in the IGRF ECI frame
%
% REFERENCES:
%
%   Vallado. pg 280.
%
%

[rhat_ECI_sun, ma_sun] = get_rhat_sun(jd_ut1); % pointing vector of sun

% get distance to sun in units of AU
rmag_ECI_sun = 1.000140612 - 0.016708617*cosd(ma_sun) - ...
    0.000139589*cosd(2*ma_sun);

rmag_ECI_sun = rmag_ECI_sun*AU; % convert to km or m

r_ECI_sun = rmag_ECI_sun*rhat_ECI_sun; % sun position vector

end

