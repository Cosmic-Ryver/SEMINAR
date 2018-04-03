function [ r_ECI_moon ] = get_r_moon( jd_ut1, rE )
% get_r_moon calculate position vector of moon in ECI coordinates
%
% Gus Buonviri, 3/6/18
% Mississippi State University
%
% INPUTS:
%
%   jd_ut1 = [1 x 1] {scalar, numeric} julian date in UT1 time
%
%   rE = [1 x 1] {scalar, numeric} radius of the Earth
%
% OUTPUTS:
%
%   r_ECI_moon = [3 x 1] {colummn vector, numeric} position vector of moon
%       in the IGRF ECI frame
%
% REFERENCES:
%
%   Vallado. pg 288.
%
%

rhat_ECI_moon = get_rhat_moon(jd_ut1); % pointing vector of moon

T_ut1 = JD2T(jd_ut1);
T_tdb = T_ut1;

rad2deg = pi/180; % conversion factor

phi = 0.9508 + 0.0518*cosd(134.9 + 477198.85*T_tdb) + ...
    0.0095*cosd(259.2 - 413335.38*T_tdb) + ...
    0.0078*cosd(235.7 + 890534.23*T_tdb) + ...
    0.0028*cosd(269.9 + 954397.70*T_tdb);
phi_rad = phi*rad2deg; % convert to radians

rmag_ECI_moon = rE/sin(phi_rad); % position magnitude

r_ECI_moon = rmag_ECI_moon * rhat_ECI_moon; % moon position vector

end

