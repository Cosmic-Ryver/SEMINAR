function [ Ps, rhat_sun_rel ] = solar_rad_press( r_ECI, jd )
% SOLAR_RAD_PRESS calculate solar radiation pressure for an earth orbiting
%   satellite

% constants
rE = 6378137;      % radius of Earth (m)
AU = 149597870700; % length of 1 AU (m)	
Fs = 1362;         % solar constant (W/m^2)
c  = 299792458;    % speed of light in a vacuum (m/s)

% sun position vector
r_sun = get_r_sun(jd, AU);

% position of the sun relative to the input position
r_sun_rel = r_sun - r_ECI;

% get distance to sun
rmag_sun_rel = norm(r_sun_rel);

% get pointing direction to sun
rhat_sun_rel = r_sun_rel/rmag_sun_rel;

% determine eclipse state
isNotEclipsed = dot(r_ECI,rhat_sun_rel) > -sqrt(norm(r_ECI)^2 - rE^2);

if isNotEclipsed
    Ps = Fs/(c*(rmag_sun_rel/AU)^2);
else
    Ps = 0;
end
end

