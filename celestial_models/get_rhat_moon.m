function [ rhat_ECI_moon ] = get_rhat_moon( jd_ut1 )
% get_rhat_moon calculate moon pointing direction vector in ECI coordinates
%
% Gus Buonviri, 3/6/18
% Mississippi State University
%
% INPUTS:
%
%   jd_ut1 = [1 x 1] {scalar, numeric} julian date in UT1 time
%
% OUTPUTS:
%
%   rhat_ECI_moon = [3 x 1] {colummn vector, numeric} moon pointing
%       direction in the IGRF ECI frame
%
% REFERENCES:
%
%   Vallado. pg 288.
%
%

deg2rad = pi/180; % conversion factor

T_ut1 = JD2T(jd_ut1); % convert to Julian centuries

T_tdb = T_ut1; % approximation

% ecliptic longitude of the moon
lambda_ec = 218.32 + 481267.8813*T_tdb + ...
    6.29*sind(134.9 + 477198.85*T_tdb) - ...
    1.27*sind(259.2 - 413335.38*T_tdb) + ...
    0.66*sind(235.7 + 890534.23*T_tdb) + ...
    0.21*sind(269.9 + 954397.70*T_tdb) - ...
    0.19*sind(357.5 +  35999.05*T_tdb) - ...
    0.11*sind(186.6 + 966404.05*T_tdb);
lambda_ec_rad = lambda_ec * deg2rad; % convert to radians

% ecliptic latitude of the moon
phi_ec = 5.13*sind(93.3 + 483202.03*T_tdb) + ...
    0.28*sind(228.2 + 960400.87*T_tdb) - ...
    0.28*sind(318.3 +   6003.18*T_tdb) - ...
    0.17*sind(217.6 - 407332.20*T_tdb);
phi_ec_rad = phi_ec * deg2rad; % convert to radians

% obliquity of the ecliptic
eta = 23.439291 - 0.0130042 * T_tdb - 1.64e-7 * T_tdb^2 + ...
    5.04e-7 * T_tdb^3;
eta_rad = eta * deg2rad; % convert to radians

% trig terms
slambda = sin(lambda_ec_rad);
clambda = cos(lambda_ec_rad);
sphi    = sin(phi_ec_rad);
cphi    = cos(phi_ec_rad);
seta    = sin(eta_rad);
ceta    = cos(eta_rad);

% un-normalized pointing direction
rhat_ECI_moon = [cphi*clambda;
                 ceta*cphi*slambda - seta*sphi;
                 seta*cphi*slambda + ceta*sphi];
            
rhat_ECI_moon = rhat_ECI_moon/norm(rhat_ECI_moon); % normalize

end

