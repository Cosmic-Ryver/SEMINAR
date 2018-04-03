function [ rhat_ECI_sol, ma_sun ] = get_rhat_sun( jd_ut1 )
% get_rhat_sun calculate solar pointing direction vector in ECI coordinates
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
%   rhat_ECI_sol = [3 x 1] {colummn vector, numeric} sun pointing direction
%       in the IGRF ECI frame
%
% REFERENCES:
%
%   Vallado. pg 279-280.
%
%

deg2rad = pi/180; % conversion factor

T_ut1 = JD2T(jd_ut1); % convert to Julian centuries

lambda_ma_sun = 280.460 + 36000.771 * T_ut1; % mean longitude of sun

T_tdb = T_ut1; % approximation

ma_sun = 357.5291092 + 35999.05034 * T_tdb; % mean anomally of the sun

lambda_ec = lambda_ma_sun + 1.914666471 * sind(ma_sun) + ...
    0.019994643 * sind(2 * ma_sun); % ecliptic longitude of the sun
lambda_ec_rad = lambda_ec * deg2rad; % convert to radians
slambda = sin(lambda_ec_rad); % store trig result

% obliquity of the ecliptic
eta = 23.439291 - 0.0130042 * T_tdb - 1.64e-7 * T_tdb^2 + ...
    5.04e-7 * T_tdb^3;
eta_rad = eta * deg2rad; % convert to radians

% un-normalized pointing direction
rhat_MOD_sol = [cos(lambda_ec_rad);
                cos(eta_rad)*slambda;
                sin(eta_rad)*slambda];
            
rhat_MOD_sol = rhat_MOD_sol/norm(rhat_MOD_sol); % normalize

CTM_MOD2ECI = MOD2ECI(jd_ut1); % get CTM

rhat_ECI_sol = CTM_MOD2ECI*rhat_MOD_sol; % rotate into ECI frame



end

