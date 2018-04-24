function [ T ] = MOD2ECI( jd_ut1 )
% MOD2ECI compute the CTM for the transformation from the MOD frame to the
%   ECI frame
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   jd_ut1 = [1 x 1] {scalar, numeric} julian date in UT1 time
%
% OUTPUTS:
%
%   T = [3 x 3] {array, numeric} coordinate transformation matrix for the
%       transformation from the Earth Centered Inertial frame to the Mean
%       of Date frame.
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 228. Eq 3.88-89.
%
%

T_ut1 = JD2T(jd_ut1); % convert to julian centuries

T_tt = T_ut1; % approximation
T_tt_sqr = T_tt^2;
T_tt_cbe = T_tt*T_tt_sqr;

arcsec2rad = pi/(3600*180); % conversion factor

% angles correct for precession

% rotation about Khat in J2000
zeta = 2306.2181 * T_tt + 0.30188 * T_tt_sqr + 0.017998 * T_tt_cbe;
zeta = zeta * arcsec2rad; % convert to rad

% rotation about intermediate Jhat
THETA = 2004.3109 * T_tt - 0.42665 * T_tt_sqr - 0.041833 * T_tt_cbe;
THETA = THETA * arcsec2rad;% convert to rad

% rotation about Khat MOD
z = 2306.2181 * T_tt + 1.09468 * T_tt_sqr + 0.018203 * T_tt_cbe;
z = z * arcsec2rad; % convert to rad

% trig terms
szeta = sin(zeta);
czeta = cos(zeta);
sTHETA = sin(THETA);
cTHETA = cos(THETA);
sz = sin(z);
cz = cos(z);

% form CTM
T = [ cTHETA*cz*czeta-sz*szeta,  sz*cTHETA*czeta+szeta*cz,  sTHETA*czeta;
     -szeta*cTHETA*cz-sz*czeta, -sz*szeta*cTHETA+cz*czeta, -sTHETA*szeta;
                    -sTHETA*cz,                -sTHETA*sz,        cTHETA];

end