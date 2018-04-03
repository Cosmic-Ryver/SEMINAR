function [ b ] = IGRF( r, jd, degree, r_frame_str )
% IGRF Calculate local Earth magnetic field strength from IGRF model
%

%% Declare persistent variables

persistent IGRF K

%% Parse input

rE_m = 6378137;
if strncmp( r_frame_str, 'LLA', 3) % I in LLA, O in NED
    latt  = r(1);
    long  = r(2);
    alt_m = r(3);
    
    % set output frame id
    output_frame_id = 1;
elseif strncmp( r_frame_str, 'ECI', 3) % I/O in ECI frame
    % convert to ECEF
    CTM_ECI2ECEF = ECI2ECEF(jd);
    r_ECEF = CTM_ECI2ECEF*r;
    
    % convert to LLA
    [latt, long, alt_m] = ECEF2LLA(r_ECEF, rE_m);
    
    % set output frame id
    output_frame_id = 2;
elseif strncmp(r_frame_str, 'ECEF', 4) % I/O in ECEF frame    
    % convert to LLA
    [latt, long, alt_m] = ECEF2LLA(r, rE_m);
    
    % set output frame id
    output_frame_id = 3;
else
    error('Invalid r_frame_str argument')
end

%% Read data

if isempty(IGRF)
    
    % file read and parsing
    
    IGRFdir  = fileparts(which('IGRF.m'));
    dataPath = [IGRFdir filesep 'data' filesep 'IGRF12coeffs.xls'];
    [IGRFnum, ~, IGRFraw] = xlsread(dataPath);
    IGRF.coeffs = IGRFnum(2:end,3:end);
    IGRF.years  = IGRFnum(1,3:(end - 1));
    IGRF.degree = IGRFnum(2:end,1);
    IGRF.order  = IGRFnum(2:end,2);
    IGRF.gh     = IGRFraw(5:end,1);
    
    % Schmidt normalization factors

    S2G      = NaN(max(IGRF.degree) + 1);
    S2G(1,1) = 1;
    for nIdx = 2:length(S2G)
        n           = nIdx - 1;
        S2G(nIdx,1) = S2G(nIdx - 1,1)*(2*n - 1)/n;
        for mIdx = 2:nIdx
            m              = mIdx - 1;
            S2G(nIdx,mIdx) = S2G(nIdx,mIdx - 1) * sqrt(((1 == m) + 1)*...
                (n - m + 1)/(n + m));
        end
    end

    % Schmidt normalization of model coefficients
    for i = 1:size(IGRF.coeffs,1)
        nIdx = IGRF.degree(i) + 1;
        mIdx = IGRF.order(i) + 1;
        for j = 1:size(IGRF.coeffs,2);
            IGRF.coeffs(i,j) = S2G(nIdx,mIdx)*IGRF.coeffs(i,j);
        end
    end
end

%% Linear interpolation of coefficients

% validate input date
y = JD2GregDate(jd);
if (y < IGRF.years(1)) || (y >= (IGRF.years(end) + 5))
    error('Input date exists outside of the IGRF model')
end


idxs = find(y < IGRF.years);
if isempty(idxs) % in the extrapolation regime of current model
    idx0 = length(IGRF.years);
    year0 = IGRF.years(idx0);
    year1 = year0 + 1;
    coeffDiffs = IGRF.coeffs(:,end)*5;
else % normal linear interpolation
    idx1 = idxs(1);
    idx0 = idx1 - 1;
    year0 = IGRF.years(idx0);
    year1 = IGRF.years(idx1);
    coeffDiffs = IGRF.coeffs(:,idx1) - IGRF.coeffs(:,idx0);
    
    % prevent errors from increases in model coefficients
    idxs = find(IGRF.coeffs(:,idx0) ~= 0);
    coeffDiffs((idxs(end) + 1):end) = 0;
end

% use fraction of time passed since start of model
%   avoids issues with leap years/seconds
jd0 = GregDate2JD(year0,1,1,0,0,0);
jd1 = GregDate2JD(year1,1,1,0,0,0);
timeFrac = (jd - jd0)/(jd1 - jd0);

% get the coeffs for the input time
coeffsVec = IGRF.coeffs(:,idx0) + timeFrac*coeffDiffs;

%% construct g & h matrices

% reduce degree if it is beyond what is available for the model
if degree > max(IGRF.degree)
    degree = max(IGRF.degree);
end

% set the index maximum
nIdxMax = degree + 1;

% preallocate
g = NaN(nIdxMax);
h = NaN(nIdxMax);

for i = 1:length(IGRF.gh)
    n = IGRF.degree(i);
    if n > degree
        break
    end
    m = IGRF.order(i);
    switch IGRF.gh{i}
        case 'g'
            g(n + 1,m + 1) = coeffsVec(i);
        case 'h'
            h(n + 1,m + 1) = coeffsVec(i);
    end
end

%% Trig evaluations

theta  = pi/2 - latt; % co-latitude
stheta = sin(theta);
ctheta = cos(theta);
slong  = sin(long);
clong  = cos(long);

%% auxilliary coefficients

if isempty(K)
    K       = NaN(nIdxMax);
    K(2,:)  = 0;
    for nIdx = 3:nIdxMax
        n = nIdx - 1;
        for mIdx = 1:(nIdx - 1)
            m = mIdx - 1;
            K(nIdx,mIdx) = ((n - 1)^2 - m^2)/((2*n - 1)*(2*n - 3));
        end
    end
end

%% Legendre polynomials

P      = zeros(nIdxMax);
P(1,1) = 1;
P(2,1) = ctheta;
P(2,2) = stheta;

for nIdx = 3:nIdxMax
    for mIdx = 1:(nIdx - 1)
        P(nIdx,mIdx) = ctheta*P(nIdx - 1,mIdx) -...
            K(nIdx,mIdx)*P(nIdx - 2,mIdx);
    end
    P(nIdx,nIdx) = stheta*P(nIdx - 1,nIdx - 1);
end

%% Legendre polynomials derived wrt theta

dP      = zeros(nIdxMax);
dP(1,1) = 0;
dP(2,1) = -stheta;
dP(2,2) = ctheta;

for nIdx = 3:nIdxMax
    for mIdx = 1:(nIdx - 1)
        dP(nIdx,mIdx) = ctheta*dP(nIdx - 1, mIdx) - ...
            stheta*P(nIdx - 1,mIdx) - K(nIdx,mIdx)*dP(nIdx - 2,mIdx);
    end
    dP(nIdx,nIdx) = stheta*dP(nIdx - 1,nIdx - 1) + ...
        ctheta*P(nIdx - 1, nIdx - 1);
end

%% Calculate magnetic field strength vector

b_rtp = zeros(3,1); % magnetic field strength in local tangetial coords
aor   = 6371200/(rE_m + alt_m);

for nIdx = 2:nIdxMax
    n = nIdx - 1;
    
    b_temp    = zeros(3,1);
    b_temp(1) = g(nIdx,1)*P(nIdx,1);
    b_temp(2) = g(nIdx,1)*dP(nIdx,1);
    
    smlong = 0;
    cmlong = 1;
    
    for mIdx = 2:nIdx
        m = mIdx - 1;
        
        % cos/sin of m*theta from the sum & diff formulas 
        %   ex: sin(m*theta) = sin(theta + (m-1)*theta) = ...
        smlongOld = smlong;
        smlong = slong*cmlong + clong*smlong;
        cmlong = clong*cmlong - slong*smlongOld;
        
        factor = g(nIdx,mIdx)*cmlong + h(nIdx,mIdx)*smlong;
        
        b_temp(1) = b_temp(1) + factor*P(nIdx,mIdx);
        b_temp(2) = b_temp(2) + factor*dP(nIdx,mIdx);
        b_temp(3) = b_temp(3) + ...
            m*(h(nIdx,mIdx)*cmlong - g(nIdx,mIdx)*smlong)*P(nIdx,mIdx);
    end
    
    aorrnp2 = aor^(n + 2);
    b_rtp   = b_rtp + [aorrnp2*(n + 1); -aorrnp2; -aorrnp2].*b_temp;
end

b_rtp(3) = b_rtp(3)/stheta;

%% Format output

switch output_frame_id
    case 1 % NED frame
        b = [-b_rtp(2); 
              b_rtp(3); 
             -b_rtp(1)]; 
    case 2 % ECI frame
        % sidereal time
        alpha = ThetaGMST(jd);
        salpha = sin(alpha);
        calpha = cos(alpha);
        slatt  = sin(latt);
        clatt  = cos(latt);
        
        b = [(b_rtp(1)*clatt + b_rtp(2)*slatt)*calpha - b_rtp(3)*salpha;
             (b_rtp(1)*clatt + b_rtp(2)*slatt)*salpha + b_rtp(3)*calpha;
                                        b_rtp(1)*slatt + b_rtp(2)*clatt];
    case 3 % ECEF frame
        CTM_NED2ECEF = ea2CTM([0; pi - theta; long]);
        b_NED = [-b_rtp(2); 
                  b_rtp(3); 
                 -b_rtp(1)]; 
        b = CTM_NED2ECEF*b_NED;
end

% convert nT to T
b = b*1e-9;

end