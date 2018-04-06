function [ rho ] = exponential_atm( r_ECI )
% EXPONENTIAL_ATM calculate atmospheric density from exponential model

% reference height (m)
h0   = 1000*[0 25:5:95 100:10:150 160 180 200:50:450 500:100:1000];

% reference density (kg/m^3)
rho0 = [    1.225;  3.899e-2;  1.774e-2;  8.297e-3;  3.972e-3;  1.995e-3; 
         1.057e-3;  5.821e-4;  3.206e-4;  1.718e-4;  8.770e-5;  4.178e-5;
         1.905e-5;  8.337e-6;  3.396e-6;  1.343e-6;  5.297e-7;  9.661e-8;
         2.438e-8;  8.484e-9;  3.845e-9;  2.070e-9;  1.224e-9; 5.464e-10;
        2.789e-10; 7.248e-11; 2.418e-11; 9.158e-12; 3.725e-12; 1.585e-12;
        6.967e-13; 1.454e-13; 3.614e-14; 1.170e-14; 5.245e-15; 3.019e-15];

% scale height (m)
H0   = 1000*[8.44; 6.49; 6.75; 7.07; 7.47; 7.83; 7.95; 7.73; 7.29; 6.81;
             6.33; 6.00; 5.70; 5.41; 5.38; 5.74; 6.15; 8.06; 11.6; 16.1; 
             20.6; 24.6; 26.3; 33.2; 38.5; 46.9; 52.5; 56.4; 59.4; 62.2;
             65.8;   79;  109;  164;  225;  268];

% radius of Earth (m)
rE = 6378137;

% height (m)
h  = norm(r_ECI) - rE;

% error check
if h < 0
    error('Position below Earth'' surface')
end

% find reference height
idx = 1;
while true
    idxp1 = idx + 1;
    if idxp1 > 36
        break;
    elseif h < h0(idxp1)
        break;
    else
        idx = idxp1;
    end
end

% get density
rho = rho0(idx)*exp(-(h - h0(idx))/H0(idx));

end

