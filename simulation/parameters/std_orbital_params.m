a         = (59 + 16.69)*6378137/2;
inc       = 63.4*pi/180;
ecc       = ((59*6378137) - a)/a;
raand     = 0;
aop       = 5.3135;
true_anom = 0;
[r, v]    = coe2rv(a,ecc,inc,raand,aop,true_anom,3.986004418e14);

o2 = [1;0;0];
o3 = [0;cosd(60);sind(60)];
o1 = cross(o2,o3);
CTM_ANT2B = [o1, o2, o3];
CTM_ECI2LVLH = ECI2LVLH(r,v);
CTM_LVLH2ANT = eye(3);

q  = CTM2quat(CTM_ECI2LVLH*CTM_LVLH2ANT*CTM_ANT2B);
om = cross(r,v)/norm(r)^2;