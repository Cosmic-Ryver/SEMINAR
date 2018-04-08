a         = (59 + 16.69)*6378137/2;
inc       = 63.4*pi/180;
ecc       = ((59*6378137) - a)/a;
raand     = 0;
aop       = 5.3135;
true_anom = 0;
[r, v]    = coe2rv(a,ecc,inc,raand,aop,true_anom,3.986004418e14);