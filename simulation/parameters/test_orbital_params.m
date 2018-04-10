a         = 6778137;
inc       = 0;
ecc       = 0;
raand     = 0;
aop       = 2;
true_anom = 0;
[r, v]    = coe2rv(a,ecc,inc,raand,aop,true_anom,3.986004418e14);

ea   = [pi/4;pi/4;pi/4];
q    = ea2quat(ea);

om = pi/180*[0.1; 0.1; 0.1];