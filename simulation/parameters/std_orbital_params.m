a         = (59 + 16.69)*6378137/2;
inc       = 63.4*pi/180;
ecc       = ((59*6378137) - a)/a;
raand     = 0;
aop       = 5.3135;
true_anom = 0;
[r, v]    = coe2rv(a,ecc,inc,raand,aop,true_anom,3.986004418e14);

rhat_ant_B = [0;cosd(60);sind(60)];
rhat_cam_B = [1;0;0];
o3 = rhat_ant_B;
o2 = rhat_cam_B;
o1 = cross(o2,o3);
CTM_ANT2B = [o1, o2, o3];
h = cross(r,v);
hhat = h/norm(h);
rhat = r/norm(r);
o3 = -rhat;
o2 = cross(hhat,rhat);
o1 = cross(o2,o3);
CTM_ANT2ECI = [o1, o2, o3];
CTM_ECI2ANT = CTM_ANT2ECI';

q_ANT2B = CTM2quat(CTM_ANT2B);
q  = CTM2quat(CTM_ANT2B*CTM_ECI2ANT);
om = cross(r,v)/norm(r)^2;

