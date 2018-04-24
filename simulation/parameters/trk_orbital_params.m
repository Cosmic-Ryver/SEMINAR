a         = (59 + 16.69)*6378137/2;
inc       = 63.4*pi/180;
ecc       = ((59*6378137) - a)/a;
raand     = 0;
aop       = 5.3135;
true_anom = 2*pi - 9.334*pi/180;
[r, v]    = coe2rv(a,ecc,inc,raand,aop,true_anom,3.986004418e14);

rhat_ant_B = [0;cosd(60);sind(60)];
rhat_cam_B = [1;0;0];
o3 = rhat_ant_B;
o2 = rhat_cam_B;
o1 = cross(o2,o3);
CTM_ANT2B = [o1, o2, o3];
q_ANT2B = CTM2quat(CTM_ANT2B);

% orientation for sky region view appropriate to this time (TESS SC)
latt = [zeros(1,13) + (90-36)*pi/180, zeros(1,13) + (-90+36)*pi/180];
long = [0:360/13:360*12/13, 0:360/13:360*12/13]*pi/180;
q_c = zeros(4,26);
for i = 1:26
    q_c(:,i) = quat_prod(ea2quat([-23.14*pi/180;0;long(i)]),ea2quat([0;latt(i);0]));
end
q  = quat_inv(q_c(:,11));
om = zeros(3,1);

