ti       = 0;
dt_data  = 60;
te       = 86400;
tv_data  = ti:dt_data:te;
N_data   = length(tv_data);
jd0      = GregDate2JD(2017,6,15,0,0,0);
odeOpts  = odeset('RelTol',1e-8,'AbsTol',1e-10);

% orientation for sky region view appropriate to this time (TESS SC)
latt = [zeros(1,13) + (90-36)*pi/180, zeros(1,13) + (-90+36)*pi/180];
long = [0:360/13:360*12/13, 0:360/13:360*12/13]*pi/180;
q_c = zeros(4,26);
for i = 1:26
    q_c(:,i) = quat_prod(ea2quat([-23.14*pi/180;0;long(i)]),ea2quat([0;latt(i);0]));
end
q_c = quat_inv(q_c(:,11));

command_mode = command_mode_enumeration.pointing;