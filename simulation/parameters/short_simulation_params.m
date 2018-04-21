ti       = 0;
dt_data  = 0.25;
te       = 3600*2;
tv_data  = ti:dt_data:te;
N_data   = length(tv_data);
jd0      = GregDate2JD(2017,6,15,0,0,0);
odeOpts  = odeset('RelTol',1e-8,'AbsTol',1e-10);