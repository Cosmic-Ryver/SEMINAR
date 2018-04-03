ti       = 0;
dt_data  = 1;
te       = 7200;
tv_data  = ti:dt_data:te;
N_data   = length(tv_data);
jd0      = GregDate2JD(2015,1,1,0,0,0);
odeOpts  = odeset('RelTol',1e-8,'AbsTol',1e-10);