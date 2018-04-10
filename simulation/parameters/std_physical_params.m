m = 365; % kg
J = diag([5/12*m*a^2, 0 , 0]); % inertia tensor of 1m cube
res_dipole = [0; 0; 0.1]; % residual magnetic dipole
surface_model.A  = [1.8*0.6*cosd(60)*ones(1,2), 0.6*1.5*ones(1,6), ...
    1.1*0.89*ones(1,4)];
a = sind(30);
b = cosd(30);
surface_model.n  = [1 -1 0  0 0  0  0  0 0  0 0  0;
                    0  0 1 -1 a -a  a -a 0  0 0  0;
                    0  0 0  0 b  b -b -b 1 -1 1 -1];
surface_model.r  = repmat([0.75*ones(1,2) 0.6*cosd(60)*ones(1,6), ...
    0*ones(1,4)],3,1).*surface_model.n;
surface_model.r(:,9:12) = repmat([0; 1.4; 0],1,4);
surface_model.Rd = [0.9*ones(1,8), 0.1*ones(1,4)];
surface_model.Rs =  zeros(1,12);
Cd = 2;