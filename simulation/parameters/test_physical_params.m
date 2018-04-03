m = 300; % kg
J = diag(m/6*ones(3,1)); % inertia tensor of 1m cube
res_dipole = [0; 0; 0.1]; % residual magnetic dipole
surface_model.A  = ones(6,1);
surface_model.n  = [1 -1 0  0 0  0;
                    0  0 1 -1 0  0;
                    0  0 0  0 1 -1];
surface_model.r  = 0.5*surface_model.n;
surface_model.Rd = 0.9*ones(6,1);
surface_model.Rs = zeros(6,1);
Cd = 2;