function [ F, L ] = aero_perturbations( t, x, Cd, sc_surf_model, ...
    density_func, func_args )
% AERO_PERTURBATIONS calculate forces and torques resulting from drag

% get density
rho = density_func(t, x, func_args{:});

% get relative wind velocity
CTM_ECI2B = quat2CTM(x(7:10));
omE       = 0.000072921158583;
v_rel     = CTM_ECI2B*[x(4) + omE*x(2);
                       x(5) - omE*x(1);
                                  x(6)];
v_rel_mag = norm(v_rel);

% preallocate
N = length(sc_surf_model.A);
Fn = zeros(3,N);
Ln = zeros(3,N);

% get force and torque per face
for i = 1:N
    alpha = dot(sc_surf_model.n(:,i),v_rel)/v_rel_mag;
    Fn(:,i) = -0.5*rho*Cd*v_rel_mag*v_rel*sc_surf_model.A(i)*max(alpha,0);
    Ln(:,i) = cross(sc_surf_model.r(:,i),Fn(:,i));
end

% sum forces and torques over all faces
F = sum(Fn,2);
L = sum(Ln,2);

end

