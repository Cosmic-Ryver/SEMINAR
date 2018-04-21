function [ F, L ] = rp_perturbations( t, x, sc_surf_model, ...
    rad_press_func, rp_func_args )
% RP_PERTURBATIONS calculate forces and torques resulting from rad press

% get pressure and source pointing direction
[P, s] = rad_press_func(t, x, rp_func_args{:});

% preallocate
N     = length(sc_surf_model.A);
Fn    = zeros(3,N);
Ln    = zeros(3,N);
alpha = sum(sc_surf_model.n.*repmat(s,1,N),1);

% get force and torque per face
for i = 1:N
    r      = sc_surf_model.r(:,i); % center of pressure
    n      = sc_surf_model.n(:,i); % surface normal
    A      = sc_surf_model.A(i);   % surface area
    Rd     = sc_surf_model.Rd(i);  % diffuse reflection coefficient
    Rs     = sc_surf_model.Rs(i);  % specular reflection coefficient 
    alphai = alpha(i);            % incidence angle
    
    Fn(:,i) = -P*A*(2*(Rd/3 + Rs*alphai)*n + (1 - Rs)*s)*max(alphai,0);
    Ln(:,i) = cross(r,Fn(:,i));
end

% sum forces and torques over all faces
F = sum(Fn,2);
L = sum(Ln,2);

end