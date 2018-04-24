function [ F, L ] = grav_grad_perturbation( t, x, mu, J )
%GRAV_GRAD_PERTURBATION perturbing torques due to gravity gradient

% Functions of this type must output force and torque
%   gravity gradient effect can't produce a force, so output is zero
F = zeros(3,1);

% Convert attitude quaternion to 321 Euler angles of body frame wrt LVLH
quat_ECI2B    = x(7:10);
CTM_ECI2LVLH  = ECI2LVLH(x(1:3),x(4:6));
quat_ECI2LVLH = CTM2quat(CTM_ECI2LVLH);
quat_LVLH2ECI = quat_inv(quat_ECI2LVLH);
quat_LVLH2B   = quat_prod(quat_ECI2B,quat_LVLH2ECI);
ea            = quat2ea(quat_LVLH2B);

% Calculate the gravity gradient torque
L = 3*mu/norm(x(1:3))^3*...
    [(J(3,3) - J(2,2))*cos(ea(2))^2*cos(ea(1))*sin(ea(1));
       (J(3,3) - J(1,1))*cos(ea(2))*sin(ea(2))*cos(ea(1));
       (J(1,1) - J(2,2))*cos(ea(2))*sin(ea(2))*sin(ea(1))];

end

