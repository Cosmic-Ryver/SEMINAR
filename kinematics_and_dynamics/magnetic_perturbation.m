function [ F, L ] = magnetic_perturbation( t, x, res_dipole, mag_field_func, ...
    func_args )
% MAGNETIC_PERTURBATION get perturbing torque caused by residual dipoles

% Functions of this type must output force and torque
%   magnetic dipoles can't produce a force, so output is zero
F = zeros(3,1);

% get mag field vector
b_ECI = mag_field_func(t, x, func_args{:});

% get coord transfer matrix
CTM_ECI2B = quat2CTM(x(7:10));

% rotate mag field vector into body frame
b_B = CTM_ECI2B*b_ECI;

% get body frame torques
L = cross(res_dipole, b_B);

end

