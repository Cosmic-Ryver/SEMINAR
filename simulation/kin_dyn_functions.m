classdef kin_dyn_functions < attitude_simulation
    enumeration
        two_body_dynamics (0)
        attitude_kin_dyn (1)
        lunar_third_body_perturbation (2)
        zonal_harmonics_perturbation (3)
    end
end