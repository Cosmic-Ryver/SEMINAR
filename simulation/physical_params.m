classdef physical_params < sim_component
    
    properties (SetAccess = private, GetAccess = public)
        m(1,1) double
        J(3,3) double
        res_dipole(3,1) double
        surface_model
        Cd(1,1) double
    end
    
    methods
        function obj = physical_params(aOwningSimHandle,aPhysicalParamsScript)
            obj.pOwningSimHandle = aOwningSimHandle;
            struct = load_parameters(aPhysicalParamsScript,'physical');
            obj.m = struct.m;
            obj.J = struct.J;
            obj.res_dipole = struct.res_dipole;
            obj.surface_model = struct.surface_model;
            obj.Cd = struct.Cd;
        end
    end
end

