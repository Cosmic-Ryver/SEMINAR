classdef orbital_params < sim_component
    
    properties (SetAccess = private, GetAccess = public)
        a(1,1) double
        inc(1,1) double
        ecc(1,1) double
        raand(1,1) double
        aop(1,1) double
        true_anom(1,1) double
        r(3,1) double
        v(3,1) double
    end
    
    methods
        function obj = orbital_params(aOwningSimHandle,aOrbitalParamsScript)
            obj.pOwningSimHandle = aOwningSimHandle;
            struct = load_parameters(aOrbitalParamsScript,'orbital');
            obj.a = struct.a;
            obj.inc = struct.inc;
            obj.ecc = struct.ecc;
            obj.raand = struct.raand;
            obj.aop = struct.aop;
            obj.true_anom = struct.true_anom;
            obj.r = struct.r;
            obj.v = struct.v;
        end
    end
end

