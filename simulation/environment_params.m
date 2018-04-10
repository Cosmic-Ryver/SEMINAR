classdef environment_params < sim_component
    
    properties (SetAccess = private, GetAccess = public)
        muE(1,1) double
        muMoon(1,1) double
        rE(1,1) double
        AU(1,1) double
        updateInterval(1,1) double
        b(3,1) double
        rho(1,1) double
        r_sun(3,1) double
        r_moon(3,1) double
    end
    properties (Access = public)
        updateTime(1,1) double
    end
    
    methods
        function obj = environment_params(aOwningSimHandle,...
                aEnvironmentParamsScript)
            obj.pOwningSimHandle = aOwningSimHandle;
            struct = load_parameters(aEnvironmentParamsScript,'environment');
            obj.muE = struct.muE;
            obj.muMoon = struct.muMoon;
            obj.rE = struct.rE;
            obj.AU = struct.AU;
            obj.updateInterval = struct.updateInterval;
            obj.updateTime = struct.updateTime;
        end
        function update(obj, t, x, params)
            jd = t/86400 + params.simulation.jd0;
            obj.b = IGRF(x(1:3), jd, 13,'ECI');
            obj.rho = exponential_atm(x(1:3));
            obj.r_sun = get_r_sun(jd, obj.AU);
            obj.r_moon = get_r_moon(jd, obj.rE);
        end
    end
end

