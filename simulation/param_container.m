classdef param_container < sim_component    
    properties (SetAccess = private, GetAccess = public)
        simulation
        environment
        physical
        orbital
    end
    
    methods
        function obj = param_container(aOwningSimHandle,...
                aSimulationParamSctipt,aEnvironmentParamScript,...
                aPhysicalParamScript,aOrbitalParamScript)
            obj.pOwningSimHandle = aOwningSimHandle;
            obj.simulation = simulation_params(aOwningSimHandle,aSimulationParamSctipt);
            obj.environment = environment_params(aOwningSimHandle,aEnvironmentParamScript);
            obj.environment.updateTime = obj.simulation.ti;
            obj.physical = physical_params(aOwningSimHandle,aPhysicalParamScript);
            obj.orbital = orbital_params(aOwningSimHandle,aOrbitalParamScript);
        end
    end
end

