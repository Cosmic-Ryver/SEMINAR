classdef (Abstract) physics_engine_component < sim_component
    properties
        pModelFcn(1,1)
    end
    methods
        getFcnHandle(obj)
        % retrieve a handle to the model
        % function
    end
end

