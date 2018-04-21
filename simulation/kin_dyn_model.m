classdef kin_dyn_model < physics_engine_component
    
    properties
        pModelEnum(1,1) kin_dyn_enumeration
    end
    
    methods
        function obj = kin_dyn_model(aOwningSimHandle,aModelStr)
            obj.pOwningSimHandle = aOwningSimHandle;
            obj.pModelEnum = kin_dyn_enumeration(aModelStr);
            obj.pModelFcn = obj.getFcnHandle();
        end
        
        function oFcnHandle = getFcnHandle(obj)
            switch obj.pModelEnum
                case kin_dyn_enumeration.two_body_dynamics
                    oFcnHandle = @(t,x,varargin) two_body_dynamics(t, x,...
                        obj.pOwningSimHandle.params.environment.muE);
                case kin_dyn_enumeration.lunar_third_body_perturbation
                    oFcnHandle = @(t,x,varargin) third_body_perturbations(...
                        t, x, ...
                        obj.pOwningSimHandle.params.environment.muMoon, ...
                        {@(varargin)...
                        obj.pOwningSimHandle.params.environment.r_moon},...
                        {{}});
                case kin_dyn_enumeration.zonal_harmonics_perturbation
                    oFcnHandle = @(t,x,varargin) ...
                        zonal_harmonics_perturbation(t, x, ...
                        obj.pOwningSimHandle.params.environment.muE, ...
                        obj.pOwningSimHandle.params.environment.rE, 6);
            end
        end
    end
end