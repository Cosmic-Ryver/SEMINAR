classdef force_torq_model < physics_engine_component
    
    properties
        pModelEnum(1,1) force_torq_enumeration
    end
    
    methods
        function obj = force_torq_model(aOwningSimHandle,aModelStr)
            obj.pOwningSimHandle = aOwningSimHandle;
            obj.pModelEnum = force_torq_enumeration(aModelStr);
            obj.pModelFcn = obj.getFcnHandle();
        end
        
        function oFcnHandle = getFcnHandle(obj)
            switch obj.pModelEnum
                case force_torq_enumeration.solar_rad_pressure
                    oFcnHandle = @(t,x,params) rp_perturbations(t, x, ...
                        params.physical.surface_model, ...
                        @(varargin)solar_rad_press(x(1:3),t/86400 + ...
                        params.simulation.jd0, ...
                        params.environment.r_sun), {});
                case force_torq_enumeration.aero_perturbations
                    oFcnHandle = @(t,x,params) aero_perturbations(t, x, ...
                        params.physical.Cd, ...
                        params.physical.surface_model, ...
                        @(varargin)params.environment.rho, {});
                case force_torq_enumeration.magnetic_perturbation
                    oFcnHandle = @(t,x,params) magnetic_perturbation( ...
                        t, x, params.physical.res_dipole, ...
                        @(varargin) ...
                        quat2CTM(x(7:10))*params.environment.b, {} );
                case force_torq_enumeration.gravity_gradient_effect
                    oFcnHandle = @(t,x,params) grav_grad_perturbation( ...
                        t, x, params.environment.muE, params.physical.J);
            end
        end
    end
end