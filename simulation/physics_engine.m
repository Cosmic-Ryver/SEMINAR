classdef physics_engine < sim_component
    
    properties (SetAccess = private, GetAccess = public)
        pForceTorqModels cell
        pKinDynModels cell
        pForceTorqFcns cell
        pKinDynFcns cell
        pOdeFcn
    end
    
    methods
        function obj = physics_engine(aOwningSimHandle, ...
                aPhysicsEngineScript)
            % Assign sim handle
            obj.pOwningSimHandle = aOwningSimHandle;
            
            % Read in data from script
            strs = load_parameters(aPhysicsEngineScript,'physics_engine');
            forceTorqModelStrs = strs{1};
            kinDynModelStrs = strs{2};
            
            % Create force/torque model objects
            nFTM = length(forceTorqModelStrs);
            obj.pForceTorqModels = cell(nFTM,1);
            obj.pForceTorqFcns  = cell(nFTM,1);
            for i = 1:nFTM
                obj.pForceTorqModels{i} = force_torq_model(...
                    aOwningSimHandle,forceTorqModelStrs{i});
                obj.pForceTorqFcns{i}  = ...
                    obj.pForceTorqModels{i}.pModelFcn;  
            end
            
            % Create kinematics/dynamics model objects
            nKDM = length(kinDynModelStrs);
            obj.pKinDynModels = cell(nKDM,1);
            obj.pKinDynFcns  = cell(nKDM,1);
            for i = 1:nKDM
                obj.pKinDynModels{i} = kin_dyn_model(...
                    aOwningSimHandle,kinDynModelStrs{i});
                obj.pKinDynFcns{i}  = obj.pKinDynModels{i}.pModelFcn;  
            end
            
            % Establish ODE Fcn property
            obj.pOdeFcn = obj.engine();
        end
        
        function oFcnHandle = engine(obj)
            oFcnHandle = @(t,x,controller,params) kin_dyn_superposition(...
                t, x, params.physical.m, params.physical.J, ...
                controller.actuators.actuate(controller,x), ...
                obj.pForceTorqFcns, {{params}}, obj.pKinDynFcns, ...
                {{params}});
        end
    end
end

