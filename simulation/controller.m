classdef controller < sim_component
    properties
        actuators
        control_signal
        control_params
        command_mode(1,1) command_mode_enumeration
        targ_dsn(1,1) double
        inDsnReset(1,1) logical = false
        isEstablishingAngAcc(1,1) logical = false
    end
    
    properties (Constant = true)
        % Deep Space Network lat, long, alt (unsure if gd or gc...)
        LLA_dsn = [  34.8668*pi/180, -35.3082*pi/180, 40.4168*pi/180;
                   -117.0430*pi/180, 149.1242*pi/180, -3.7038*pi/180;
                                682,             580,            600];             
    end
    
    methods
        function obj = controller(aOwningSimHandle, aControllerParamScript)

            % set owning sim
            obj.pOwningSimHandle = aOwningSimHandle;
            
            % target DSN site initially NaN, set through changeCommandMode
            obj.targ_dsn = NaN;
            
            % load parameters
            params = load_parameters(aControllerParamScript,'controller');
            
            % reaction wheels
            obj.actuators = params.actuators;
            
            % set actuation function
            obj.actuators.actuate = @(varargin) obj.actuateRWheelArray();
            
            % controller parameters
            obj.control_params = params.control_params;
            
        end
        
        function [Lout] = actuateRWheelArray(controller)

            doLoop       = true;
            count        = 0;
            config       = controller.actuators.rwheels.config;
            signal       = controller.actuators.rwheels.signal;
            kappa        = controller.actuators.rwheels.kappa;
            maxTorque    = controller.actuators.rwheels.max_torque;
            maxMomentum  = controller.actuators.rwheels.max_momentum;
            curMomentum  = reshape(controller.actuators.rwheels.Hw,3,[]);
            N            = size(config,2);
            Lout         = zeros(3,N);
            wheelIdxs    = 1:N;
            
            % get wheel frame momentum
            curMomentumW = NaN(N,1);
            for i = 1:N
                curMomentumW(i) = norm(curMomentum(:,i))*...
                    sign(dot(curMomentum(:,i),config(:,i)));
            end
            
            % Distribute torque within a loop. If a distributed torque
            %   would push a wheel over its momentum capacity, disable that
            %   wheel for this control interval and redistribute. Repeat
            %   until either the torque cant be distributed or a viable
            %   distribution is found.
            while doLoop
                
                % error check
                if count > 1e5
                    error('Infinite loop encountered')
                end
                
                % distribution law
                L = minimax_max(config,signal,curMomentumW,kappa);
%                 L = config.*repmat(signal',3,1); % for 3 wheels only

                % check torque saturation
                lowRatio = realmax;
                N        = size(L,2);
                Lmag     = NaN(N,1);
                for i = 1:N
                    Lmag(i) = norm(L(:,i));
                    ratio   = maxTorque(i)/Lmag(i);
                    if ratio < lowRatio
                        lowRatio = ratio;
                    end
                end

                % reduce torque if there is saturation
                if lowRatio < 1 
                    L = lowRatio*L;
                end
                
                % check momentum saturation
                lowRatio = realmax;
                lowi     = NaN;
                LW       = NaN(N,1);
                for i = 1:N
                    LW(i) = Lmag(i)*sign(dot(L(:,i),config(:,i)));
                    ratio = maxMomentum(i)/abs(LW(i)*...
                        controller.pOwningSimHandle.estimator.updateInterval...
                        + curMomentumW(i));
                    if ratio < lowRatio
                        lowRatio = ratio;
                        lowi = i;
                    end
                end
                
                % Break loop if no wheels are saturated
                if lowRatio >= 1
                    break
                end
                
                % remove saturated wheel from operation, then redistribute
                maxMomentum(lowi)   = [];
                config(:,lowi)      = [];
                curMomentum(:,lowi) = [];
                curMomentumW(lowi)  = [];
                maxTorque(lowi)     = [];
                wheelIdxs(lowi)     = [];
                
                % 3-axis control not possible
                if length(wheelIdxs) < 3
                    L = zeros(3,length(wheelIdxs));
                    warning('Wheel momentum is saturated, controller disabled')
                    break
                end
                
                % increment counter for error checking
                count = count + 1;
            end

            % shift torques into the output array
            for i = 1:length(wheelIdxs)
                wheelIdx = wheelIdxs(i);
                Lout(:,wheelIdx) = L(:,i);
            end
        end

        function update(obj)
            
            % command logic
            obj.commandLogic();
            
            % control law
            [obj.actuators.rwheels.signal, obj.control_signal.h] = ...
                hybrid(obj.pOwningSimHandle.estimator.estimate.state(7:10),...
                obj.pOwningSimHandle.estimator.estimate.state(11:13),...
                obj.pOwningSimHandle.params.physical.J,...
                obj.control_signal.q_c,obj.control_signal.om_c,...
                obj.control_signal.om_dot_c,obj.control_signal.h,...
                obj.control_params.delta,obj.control_params.kp,...
                obj.control_params.kd);
            
            % wheels act as internal torques: invert sign
            obj.actuators.rwheels.signal = ...
                -obj.actuators.rwheels.signal;
            
            % controller fetches momentum estimate from estimator
            obj.actuators.rwheels.Hw     = ...
                obj.pOwningSimHandle.estimator.estimate.Hw;
        end
        
        function initiallizeCommandMode(aObj)
            aObj.changeCommandMode(...
                aObj.pOwningSimHandle.params.simulation.command_mode, ...
                aObj.pOwningSimHandle.params.simulation.q_c);
        end
        
        function changeCommandMode(obj, aCommandMode, varargin)
            
            % set command mode
            obj.command_mode = aCommandMode;
            
            % mode switch
            switch aCommandMode
                case command_mode_enumeration.pointing
                    % Set desired pointing direction and null the rates
                    obj.control_signal.q_c      = varargin{1};
                    obj.control_signal.om_c     = zeros(3,1);
                    obj.control_signal.om_dot_c = zeros(3,1);
                    obj.control_signal.h        = 1;
                case command_mode_enumeration.dsn_tracking
                    
                    % prep angular acc
                    obj.isEstablishingAngAcc = true;
                    obj.control_signal.om_dot_c = zeros(3,1);
                    
                    % initialize the hysteresis variable
                    obj.control_signal.h = 1;
                    
                    % get JD
                    jd = obj.pOwningSimHandle.estimator.estimate.t/86400 ...
                        + obj.pOwningSimHandle.params.simulation.jd0;
                    r_SC_ECI = obj.pOwningSimHandle.estimator.estimate.state(1:3);
                    v_SC_ECI = obj.pOwningSimHandle.estimator.estimate.state(4:6);
                    
                    % transform r_SC into ECEF frame
                    CTM_ECI2ECEF = ECI2ECEF(jd);
                    r_SC_ECEF    = CTM_ECI2ECEF*r_SC_ECI;
                    
                    % find az el for each DSN site
                    AzEl_dsn     = zeros(2,3);
                    r_dsn_ECEF   = zeros(3,3);
                    r_SCoDSN_ECEF   = zeros(3,3);
                    obj.targ_dsn = NaN;
                    for i = 1:3
                        r_dsn_ECEF(:,i) = LLA2ECEF(obj.LLA_dsn(:,i), ...
                            obj.pOwningSimHandle.params.environment.rE);
                        
                        r_SCoDSN_ECEF(:,i) = r_SC_ECEF - r_dsn_ECEF(:,i);
                        
                        CTM_ECEF2SEZ = ea2CTM([0; pi/2 - obj.LLA_dsn(1,i); obj.LLA_dsn(2,i)]);
                        
                        r_SC_SEZ = CTM_ECEF2SEZ*r_SCoDSN_ECEF(:,i);
                        
                        AzEl_dsn(1,i) = atan2(r_SC_SEZ(2),-r_SC_SEZ(1));
                        AzEl_dsn(2,i) = asin(r_SC_SEZ(3)/norm(r_SC_SEZ));
                        
                        % break early if site is ideal (assumes satellite
                        %   is orbiting in direction of Earth rotation, or 
                        %   orbit is polar)
                        if AzEl_dsn(2,i) > 10*pi/180
                            if AzEl_dsn(1,i) >= 0
                                obj.targ_dsn = i;
                                break
                            end
                        end
                    end
                    
                    % no ideal site, find one that is sufficient
                    if isnan(obj.targ_dsn)
                        obj.targ_dsn = find(AzEl_dsn(2,:) > 10*pi/180);
                        if length(obj.targ_dsn) > 1
                            obj.targ_dsn = obj.targ_dsn(1);
                        elseif isempty(obj.targ_dsn)
                            error('No valid DSN target site found')
                        end
                    end
                    
                    % determine an initial pointing direction
                    q_ANT2B = obj.pOwningSimHandle.params.orbital.q_ANT2B;
                    h = cross(r_SC_ECI,v_SC_ECI);
                    hhat = h/norm(h);
                    CTM_ECEF2ECI = CTM_ECI2ECEF';
                    r_SCoDSN_ECI = CTM_ECEF2ECI*r_SCoDSN_ECEF(:,obj.targ_dsn);
                    rhat = r_SCoDSN_ECI/norm(r_SCoDSN_ECI);
                    o3 = -rhat;
                    o2 = cross(hhat,rhat);
                    o1 = cross(o2,o3);
                    CTM_ANT2ECI = [o1, o2, o3];
                    CTM_ECI2ANT = CTM_ANT2ECI';
                    q_ECI2ANT = CTM2quat(CTM_ECI2ANT);
                    q_ECI2B  = quat_prod(q_ANT2B,q_ECI2ANT);
                    obj.control_signal.q_c = q_ECI2B;
                    
                    % determine an initial angular rotation rate
                    r_dsn_ECI = CTM_ECEF2ECI*r_dsn_ECEF(:,i);
                    vmag_dsn_ECI = 2*pi*norm(r_dsn_ECI(1:2))/86164.1;
                    vhat_dsn_ECI = cross(-r_dsn_ECI/norm(r_dsn_ECI),...
                        [1;0;0]);
                    v_dsn_ECI = vmag_dsn_ECI*vhat_dsn_ECI;
                    v_DSNoSC_ECI = v_dsn_ECI - v_SC_ECI;
                    r_DSNoSC_ECI = -r_SCoDSN_ECI;
                    om_DSNoSC_ECI = cross(r_DSNoSC_ECI,v_DSNoSC_ECI)/...
                        norm(r_DSNoSC_ECI)^2;
                    om_BoECI_ECI = om_DSNoSC_ECI;
                    om_BoECI_B = quat2CTM(q_ECI2B)*om_BoECI_ECI;
                    obj.control_signal.om_c = om_BoECI_B;
                    
                case command_mode_enumeration.nadir_pointing
                    error('Not implemented')
            end
        end
        
        function commandLogic(obj)
            
            % mode switch
            switch obj.command_mode
                case command_mode_enumeration.pointing
                    % nothing to do...                    
                case command_mode_enumeration.dsn_tracking
                    
                    if obj.isEstablishingAngAcc
                        % skip one step to initiallize the angular
                        %   acceleration
                        
                        % set flag
                        obj.isEstablishingAngAcc = false;
                        
                        % fetch the current time from the estimator
                        obj.control_signal.t = ...
                            obj.pOwningSimHandle.estimator.estimate.t;
                        
                    else
                        
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % confirm that the DSN site is still visible
                        
                        % get JD
                        t = obj.pOwningSimHandle.estimator.estimate.t;
                        jd = t/86400 ...
                            + obj.pOwningSimHandle.params.simulation.jd0;
                        r_SC_ECI = obj.pOwningSimHandle.estimator.estimate.state(1:3);
                        v_SC_ECI = obj.pOwningSimHandle.estimator.estimate.state(4:6);

                        % transform r_SC into ECEF frame
                        CTM_ECI2ECEF = ECI2ECEF(jd);
                        r_SC_ECEF    = CTM_ECI2ECEF*r_SC_ECI;

                        % find el for DSN site
                        r_dsn_ECEF = LLA2ECEF(obj.LLA_dsn(:,obj.targ_dsn), ...
                            obj.pOwningSimHandle.params.environment.rE);

                        r_SCoDSN_ECEF = r_SC_ECEF - r_dsn_ECEF;

                        CTM_ECEF2SEZ = ea2CTM([0; ...
                            pi/2 - obj.LLA_dsn(1,obj.targ_dsn); ...
                            obj.LLA_dsn(2,obj.targ_dsn)]);

                        r_SC_SEZ = CTM_ECEF2SEZ*r_SCoDSN_ECEF;
                        
                        El_dsn = asin(r_SC_SEZ(3)/norm(r_SC_SEZ));

                        % if target site isn't visible: reset command mode
                        %   and call 
                        if El_dsn < 10*pi/180
                            if ~obj.inDsnReset
                                obj.inDsnReset = true;
                                obj.changeCommandMode(...
                                    command_mode_enumeration.dsn_tracking);
                                obj.commandLogic();
                                obj.inDsnReset = false;
                                return
                            else
                                error('Failled to find valid DSN site after reset')
                            end
                        end
                        
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % compute control signal
                        
                        % determine an initial pointing direction
                        q_ANT2B = obj.pOwningSimHandle.params.orbital.q_ANT2B;
                        h = cross(r_SC_ECI,v_SC_ECI);
                        hhat = h/norm(h);
                        CTM_ECEF2ECI = CTM_ECI2ECEF';
                        r_SCoDSN_ECI = CTM_ECEF2ECI*r_SCoDSN_ECEF;
                        rhat = r_SCoDSN_ECI/norm(r_SCoDSN_ECI);
                        o3 = -rhat;
                        o2 = cross(hhat,rhat);
                        o1 = cross(o2,o3);
                        CTM_ANT2ECI = [o1, o2, o3];
                        CTM_ECI2ANT = CTM_ANT2ECI';
                        q_ECI2ANT = CTM2quat(CTM_ECI2ANT);
                        q_ECI2B  = quat_prod(q_ANT2B,q_ECI2ANT);
                        obj.control_signal.q_c = q_ECI2B;

                        % determine an initial angular rotation rate
                        r_dsn_ECI = CTM_ECEF2ECI*r_dsn_ECEF;
                        vmag_dsn_ECI = 2*pi*norm(r_dsn_ECI(1:2))/86164.1;
                        vhat_dsn_ECI = cross(-r_dsn_ECI/norm(r_dsn_ECI),...
                            [1;0;0]);
                        v_dsn_ECI = vmag_dsn_ECI*vhat_dsn_ECI;
                        v_DSNoSC_ECI = v_dsn_ECI - v_SC_ECI;
                        r_DSNoSC_ECI = -r_SCoDSN_ECI;
                        om_DSNoSC_ECI = cross(r_DSNoSC_ECI,v_DSNoSC_ECI)/...
                            norm(r_DSNoSC_ECI)^2;
                        om_BoECI_ECI = om_DSNoSC_ECI;
                        om_BoECI_B = quat2CTM(q_ECI2B)*om_BoECI_ECI;
                        
                        % calculate angular acceleration as a finite
                        %   difference
                        tm = obj.control_signal.t;
                        omm = obj.control_signal.om_c;
                        obj.control_signal.om_dot_c = (om_BoECI_B - omm)/...
                            (t - tm);
                        
                        % set angular rate
                        obj.control_signal.om_c = om_BoECI_B;
                        
                        % set time
                        obj.control_signal.t = t;
                        
                    end
                    
                case command_mode_enumeration.nadir_pointing
                    error('Not implemented')
            end
        end
    end
end