classdef controller < sim_component
    properties
        actuators
        control_signal
        control_params
    end
    
    methods
        function obj = controller(aOwningSimHandle)

            % set owning sim
            obj.pOwningSimHandle = aOwningSimHandle;
            
            % set actuation function
            obj.actuators.actuate = @(varargin) ...
                obj.actuators.rwheels.actuate();
            
            % reaction wheels
            obj.actuators.rwheels.config = ...
                [0.5^0.5, -0.5^0.5,       0,        0;
                 0.5^0.5,  0.5^0.5, 0.5^0.5,  0.5^0.5;
                       0,        0, 0.5^0.5, -0.5^0.5];
%             obj.actuators.rwheels.config = eye(3); % 3-wheel system
            N = size(obj.actuators.rwheels.config,2);
            obj.actuators.rwheels.N = N;
            obj.actuators.rwheels.max_torque = 300e-3 + zeros(1,N); % Nm
            obj.actuators.rwheels.Hw = zeros(3,N); % Nms
            obj.actuators.rwheels.max_momentum = 8 + zeros(1,N); % Nms
            obj.actuators.rwheels.actuate = @(varargin) ...
                obj.actuateRWheelArray();
            obj.actuators.rwheels.signal = zeros(3,1);
            obj.actuators.rwheels.kappa = 1; % minimax feedback gain
            
            % initial control signal
            obj.control_signal.q_c = ea2quat([pi;pi*3/8;pi]);
            obj.control_signal.om_c = zeros(3,1);
            obj.control_signal.om_dot_c = zeros(3,1);
            obj.control_signal.h = 1;
            
            % controller parameters
            obj.control_params.delta = 0.3;     % hysteresis half length
            obj.control_params.kp    = [1, 10]; % proportional gain
            obj.control_params.kd    = 2;       % derivative gain
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
    end
end