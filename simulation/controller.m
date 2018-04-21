classdef controller < sim_component
    properties
        actuators
        control_signal
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
%             obj.actuators.rwheels.config = eye(3);
            N = size(obj.actuators.rwheels.config,2);
            obj.actuators.rwheels.N = N;
            obj.actuators.rwheels.max_torque = 300e-3 + zeros(1,N); % Nm
            obj.actuators.rwheels.Hw = zeros(3,N); % Nms
            obj.actuators.rwheels.max_momentum = 8 + zeros(1,N); % Nms
            obj.actuators.rwheels.actuate = @(varargin) ...
                obj.actuateRWheelArray();
            obj.actuators.rwheels.signal = zeros(3,1);
            
            % initial control signal
            obj.control_signal.q_c = ea2quat([pi;pi*3/8;pi]);
            obj.control_signal.om_c = zeros(3,1);
            obj.control_signal.om_dot_c = zeros(3,1);
            obj.control_signal.h = 1;
        end
        
        function [Lout] = actuateRWheelArray(controller)

            doLoop       = true;
            count        = 0;
            config       = controller.actuators.rwheels.config;
            signal       = controller.actuators.rwheels.signal;
            maxTorque    = controller.actuators.rwheels.max_torque;
            maxMomentum  = controller.actuators.rwheels.max_momentum;
            curMomentum  = reshape(controller.actuators.rwheels.Hw,3,[]);
            N            = size(config,2);
            Lout         = zeros(3,N);
            wheelIdxs    = 1:N;
            curMomentumW = NaN(N,1);
            for i = 1:N
                curMomentumW(i) = norm(curMomentum(:,i))*...
                    sign(dot(curMomentum(:,i),config(:,i)));
            end
            while doLoop
                if count > 1e5
                    error('Infinite loop encountered')
                end
                L = minimax_max(config,signal,curMomentumW,1);
%                 L = config.*repmat(signal',3,1);
                lowRatio = realmax;
                lowi = NaN;
                N    = size(L,2);
                Lmag = NaN(N,1);
                LW   = NaN(N,1);

                % address momentum saturation
                for i = 1:N
                    Lmag(i) = norm(L(:,i));
                    LW(i) = Lmag(i)*sign(dot(L(:,i),config(:,i)));
                    ratio = maxMomentum(i)/abs(LW(i)*...
                        controller.pOwningSimHandle.estimator.updateInterval...
                        + curMomentumW(i));
                    if ratio < lowRatio
                        lowRatio = ratio;
                        lowi = i;
                    end
                end
                if lowRatio < 1
                    % a wheel is saturated, remove from operation
                    maxMomentum(lowi)   = [];
                    config(:,lowi)      = [];
                    curMomentum(:,lowi) = [];
                    curMomentumW(lowi)  = [];
                    maxTorque(lowi)     = [];
                    wheelIdxs(lowi)     = [];
                    if length(wheelIdxs) < 3
                        L = zeros(3,length(wheelIdxs));
                        warning('Wheel momentum is saturated, controller disabled')
                        break
                    end
                    count = count + 1;
                else
                    % no wheels are saturated, break loop
                    doLoop = false;
                end
            end

            % check torque saturation
            lowRatio = realmax;
            for i = 1:size(L,2)
                ratio = maxTorque(i)/Lmag(i);
                if ratio < lowRatio
                    lowRatio = ratio;
                end
            end

            % reduce torque if there is saturation
            if lowRatio < 1 
                L = lowRatio*L;
            end

            % shift torques into the output array
            for i = 1:length(wheelIdxs)
                wheelIdx = wheelIdxs(i);
                Lout(:,wheelIdx) = L(:,i);
            end

        end

        function update(obj)
            [obj.actuators.rwheels.signal, obj.control_signal.h] = ...
                hybrid(obj.pOwningSimHandle.estimator.estimate.state(7:10),...
                obj.pOwningSimHandle.estimator.estimate.state(11:13),...
                obj.pOwningSimHandle.params.physical.J,...
                obj.control_signal.q_c,obj.control_signal.om_c,...
                obj.control_signal.om_dot_c,obj.control_signal.h,0.3,[1, 10],2);
%             obj.actuators.rwheels.signal = sliding_mode_control(...
%                 obj.pOwningSimHandle.estimator.truth.state(7:10),...
%                 obj.pOwningSimHandle.estimator.truth.state(11:13),...
%                 sum(reshape(...
%                 obj.pOwningSimHandle.estimator.truth.state(14:end),...
%                 3,[]),2), obj.pOwningSimHandle.params.physical.J,...
%                 obj.control_signal.q_c,obj.control_signal.om_c,...
%                 obj.control_signal.om_dot_c, 0.3, 0.015*eye(3), ...
%                 0.01+zeros(3,1));
            obj.actuators.rwheels.signal = ...
                -obj.actuators.rwheels.signal;
            obj.actuators.rwheels.Hw     = ...
                obj.pOwningSimHandle.estimator.estimate.Hw;
        end
    end
end