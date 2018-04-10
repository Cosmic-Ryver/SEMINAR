function [controller] = std_controller()

controller.actuators.actuate = @(controller, x) ...
    controller.actuators.rwheels.actuate(controller, x);
controller.actuators.rwheels.config = ...
    [0.5^0.5, -0.5^0.5,       0,        0;
     0.5^0.5,  0.5^0.5, 0.5^0.5,  0.5^0.5;
           0,        0, 0.5^0.5, -0.5^0.5];
controller.actuators.rwheels.max_torque = ...
    [300, 300, 300, 300]*1e-3; % Nm
controller.actuators.rwheels.N = ...
    size(controller.actuators.rwheels.config,2);
controller.actuators.rwheels.position = ...
    [];
controller.actuators.rwheels.initial_momentum = zeros(3,4); % Nms
controller.actuators.rwheels.max_momentum = 8 + zeros(1,4); % Nms
controller.actuators.rwheels.actuate = @(controller, x) ...
    actuate(controller, x);
controller.control_signal.q_c = ea2quat([pi;pi*3/8;pi]);
controller.control_signal.om_c = zeros(3,1);
controller.control_signal.om_dot_c = zeros(3,1);
controller.updateInterval = 0.25;
controller.update = @(controller,estimator,params) update(controller,...
    estimator,params);

end

function [Lout] = actuate(controller,x)
% Momentum should really be sensed by the estimator, and passed into this
% function that way, instead of passing in the state. Should fix this
% later...
doLoop       = true;
count        = 0;
config       = controller.actuators.rwheels.config;
signal       = controller.actuators.rwheels.signal;
maxTorque    = controller.actuators.rwheels.max_torque;
maxMomentum  = controller.actuators.rwheels.max_momentum;
curMomentum  = reshape(x(14:end),3,[]);
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
    lowRatio = realmax;
    lowi = NaN;
    N    = size(L,2);
    Lmag = NaN(N,1);
    LW   = NaN(N,1);
    
    % address momentum saturation
    for i = 1:N
        Lmag(i) = norm(L(:,i));
        LW(i) = Lmag(i)*sign(dot(L(:,i),config(:,i)));
        ratio = maxMomentum(i)/abs(LW(i)*controller.updateInterval...
            + curMomentumW(i));
        if ratio < lowRatio
            lowRatio = ratio;
            lowi = i;
        end
    end
    if lowRatio < 1
        % wheel is saturated, remove from operation
        maxMomentum(lowi)  = [];
        config(:,lowi)     = [];
        curMomentum(lowi)  = [];
        curMomentumW(lowi) = [];
        maxTorque(lowi)    = [];
        wheelIdxs(lowi)    = [];
        if length(wheelIdxs) < 3
            L = zeros(3,length(wheelIdxs));
            warning('Wheel momentum is saturated, controller disabled')
            break
        end
        count = count + 1;
        continue
    end
    
    % address torque saturation
    lowRatio = realmax;
    for i = 1:size(L,2)
        ratio = maxTorque(i)/Lmag(i);
        if ratio < lowRatio
            lowRatio = ratio;
        end
    end
    
    if lowRatio >= 1 % break loop if no wheel torque violates the constraints
        doLoop = false;
    else
        signal = lowRatio*signal;
        count = count + 1;
    end
end

% shift torques into the output array
for i = 1:length(wheelIdxs)
    wheelIdx = wheelIdxs(i);
    Lout(:,wheelIdx) = L(:,i);
end
end

function [controller] = update(controller,estimator,params)
controller.actuators.rwheels.signal = ...
    -hybrid(estimator.estimate.state(7:10),...
    estimator.estimate.state(11:13),params.physical.J,...
    controller.control_signal.q_c,controller.control_signal.om_c,...
    controller.control_signal.om_dot_c,1,1);
controller.updateInterval = estimator.updateInterval;
end