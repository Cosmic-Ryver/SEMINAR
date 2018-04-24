% reaction wheels
actuators.rwheels.config = ...
    [0.5^0.5, -0.5^0.5,       0,        0;
     0.5^0.5,  0.5^0.5, 0.5^0.5,  0.5^0.5;
           0,        0, 0.5^0.5, -0.5^0.5];
N = size(actuators.rwheels.config,2);
actuators.rwheels.N = N;
actuators.rwheels.max_torque = 300e-3 + zeros(1,N); % Nm
actuators.rwheels.Hw = zeros(3,N); % Nms
actuators.rwheels.max_momentum = 8 + zeros(1,N); % Nms
actuators.rwheels.signal = zeros(3,1);
actuators.rwheels.kappa = 1; % minimax feedback gain

% controller parameters
control_params.delta = 0.3;     % hysteresis half length
control_params.kp    = [1, 10]; % proportional gain
control_params.kd    = 5;       % derivative gain