function [ estimator ] = truth_estimator()
%% Initial conditions

% preallocate
estimator.estimate.state = zeros(13,1);
estimator.truth.state    = zeros(13,1);

% initial attitude estimate (rad)
ea_est_0   = [pi/4;pi/4;pi/4];

% initial attitude estimate
q_est_0 = ea2quat(ea_est_0);
estimator.estimate.state(7:10) = q_est_0; 

% initial gyro bias estimate (rad/s)
beta_est_0 = zeros(3,1);
estimator.estimate.error = beta_est_0;

% initial attitude error covariance matrix
P_dv_0     = zeros(3);

% initial gyro bias error covariance matrix
P_dbeta_0  = zeros(3);

% Initial error covariance matrix
estimator.estimate.uncertainty = blkdiag(P_dv_0, P_dbeta_0);

% initial attitude error (sampled from initial estimate distribution)
dv_0            = (diag(P_dv_0).^0.5).*randn(3,1);

% initial attitude (error x estimate; normalized)
q_truth_0    = q_est_0 + quat_prod([dv_0/2;0],q_est_0);
q_truth_0    = q_truth_0/norm(q_truth_0);
estimator.truth.state(7:10) = q_truth_0;

% initial angular rates
om_truth_0 = zeros(3,1);
estimator.truth.state(11:13) = om_truth_0;

% initial gyro bias (error + estimate)
estimator.truth.error = beta_est_0 + diag(P_dbeta_0).^0.5.*randn(3,1);

%% Sensor parameters

% timing
rates = [1/60, 1/60]; % Hz
frequencies = rates.^-1; % s
estimator.sensors.base_frequency = min(frequencies);

% star trackers 1 & 2
idx = 1;
estimator.sensors.sensor{idx}.rate = rates(idx);
estimator.sensors.sensor{idx}.frequency = frequencies(idx);
estimator.sensors.sensor{idx}.measure = @(estimator) ...
    {estimator.truth.state(7:10), estimator.truth.state(7:10)};

% gyro
idx = 2;
estimator.sensors.sensor{idx}.rate = rates(idx);
estimator.sensors.sensor{idx}.frequency = frequencies(idx);
estimator.sensors.sensor{idx}.measure = @(estimator) ...
    estimator.truth.state(11:13);

% all
estimator.sensors.measure = @(estimator, t) measure(estimator, t);

%% Truth update and propagation

estimator.truth.update = @(estimator, x_tru) ...
    update_truth(estimator, x_tru, 0);

estimator.estimate.propagate = @(estimator) estimator;

%% Measurement update

% unique updator for each sensor
estimator.estimate.updaters{1}.update = @(estimator) ...
    filter_update(estimator);
estimator.estimate.updaters{2}.update = @(estimator) ...
    gyro_update(estimator);

% all
estimator.estimate.update = @(estimator,t) ...
    update_estimate(estimator, t);

%% Initiallize rate estimate

estimator.sensors.sensor{2}.measurement = ...
    estimator.sensors.sensor{2}.measure(estimator);
estimator = estimator.estimate.updaters{2}.update(estimator);

end

function [ estimator ] = measure(estimator, t)

N = length(estimator.sensors.sensor);
for i = 1:N
    if rem(t,estimator.sensors.sensor{i}.frequency) == 0
        estimator.sensors.sensor{i}.measurement = ...
            estimator.sensors.sensor{i}.measure(estimator);
    end
end

end

function [ estimator ] = update_truth(estimator, x_tru, sigma_u)

base_frequency = estimator.sensors.base_frequency;
estimator.truth.error = estimator.truth.error + ...
    sigma_u*base_frequency^0.5*randn(3,1);
estimator.truth.state = x_tru;

end

function [estimator] = gyro_update(estimator)

estimator.estimate.state(11:13) = estimator.truth.state(11:13);

end

function [estimator] = filter_update(estimator)

% update with truth
estimator.estimate.state(7:10) = estimator.truth.state(7:10);
estimator.estimate.error = estimator.truth.error;
estimator.estimate.uncertainty = zeros(6);
    
end

function [estimator] = update_estimate(estimator, t)

N = length(estimator.sensors.sensor);
for i = 1:N
    if rem(t,estimator.sensors.sensor{i}.frequency) == 0
        estimator = estimator.estimate.updaters{i}.update(estimator);
    end
end

end