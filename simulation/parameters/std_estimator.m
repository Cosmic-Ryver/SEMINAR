function [ estimator ] = std_estimator()
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
beta_est_0 = [0.1/3600; 0.1/3600; 0.1/3600] * pi/180;
estimator.estimate.error = beta_est_0;

% initial attitude error covariance matrix
P_dv_0     = ((6/3600)*(pi/180))^2*eye(3);

% initial gyro bias error covariance matrix
P_dbeta_0  = ((0.02/3600)*(pi/180))^2*eye(3);

% Initial error covariance matrix
estimator.estimate.uncertainty = blkdiag(P_dv_0, P_dbeta_0);

% initial attitude error (sampled from initial estimate distribution)
dv_0            = (diag(P_dv_0).^0.5).*randn(3,1);

% initial attitude (error x estimate; normalized)
q_truth_0    = q_est_0 + quat_prod([dv_0/2;0],q_est_0);
q_truth_0    = q_truth_0/norm(q_truth_0);
estimator.truth.state(7:10) = q_truth_0;

% initial angular rates
om_truth_0 = pi/180*[0.1; 0.1; 0.1];
estimator.truth.state(11:13) = om_truth_0;

% initial gyro bias (error + estimate)
estimator.truth.error = beta_est_0 + diag(P_dbeta_0).^0.5.*randn(3,1);

%% Measurement model

% Measurement model
h = @(m) 0;

% Measurement sensitivity
H = @(m) [eye(3), zeros(3); eye(3), zeros(3)];

%% Sensor parameters

% Star tracker covariance (rad^2) (based on terma T1)
R1 = diag(([1.5 1.5 9.5]*(1/3600*pi/180)).^2);
R2 = diag(([1.5 9.5 1.5]*(1/3600*pi/180)).^2);
R  = blkdiag(R1,R2);

% Gyro rate standard deviation (based on SIRU-E from NG)
% sigma_v = 0.00005/3600*pi/180;
sigma_v = 10^0.5*10^-7;

% Gyro bias standard deviation (based on SIRU-E from NG)
% sigma_u = 0.0005/3600*pi/180;
sigma_u = 10^0.5*10^-10;

% timing
rates = [1, 4]; % Hz
frequencies = rates.^-1; % s
estimator.sensors.base_frequency = min(frequencies);

% star trackers 1 & 2
idx = 1;
estimator.sensors.sensor{idx}.rate = rates(idx);
estimator.sensors.sensor{idx}.frequency = frequencies(idx);
estimator.sensors.sensor{idx}.measure = @(estimator) ...
    { star_tracker(estimator.truth.state(7:10),R1), ...
      star_tracker(estimator.truth.state(7:10),R2) };

% gyro
idx = 2;
estimator.sensors.sensor{idx}.rate = rates(idx);
estimator.sensors.sensor{idx}.frequency = frequencies(idx);
estimator.sensors.sensor{idx}.measure = @(estimator) ...
    biased_gyro(estimator.truth.state(11:13),sigma_v, ...
    estimator.truth.error);

% all
estimator.sensors.measure = @(estimator, t) measure(estimator, t);

%% Covariance dynamics

F = @(om) [-skew(om), -eye(3); zeros(3), zeros(3)];

G = blkdiag(-eye(3),eye(3));

Q = blkdiag(sigma_v^2*eye(3),sigma_u^2*eye(3));

%% Truth update and propagation

estimator.truth.update = @(estimator, x_tru) ...
    update_truth(estimator, x_tru, sigma_u);

estimator.estimate.propagate = @(estimator) ...
    propagate_estimate(estimator,F,G,Q);

%% Measurement update

% unique updator for each sensor
estimator.estimate.updaters{1}.update = @(estimator) ...
    filter_update(estimator, 1, R, h, H);
estimator.estimate.updaters{2}.update = @(estimator) ...
    gyro_update(estimator, 2);

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

function [ estimator ] = propagate_estimate(estimator,F,G,Q)

tspan  = [0 estimator.sensors.base_frequency];
q_est  = estimator.estimate.state(7:10);
om_est = estimator.estimate.state(11:13);
dim    = length(G);
Pr     = @(p) reshape(p,dim,dim);
P      = estimator.estimate.uncertainty;
Xn     = cell(2,1);

for i = 1:2
    if i == 1
        opts  = odeset('RelTol',1e-8,'AbsTol',1e-10);
        [ ~, X ] = ode45(@(t, x, om) 0.5*quat_prod([om; 0],x/norm(x)), ...
            tspan, q_est, opts, om_est);
        Xn{i} = X;
    else
        rP = @(P) reshape(P,dim^2,1);
        P_est = rP(P);
        opts   = odeset('RelTol',1e-10,'AbsTol',1e-13);
        [~,X] = ode45(@(t,x,om) rP(F(om)*Pr(x) + Pr(x)*F(om)' + G*Q*G'),...
                tspan,P_est,opts,om_est);
        Xn{i} = X;
    end
end

estimator.estimate.state(7:10) = Xn{1}(end,:)'/norm(Xn{1}(end,:));
estimator.estimate.uncertainty = Pr(Xn{2}(end,:)');


end

function [estimator] = gyro_update(estimator, sensorIdx)

estimator.estimate.state(11:13) = ...
    estimator.sensors.sensor{sensorIdx}.measurement - ...
    estimator.estimate.error;

end

function [estimator] = filter_update(estimator, sensorIdx, R, h, H)

q_km = estimator.estimate.state(7:10);
beta_km = estimator.estimate.error;
P_km = estimator.estimate.uncertainty;
qy = estimator.sensors.sensor{sensorIdx}.measurement;
nq = length(qy);
y = zeros(3*nq,1);
for i = 1:nq
    dqi = quat_prod(qy{i},quat_inv(q_km));
    yi  = 2 * dqi(1:3)/dqi(4);
    y((1 + (i-1)*3):(i*3)) = yi;
end

[er_k, P_k] = G_EKF_update(zeros(6,1),P_km,y,R,h,H,@(m)eye(6));

% extract errors from error-state vector
dv_k    = er_k(1:3);
dbeta_k = er_k(4:6);

% update attitude quaternion
q_k = q_km + quat_prod([dv_k/2;0],q_km);
q_k = q_k/norm(q_k);

% update gyro bias
beta_k = beta_km + dbeta_k;

% set updated info in struct
estimator.estimate.state(7:10) = q_k;
estimator.estimate.error = beta_k;
estimator.estimate.uncertainty = P_k;
    
end

function [estimator] = update_estimate(estimator, t)

N = length(estimator.sensors.sensor);
for i = 1:N
    if rem(t,estimator.sensors.sensor{i}.frequency) == 0
        estimator = estimator.estimate.updaters{i}.update(estimator);
    end
end

end