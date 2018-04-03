%% HEADER
%
% EXAMPLE_MEKF_1
%
% Example problem for testing the multiplicitive EKF for on-orbit attitude
% estimation
%
% Gus Buonviri, 3/16/18
% Mississippi State University
%
%
%% Clean workspace

clc; clear all; close all;

%% Initial conditions

% random seed
rng(595246290)

% progress bar
progressbar('Filtering','Measurement generation','Truth generation')

% time
nH = 1;
ti = 0;
dt = 0.5;
tf = nH*3600;
tv = ti:dt:tf;
N  = length(tv);

% initial attitude estimate (rad)
ea_est_0   = [pi/4;pi/4;pi/4];

% initial attitude estimate
q_est_0    = ea2quat(ea_est_0);

% initial gyro bias estimate (rad/s)
beta_est_0 = [0.1/3600; 0.1/3600; 0.1/3600] * pi/180;

% initial attitude error covariance matrix
P_dv_0     = ((6/3600)*(pi/180))^2*eye(3);

% initial gyro bias error covariance matrix
P_dbeta_0  = ((0.02/3600)*(pi/180))^2*eye(3);

% Initial error covariance matrix
P_0        = blkdiag(P_dv_0, P_dbeta_0);

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

%% Dynamics model

% % Discrete time attitude kinematics
% a_func = @(dt,om_est_mag) cos(0.5*om_est_mag*dt);
% psi_est_func = @(dt,om_est,om_est_mag)...
%     om_est * sin(0.5*om_est_mag*dt)/om_est_mag;
% f_sub = @(dt,a,psi_est)...
%     [a*eye(3) - skew(psi_est), psi_est; psi_est', a];
% f = @(quat,dt,om_est,om_est_mag,varargin)...
%     f_sub(dt,a_func(dt,om_est_mag),psi_est_func(dt,om_est,om_est_mag))*...
%     quat;
% 
% % Discrete time error dynamics Jacobian 
% Phi_er = @(m,dt,om_est,om_est_mag,om_est_skw)...
%     [eye(3) - om_est_skw*sin(om_est_mag*dt)/om_est_mag + ...
%      om_est_skw^2*(1-cos(om_est_mag*dt))/om_est_mag^2, ...
%      om_est_skw*(1-cos(om_est_mag*dt))/om_est_mag^2 - eye(3)*dt - ...
%      om_est_skw^2*(om_est_mag*dt - sin(om_est_mag*dt))/om_est_mag^3;
%      zeros(3),eye(3)];
% 
% % Discrete time error process noise covariance
% Q = @(m,dt)...
%     [(sigma_v^2*dt + (1/3)*sigma_u^2*dt^3)*eye(3),...
%                      -(0.5*sigma_u^2*dt^2)*eye(3);
%                      -(0.5*sigma_u^2*dt^2)*eye(3),...
%                             (sigma_u^2*dt)*eye(3)];

F = @(om) [-skew(om), -eye(3); zeros(3), zeros(3)];

G = blkdiag(-eye(3),eye(3));

Q = blkdiag(sigma_v^2*eye(3),sigma_u^2*eye(3));

Pr = @(p) reshape(p,6,6);

rP = @(P) reshape(P,36,1);

%% Truth propagation

% solver options
options1 = odeset('RelTol',1e-8,'AbsTol',1e-10);
options2 = odeset('RelTol',1e-15,'AbsTol',1e-17);

% gyro sampling
fq_om = 30; % (Hz)
dt_om = 1/fq_om; % (s)
tv_om = ti:dt_om:tf;
N_om  = length(tv_om);

% preallocate
q_truth         = NaN(4,N);
om_truth_vec    = NaN(3,N);
beta_truth      = NaN(3,N_om);

% initial attitude error (sampled from initial estimate distribution)
dv_0            = (diag(P_dv_0).^0.5).*randn(3,1);

% initial attitude (error x estimate; normalized)
q_truth(:,1)    = q_est_0 + quat_prod([dv_0/2;0],q_est_0);
q_truth(:,1)    = q_truth(:,1)/norm(q_truth(:,1));

% angular rate function
om_truth = @(t) 0.1*pi/180*[sin(pi/3600*t); cos(pi/3600*t); sin(0.25*pi+pi/3600*t)];

% angular rates
om_truth_vec(:,1)   = om_truth(0);

% initial gyro bias error (sampled from initial estimate distribution)
dbeta_0         = diag(P_dbeta_0).^0.5.*randn(3,1);

% initial gyro bias (error + estimate)
beta_truth(:,1) = beta_est_0 + dbeta_0;

% span for the interval
tspan = [tv(1) tv(2)];

for j = 2:(dt*fq_om)
    % bias change modeled as random walk
    beta_truth(:,j) = beta_truth(:,j-1) + sigma_u*dt_om^0.5*randn(3,1);
end

for i = 2:N
    % time
    t_i = tv(i);
    
    % span for the interval
    tspan = [tv(i-1) tv(i)];
    
    % angular rates
    om_truth_vec(:,i) = om_truth(tv(i));
    
    % integrate quaternion kinematics
    [~,X] = ode45(@(t,x,om) 0.5*quat_prod([om(t); 0],x), tspan, q_truth(:,i-1), options1, om_truth);
    q_truth(:,i) = X(end,:)';
    
    for j = ((t_i-dt)*fq_om + 1):((t_i)*fq_om + 1)
        % bias change modeled as random walk
        beta_truth(:,j) = beta_truth(:,j-1) + sigma_u*dt_om^0.5*randn(3,1);
    end
    
    % progressbar update
    progressbar(0,0,i/N);
end

%% Measurement model

% Measurement model
h = @(m) 0;

% Measurement sensitivity
H = @(m) [eye(3), zeros(3); eye(3), zeros(3)];

%% Measurement generation

% Pre-allocate
q_y1 = NaN(4,N);
q_y2 = NaN(4,N);
om_y = NaN(3,N_om);

for i = 1:N
    % time
    t_i = tv(i);
    
    % Attitude error (noise)
    dv1_i = (diag(R1).^0.5).*randn(3,1);
    dv2_i = (diag(R2).^0.5).*randn(3,1);
    
    % Attitude measurement (error x truth; normalized)
    q_y1(:,i) = q_truth(:,i) + quat_prod([dv1_i/2;0],q_truth(:,i));
    q_y1(:,i) = q_y1(:,i)/norm(q_y1(:,i));
    q_y2(:,i) = q_truth(:,i) + quat_prod([dv2_i/2;0],q_truth(:,i));
    q_y2(:,i) = q_y2(:,i)/norm(q_y2(:,i));
    
    for j = (t_i*fq_om + 1):min(((t_i + dt)*fq_om),N_om)
        % Angular rate measurement error (noise + bias)
        dom_i = sigma_v*randn(3,1) + beta_truth(:,j);
    
        % Angular rate measurement (truth + error)
        om_y(:,j) = om_truth(tv_om(j)) + dom_i;
    end
    
    % progressbar update
    progressbar(0,i/N,[]);
end

%% Measurement plotting

figure('Name','Quaternion Measurements')
hold on
title('Quaternion Measurements')
scatter(tv,q_y1(1,:),'r')
scatter(tv,q_y1(2,:),'g')
scatter(tv,q_y1(3,:),'b')
scatter(tv,q_y1(4,:),'k')
plot(tv,q_truth(1,:),'y',tv,q_truth(2,:),'c',tv,q_truth(3,:),'m',tv,q_truth(4,:),'w')
grid on
xlabel('Time (s)')
ylabel('Quaternion')
hold off

% figure('Name','Gyro Measurements')
% hold on
% title('Gyro Measurements')
% scatter(tv,om_y(1,:),'r')
% scatter(tv,om_y(2,:),'g')
% scatter(tv,om_y(3,:),'b')
% plot(tv,om_truth_vec(1,:),'y',tv,om_truth_vec(2,:),'c',tv,om_truth_vec(3,:),'m')
% grid on
% xlabel('Time (s)')
% ylabel('Angular Rate (rad/s)')
% hold off

%% Simulation

% preallocate
q_est    = NaN(4,N);
beta_est = NaN(3,N);
om_est   = NaN(3,N);
P        = NaN(6,6,N);

% initial conditions
q_est(:,1)    = q_est_0;
beta_est(:,1) = beta_est_0;
om_est(:,1)   = om_y(:,1) - beta_est(:,1);
P(:,:,1)      = P_0;

% loop
for k = 2:N
    
    % step variables
    km1          = k-1;
    tspan        = [tv(km1) tv(k)];
    dt           = diff(tspan);
    q_est_km1    = q_est(:,km1);
    P_km1        = P(:,:,km1);
    om_est_km1   = om_est(:,km1);
    beta_est_km1 = beta_est(:,km1);
    beta_est_km  = beta_est_km1;
    q_y1_k       = q_y1(:,k);
    q_y2_k       = q_y2(:,k);
    om_y_k       = om_y(:,(tv(km1)*fq_om + 1):(tv(k)*fq_om + 1));
    
    % propagation
    [~,X] = ode45(@(t,x,om,beta)...
        0.5*quat_prod([om(:,floor((t - dt*floor(t/dt))/dt_om) + 1) - beta; 0],x),...
        tspan,q_est_km1,options1,om_y_k,beta_est_km1);
    q_est_km = X(end,:)';
    [~,X] = ode45(@(t,x,om,beta)...
        rP(F(om(:,floor((t - dt*floor(t/dt))/dt_om) + 1) - beta)*Pr(x) + ...
        Pr(x)*F(om(:,floor((t - dt*floor(t/dt))/dt_om) + 1) - beta)' + G*Q*G'),...
        tspan,rP(P_km1),options2,om_y_k,beta_est_km1);
    P_km = Pr(X(end,:)');
    
    % get attitude error measurement from attitude measurement
    dq_y1 = quat_prod(q_y1_k,quat_inv(q_est_km));
    y1    = 2 * dq_y1(1:3)/dq_y1(4);
    dq_y2 = quat_prod(q_y2_k,quat_inv(q_est_km));
    y2    = 2 * dq_y2(1:3)/dq_y2(4);
    y     = [y1; y2];
    
    % update
    [er_k, P_k] = G_EKF_update(zeros(6,1),P_km,y,R,h,H,@(m)eye(6));
    
    % extract errors from error-state vector
    dv_k    = er_k(1:3);
    dbeta_k = er_k(4:6);
    
    % update attitude quaternion
    q_est_k = q_est_km + quat_prod([dv_k/2;0],q_est_km);
    q_est_k = q_est_k/norm(q_est_k);
    
    % update gyro bias
    beta_est_k = beta_est_km + dbeta_k;
    
    % update rate estimate
    om_est_k = om_y_k(:,end) - beta_est_k;
    
    % store updated variables
    q_est(:,k)    = q_est_k;
    beta_est(:,k) = beta_est_k;
    om_est(:,k)   = om_est_k;
    P(:,:,k)      = P_k;
    
    % progressbar update
    progressbar(k/N,[],[]);
end

%% Plotting

% preallocate
ea_truth = zeros(3,N);
ea_est   = zeros(3,N);
A_truth  = zeros(3,3,N);
A_est    = zeros(3,3,N);
ang_er   = zeros(1,N);

% clean up and assign variables
for i = 1:N
    q_truth(:,i)   = quat_pq4(q_truth(:,i));
    q_est(:,i)     = quat_pq4(q_est(:,i));
    q_y1(:,i)       = quat_pq4(q_y1(:,i));
    ea_truth(:,i)  = quat2ea(q_truth(:,i));
    ea_est(:,i)    = quat2ea(q_est(:,i));
    A_truth(:,:,i) = quat2CTM(q_truth(:,i));
    A_est(:,:,i)   = quat2CTM(q_est(:,i));
    r_truth        = A_truth(:,:,i) * [1;0;0];
    r_est          = A_est(:,:,i) * [1;0;0];
    ang_er(i)      = acos(dot(r_truth,r_est)/(norm(r_truth)*norm(r_est)));
end
tv = tv/3600;

figure('Name','Attitude Quaternion Estimate')
hold on
title('Attitude Quaternion Estimate')
plot(tv,zeros(1,N),'k')
plot(tv,q_truth(1,:),'y-',tv,q_truth(2,:),'c-',tv,q_truth(3,:),'m-',tv,q_truth(4,:),'w-')
plot(tv,q_est(1,:),'r--',tv,q_est(2,:),'g--',tv,q_est(3,:),'b--',tv,q_est(4,:),'k--')
grid on
xlabel('Time (h)')
ylabel('Quaternion Element Value')
ylim([-1, 1])
hold off

figure('Name','Roll Error')
hold on
title('Roll Error')
plot(tv,(ea_truth(1,:) - ea_est(1,:))*1e6)
plot(tv,zeros(1,N),'k')
sigma = permute(P(1,1,:).^0.5*1e6,[1 3 2]);
plot(tv,3*sigma.*ones(1,N),'--')
plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angle (urad)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
hold off

figure('Name','Pitch Error')
hold on
title('Pitch Error')
plot(tv,(ea_truth(2,:) - ea_est(2,:))*1e6)
plot(tv,zeros(1,N),'k')
sigma = permute(P(2,2,:).^0.5*1e6,[1 3 2]);
plot(tv,3*sigma.*ones(1,N),'--')
plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angle (urad)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
hold off

figure('Name','Yaw Error')
hold on
title('Yaw Error')
plot(tv,(ea_truth(3,:) - ea_est(3,:))*1e6)
plot(tv,zeros(1,N),'k')
sigma = permute(P(3,3,:).^0.5*1e6,[1 3 2]);
plot(tv,3*sigma.*ones(1,N),'--')
plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angle (urad)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
hold off

figure('Name','Total Angular Error')
hold on
title('Total Angular Error')
plot(tv,ang_er*180/pi*3600)
xlabel('Time (h)')
ylabel('Angle (arcsec)')
grid on
hold off

figure('Name','Gyro Bias Estimate')
hold on
title('Gyro Bias Estimate')
plot(tv,zeros(1,N),'k')
plot(tv,180/pi*beta_truth(1,1:fq_om*dt:end)*3600,'y-',tv,180/pi*beta_truth(2,1:fq_om*dt:end)*3600,'c-',tv,180/pi*beta_truth(3,1:fq_om*dt:end)*3600,'m-')
plot(tv,180/pi*beta_est(1,:)*3600,'r--',tv,180/pi*beta_est(2,:)*3600,'g--',tv,180/pi*beta_est(3,:)*3600,'b--')
xlabel('Time (h)')
ylabel('Bias (deg/h)')
grid on
hold off

figure('Name','Bias 1 Error')
hold on
title('Bias 1 Error')
plot(tv,(beta_truth(1,1:fq_om*dt:end) - beta_est(1,:))*3600*180/pi)
plot(tv,zeros(1,N),'k')
sigma = permute(P(4,4,:).^0.5*180/pi*3600,[1 3 2]);
plot(tv,3*sigma.*ones(1,N),'--')
plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
hold off

figure('Name','Bias 2 Error')
hold on
title('Bias 2 Error')
plot(tv,(beta_truth(2,1:fq_om*dt:end) - beta_est(2,:))*3600*180/pi)
plot(tv,zeros(1,N),'k')
sigma = permute(P(5,5,:).^0.5*180/pi*3600,[1 3 2]);
plot(tv,3*sigma.*ones(1,N),'--')
plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
hold off

figure('Name','Bias 3 Error')
hold on
title('Bias 3 Error')
plot(tv,(beta_truth(3,1:fq_om*dt:end) - beta_est(3,:))*3600*180/pi)
plot(tv,zeros(1,N),'k')
sigma = permute(P(6,6,:).^0.5*180/pi*3600,[1 3 2]);
plot(tv,3*sigma.*ones(1,N),'--')
plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
hold off

figure('Name','Angular Rate Estimate')
hold on
title('Angular Rate Estimate')
plot(tv,zeros(1,N),'k')
plot(tv,180/pi*om_truth_vec(1,:)*3600,'y-',tv,180/pi*om_truth_vec(2,:)*3600,'c-',tv,180/pi*om_truth_vec(3,:)*3600,'m-')
plot(tv,180/pi*om_est(1,:)*3600,'r--',tv,180/pi*om_est(2,:)*3600,'g--',tv,180/pi*om_est(3,:)*3600,'b--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
grid on
hold off

figure('Name','Rate 1 Error')
hold on
title('Rate 1 Error')
plot(tv,(om_truth_vec(1,:) - om_est(1,:))*3600*180/pi)
plot(tv,zeros(1,N),'k')
sigma = sigma_v*180/pi*3600;
plot(tv,3*sigma.*ones(1,N),'--')
plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
hold off

figure('Name','Rate 2 Error')
hold on
title('Rate 2 Error')
plot(tv,(om_truth_vec(2,:) - om_est(2,:))*3600*180/pi)
plot(tv,zeros(1,N),'k')
sigma = sigma_v*180/pi*3600;
plot(tv,3*sigma.*ones(1,N),'--')
plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
hold off

figure('Name','Rate 3 Error')
hold on
title('Rate 3 Error')
plot(tv,(om_truth_vec(3,:) - om_est(3,:))*3600*180/pi)
plot(tv,zeros(1,N),'k')
sigma = sigma_v*180/pi*3600;
plot(tv,3*sigma.*ones(1,N),'--')
plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
hold off