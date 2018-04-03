%% HEADER
%
% Main script for simulation execution.
%
% Gus Buonviri, 3/5/2018
% Mississippi State University
%
%
%% Clean workspace

clc; clear all; close all;

%% LOAD PARAMETERS
%
% All major simulation parameters should be specified in their own version
% controlled scripts. The relevant variables from these scripts should be
% loaded into the simulation via the appropriate functions.

%%%% PARAM SCRIPT NAMES %%%%
orbital_param_script     = 'test_orbital_params.m';
physical_param_script    = 'test_physical_params.m';
simulation_param_script  = 'test_simulation_params.m';
environment_param_script = 'test_environment_params';
kin_dyn_engine_script    = 'test_kin_dyn_engine';
estimator_script         = 'test_estimator';
controller_script        = 'test_controller';
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% load params and primary components
params.orbital     = load_parameters(orbital_param_script,'orbital');
params.physical    = load_parameters(physical_param_script,'physical');
params.simulation  = load_parameters(simulation_param_script,'simulation');
params.environment = load_parameters(environment_param_script,'environment');
kin_dyn_engine     = load_parameters(kin_dyn_engine_script,'kin_dyn_engine');
estimator          = load_parameters(estimator_script,'estimator');
controller         = load_parameters(controller_script,'controller');

% determine timestep needed
dt = estimator.sensors.base_frequency;
ti = params.simulation.ti;
te = params.simulation.te;
tv = ti:dt:te;
N  = length(tv);
N_data = params.simulation.N_data;

% preallocate
x        = zeros(16,N_data);
x_est    = zeros(13,N_data);
beta     = zeros(3,N_data);
beta_est = zeros(3,N_data);
L_ctr    = zeros(3,N_data);
qy1      = zeros(4,N_data);
qy2      = zeros(4,N_data);
omy      = zeros(3,N_data);

% initial conditions
x(1:3,1)       = params.orbital.r;
x(4:6,1)       = params.orbital.v;
x(7:10,1)      = estimator.truth.state(7:10);
x(11:13,1)     = estimator.truth.state(11:13);
x(14:16,1)     = [0;0;0];
x_est(7:10,1)  = estimator.estimate.state(7:10);
x_est(11:13,1) = estimator.estimate.state(11:13);
beta(:,1)      = estimator.truth.error;
beta_est(:,1)  = estimator.estimate.error;
x1             = x(:,1);
x_est1         = x_est(:,1);

% initialize environment parameters
params = params.environment.update(tv(1), x1, params);

% progress bar
progressbar('Simulation');

% main loop
for i = 2:N

    % shift state and estimate variables
    x0     = x1;
    x_est0 = x_est1;
    
    % time & span
    t = tv(i);
    tspan = [tv(i-1) tv(i)];
    
    % update controller
    controller = controller.update(controller,estimator,params);
    
    % propagate truth
    [~, X] = ode45(kin_dyn_engine, tspan, x0, params.simulation.odeOpts,...
        controller, params);
    x1 = X(end,:)';
    x1(7:10) = x1(7:10)/norm(x1(7:10));
    
    % propagate estimate
    estimator = estimator.estimate.propagate(estimator);
    
    % update estimator truth
    estimator = estimator.truth.update(estimator, x1);
    
    % generate measurements
    estimator = estimator.sensors.measure(estimator, t);
    
    % update estimate
    estimator = estimator.estimate.update(estimator, t);
    
    % retrieve estimate
    x_est1(7:10)  = estimator.estimate.state(7:10);
    x_est1(11:13) = estimator.estimate.state(11:13);
    
    % lowest frequency events
    if mod(t,params.simulation.dt_data) == 0
        % store date
        idx = (t - tv(1))/params.simulation.dt_data + 1;
        x(:,idx) = x1;
        x_est(:,idx) = x_est1;
        qy12 = estimator.sensors.sensor{1}.measurement;
        qy1(:,idx) = qy12{1};
        qy2(:,idx) = qy12{2};
        omy(:,idx) = estimator.sensors.sensor{2}.measurement;
        L_ctr(:,idx) = controller.actuator(controller);
        
        %update environment
        params = params.environment.update(t, x1, params);
        
        % update progress bar
        progressbar(t/te);
    end
end

%% plotting

% preallocate
r_truth  = zeros(3,N_data);
v_truth  = zeros(3,N_data);
q_truth  = zeros(4,N_data);
q_est    = zeros(4,N_data);
om_truth = zeros(3,N_data);
om_est   = zeros(3,N_data);
ea_truth = zeros(3,N_data);
ea_est   = zeros(3,N_data);
A_truth  = zeros(3,3,N_data);
A_est    = zeros(3,3,N_data);
ang_er   = zeros(1,N_data);

% clean up and assign variables
for i = 1:N_data
    r_truth(:,i)   = x(1:3,i);
    v_truth(:,i)   = x(4:6,i);
    q_truth(:,i)   = quat_pq4(x(7:10,i));
    q_est(:,i)     = quat_pq4(x_est(7:10,i));
    ea_truth(:,i)  = quat2ea(q_truth(:,i));
    ea_est(:,i)    = quat2ea(q_est(:,i));
    om_truth(:,i)  = x(11:13,i);
    om_est(:,i)    = x_est(11:13,i);
    A_truth(:,:,i) = quat2CTM(q_truth(:,i));
    A_est(:,:,i)   = quat2CTM(q_est(:,i));
    point_truth    = A_truth(:,:,i) * [1;0;0];
    point_est      = A_est(:,:,i) * [1;0;0];
    ang_er(i)      = acos(dot(point_truth,point_est)/...
        (norm(point_truth)*norm(point_est)));
end
tv_data = params.simulation.tv_data/3600;

figure('Name','Orbit')
hold on
title('Orbit')
plot3(r_truth(1,:),r_truth(2,:),r_truth(3,:));
[Xe, Ye, Ze] = sphere(35);
surf(Xe*6378000,Ye*6378000,Ze*6378000,ones(36))
hold off

figure('Name','Velocity')
hold on
title('Velocity')
plot(tv_data,zeros(1,N_data),'k')
plot(tv_data,v_truth(1,:),'r-',tv_data,v_truth(2,:),'g-',tv_data,v_truth(3,:),'b-')
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
xlabel('Time (h)')
ylabel('Velocity (m/s)')
hold off

figure('Name','Attitude Quaternion Estimate')
hold on
title('Attitude Quaternion Estimate')
plot(tv_data,zeros(1,N_data),'k')
plot(tv_data,q_truth(1,:),'y-',tv_data,q_truth(2,:),'c-',tv_data,q_truth(3,:),'m-',tv_data,q_truth(4,:),'w-')
plot(tv_data,q_est(1,:),'r--',tv_data,q_est(2,:),'g--',tv_data,q_est(3,:),'b--',tv_data,q_est(4,:),'k--')
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
xlabel('Time (h)')
ylabel('Quaternion Element Value')
ylim([-1, 1])
hold off

figure('Name','Roll Error')
hold on
title('Roll Error')
plot(tv_data,(ea_truth(1,:) - ea_est(1,:))*1e6)
plot(tv_data,zeros(1,N_data),'k')
% sigma = permute(P(1,1,:).^0.5*1e6,[1 3 2]);
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angle (urad)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Pitch Error')
hold on
title('Pitch Error')
plot(tv_data,(ea_truth(2,:) - ea_est(2,:))*1e6)
plot(tv_data,zeros(1,N_data),'k')
% sigma = permute(P(2,2,:).^0.5*1e6,[1 3 2]);
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angle (urad)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Yaw Error')
hold on
title('Yaw Error')
plot(tv_data,(ea_truth(3,:) - ea_est(3,:))*1e6)
plot(tv_data,zeros(1,N_data),'k')
% sigma = permute(P(3,3,:).^0.5*1e6,[1 3 2]);
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angle (urad)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Total Angular Error')
hold on
title('Total Angular Error')
plot(tv_data,ang_er*180/pi*3600)
xlabel('Time (h)')
ylabel('Angle (arcsec)')
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

% figure('Name','Gyro Bias Estimate')
% hold on
% title('Gyro Bias Estimate')
% plot(tv,zeros(1,N),'k')
% plot(tv,180/pi*beta_truth(1,1:fq_om*dt:end)*3600,'y-',tv,180/pi*beta_truth(2,1:fq_om*dt:end)*3600,'c-',tv,180/pi*beta_truth(3,1:fq_om*dt:end)*3600,'m-')
% plot(tv,180/pi*beta_est(1,:)*3600,'r--',tv,180/pi*beta_est(2,:)*3600,'g--',tv,180/pi*beta_est(3,:)*3600,'b--')
% xlabel('Time (h)')
% ylabel('Bias (deg/h)')
% grid on
% hold off

% figure('Name','Bias 1 Error')
% hold on
% title('Bias 1 Error')
% plot(tv,(beta_truth(1,1:fq_om*dt:end) - beta_est(1,:))*3600*180/pi)
% plot(tv,zeros(1,N),'k')
% sigma = permute(P(4,4,:).^0.5*180/pi*3600,[1 3 2]);
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
% xlabel('Time (h)')
% ylabel('Angular Rate (deg/h)')
% % ylim(mean(abs(sigma))*[-4 4])
% grid on
% hold off
% 
% figure('Name','Bias 2 Error')
% hold on
% title('Bias 2 Error')
% plot(tv,(beta_truth(2,1:fq_om*dt:end) - beta_est(2,:))*3600*180/pi)
% plot(tv,zeros(1,N),'k')
% sigma = permute(P(5,5,:).^0.5*180/pi*3600,[1 3 2]);
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
% xlabel('Time (h)')
% ylabel('Angular Rate (deg/h)')
% % ylim(mean(abs(sigma))*[-4 4])
% grid on
% hold off
% 
% figure('Name','Bias 3 Error')
% hold on
% title('Bias 3 Error')
% plot(tv,(beta_truth(3,1:fq_om*dt:end) - beta_est(3,:))*3600*180/pi)
% plot(tv,zeros(1,N),'k')
% sigma = permute(P(6,6,:).^0.5*180/pi*3600,[1 3 2]);
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
% xlabel('Time (h)')
% ylabel('Angular Rate (deg/h)')
% % ylim(mean(abs(sigma))*[-4 4])
% grid on
% hold off

figure('Name','Angular Rate Estimate')
hold on
title('Angular Rate Estimate')
plot(tv_data,zeros(1,N_data),'k')
plot(tv_data,180/pi*om_truth(1,:)*3600,'y-',tv_data,180/pi*om_truth(2,:)*3600,'c-',tv_data,180/pi*om_truth(3,:)*3600,'m-')
plot(tv_data,180/pi*om_est(1,:)*3600,'r--',tv_data,180/pi*om_est(2,:)*3600,'g--',tv_data,180/pi*om_est(3,:)*3600,'b--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Rate 1 Error')
hold on
title('Rate 1 Error')
plot(tv_data,(om_truth(1,:) - om_est(1,:))*3600*180/pi)
plot(tv_data,zeros(1,N_data),'k')
% sigma = sigma_v*180/pi*3600;
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Rate 2 Error')
hold on
title('Rate 2 Error')
plot(tv_data,(om_truth(2,:) - om_est(2,:))*3600*180/pi)
plot(tv_data,zeros(1,N_data),'k')
% sigma = sigma_v*180/pi*3600;
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Rate 3 Error')
hold on
title('Rate 3 Error')
plot(tv_data,(om_truth(3,:) - om_est(3,:))*3600*180/pi)
plot(tv_data,zeros(1,N_data),'k')
% sigma = sigma_v*180/pi*3600;
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off