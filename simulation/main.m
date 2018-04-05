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
environment_param_script = 'std_environment_params';
kin_dyn_engine_script    = 'std_kin_dyn_engine';
estimator_script         = 'std_estimator';
controller_script        = 'null_controller';
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
x          = zeros(16,N_data);
x_est      = zeros(13,N_data);
beta_truth = zeros(3,N_data);
beta_est   = zeros(3,N_data);
L_ctr      = zeros(3,N_data);
qy1        = zeros(4,N_data);
qy2        = zeros(4,N_data);
omy        = zeros(3,N_data);
L_int      = zeros(3,N_data);
nP         = length(estimator.estimate.uncertainty);
P          = zeros(nP,nP,N_data);
q_c        = zeros(4,N_data);

% initial conditions
x(1:3,1)        = params.orbital.r;
x(4:6,1)        = params.orbital.v;
x(7:10,1)       = estimator.truth.state(7:10);
x(11:13,1)      = estimator.truth.state(11:13);
x(14:16,1)      = [0;0;0];
x_est(7:10,1)   = estimator.estimate.state(7:10);
x_est(11:13,1)  = estimator.estimate.state(11:13);
beta_truth(:,1) = estimator.truth.error;
beta_est(:,1)   = estimator.estimate.error;
x1              = x(:,1);
x_est1          = x_est(:,1);

% initialize environment parameters
params = params.environment.update(tv(1), x1, params);

% progress bar
progressbar('Simulation');

% main loop
for i = 2:N

    % shift state variable
    x0     = x1;
    
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
    
    % lowest frequency events
    if mod(t,params.simulation.dt_data) == 0
        % store date
        idx = (t - tv(1))/params.simulation.dt_data + 1;
        x(:,idx) = x1;
        x_est(7:10,idx) = estimator.estimate.state(7:10);
        x_est(11:13,idx) = estimator.estimate.state(11:13);
        beta_truth(:,idx) = estimator.truth.error;
        beta_est(:,idx) = estimator.estimate.error;
        P(:,:,idx) = estimator.estimate.uncertainty;
        L_int(:,idx) = sum(controller.actuator(controller),2);
        qy12 = estimator.sensors.sensor{1}.measurement;
        qy1(:,idx) = qy12{1};
        qy2(:,idx) = qy12{2};
        omy(:,idx) = estimator.sensors.sensor{2}.measurement;
        L_ctr(:,idx) = controller.actuator(controller);
        q_c(:,idx) = controller.control_signal;
        
        %update environment
        params = params.environment.update(t, x1, params);
        
        % update progress bar
        progressbar(t/te);
    end
end

%% plotting

% preallocate
r_truth    = zeros(3,N_data);
v_truth    = zeros(3,N_data);
q_truth    = zeros(4,N_data);
q_est      = zeros(4,N_data);
dq_c       = zeros(4,N_data);
om_truth   = zeros(3,N_data);
om_est     = zeros(3,N_data);
ea_truth   = zeros(3,N_data);
ea_est     = zeros(3,N_data);
A_truth    = zeros(3,3,N_data);
A_est      = zeros(3,3,N_data);
A_c        = zeros(3,3,N_data);
est_ang_er = zeros(1,N_data);
c_ang_er   = zeros(1,N_data);

% clean up and assign variables
for i = 1:N_data
    r_truth(:,i)   = x(1:3,i);
    v_truth(:,i)   = x(4:6,i);
    q_truth(:,i)   = quat_pq4(x(7:10,i));
    q_est(:,i)     = quat_pq4(x_est(7:10,i));
    dq_c(:,i)      = quat_pq4(quat_prod(q_c(:,i),quat_inv(q_truth(:,i))));
    ea_truth(:,i)  = quat2ea(q_truth(:,i));
    ea_est(:,i)    = quat2ea(q_est(:,i));
    om_truth(:,i)  = x(11:13,i);
    om_est(:,i)    = x_est(11:13,i);
    A_truth(:,:,i) = quat2CTM(q_truth(:,i));
    A_c(:,:,i)     = quat2CTM(q_c(:,i));
    A_est(:,:,i)   = quat2CTM(q_est(:,i));
    point_truth    = A_truth(:,:,i) * [1;0;0];
    point_c        = A_c(:,:,i) * [1;0;0];
    point_est      = A_est(:,:,i) * [1;0;0];
    est_ang_er(i)  = acos(dot(point_truth,point_est)/...
        (norm(point_truth)*norm(point_est)));
    c_ang_er(i)    = acos(dot(point_truth,point_c)/...
        (norm(point_truth)*norm(point_c)));
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
plot(tv_data,v_truth(1,:),'r-','DisplayName','X Axis')
plot(tv_data,v_truth(2,:),'g-','DisplayName','Y Axis')
plot(tv_data,v_truth(3,:),'b-','DisplayName','Z Axis')
legend('Location','northeast')
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
xlabel('Time (h)')
ylabel('Velocity (m/s)')
hold off

figure('Name','Attitude Quaternion','Units','normalized','Position',[5/8 0.5 3/8 1/3])
hold on
title('Attitude Quaternion')
plot(tv_data,q_truth(1,:),'r-','DisplayName','q1 truth');
plot(tv_data,q_truth(2,:),'g-','DisplayName','q2 truth');
plot(tv_data,q_truth(3,:),'b-','DisplayName','q3 truth');
plot(tv_data,q_truth(4,:),'k-','DisplayName','q4 truth');
plot(tv_data,q_est(1,:),'c--','DisplayName','q1 est');
plot(tv_data,q_est(2,:),'y--','DisplayName','q2 est');
plot(tv_data,q_est(3,:),'m--','DisplayName','q3 est');
plot(tv_data,q_est(4,:),'w--','DisplayName','q4 est');
lgd = legend('Location','eastoutside');
set(lgd,'Color',[0.9; 0.9; 0.9]);
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
xlabel('Time (h)')
ylabel('Quaternion Element Value')
ylim([-1, 1])
hold off

figure('Name','Error Quaternion Vector')
hold on
title('Error Quaternion Vector')
plot(tv_data,dq_c(1,:),'r-','DisplayName','q1');
plot(tv_data,dq_c(2,:),'g-','DisplayName','q2');
plot(tv_data,dq_c(3,:),'b-','DisplayName','q3');
lgd = legend('Location','northeast');
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
xlabel('Time (h)')
ylabel('Quaternion Element Value')
ylim([-1, 1])
hold off

figure('Name','Error Quaternion Scalar')
hold on
title('Error Quaternion Scalar')
plot(tv_data,dq_c(4,:),'k-');
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
xlabel('Time (h)')
ylabel('Quaternion Element Value')
ylim([0, 1])
hold off

figure('Name','Pointing Error')
hold on
title('Pointing Error')
plot(tv_data,c_ang_er*180/pi*3600)
xlabel('Time (h)')
ylabel('Angle (arcsec)')
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Angular Rate','Units','normalized','Position',[5/8 0.5 3/8 1/3])
hold on
title('Angular Rate')
plot(tv_data,180/pi*om_truth(1,:)*3600,'r-','DisplayName','X Truth');
plot(tv_data,180/pi*om_truth(2,:)*3600,'g-','DisplayName','Y Truth');
plot(tv_data,180/pi*om_truth(3,:)*3600,'b-','DisplayName','Z Truth');
plot(tv_data,180/pi*om_est(1,:)*3600,'c--','DisplayName','q1 Est');
plot(tv_data,180/pi*om_est(2,:)*3600,'y--','DisplayName','q1 Est');
plot(tv_data,180/pi*om_est(3,:)*3600,'m--','DisplayName','q1 Est');
legend('Location','eastoutside')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Control Torques')
hold on
title('Control Torques')
plot(tv_data,L_int(1,:),'r-','DisplayName','Roll axis');
plot(tv_data,L_int(2,:),'g-','DisplayName','Pitch axis');
plot(tv_data,L_int(3,:),'b-','DisplayName','Yaw axis');
legend('Location','northeast')
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
xlabel('Time (h)')
ylabel('Torque (N*m)')
hold off

figure('Name','Roll Estimate Error')
hold on
title('Roll Estimate Error')
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

figure('Name','Pitch Estimate Error')
hold on
title('Pitch Estimate Error')
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

figure('Name','Yaw Estimate Error')
hold on
title('Yaw Estimate Error')
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

figure('Name','Pointing Estimate Error')
hold on
title('Pointing Estimate Error')
plot(tv_data,est_ang_er*180/pi*3600)
xlabel('Time (h)')
ylabel('Angle (arcsec)')
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Rate Estimate 1 Error')
hold on
title('Rate Estimate 1 Error')
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

figure('Name','Rate Estimate 2 Error')
hold on
title('Rate Estimate 2 Error')
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

figure('Name','Rate Estimate 3 Error')
hold on
title('Rate Estimate 3 Error')
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

figure('Name','Gyro Bias Estimate','Units','normalized','Position',[5/8 0.5 3/8 1/3])
hold on
title('Gyro Bias')
plot(tv_data,180/pi*beta_truth(1,:)*3600,'r-','DisplayName','X Truth')
plot(tv_data,180/pi*beta_truth(2,:)*3600,'g-','DisplayName','Y Truth')
plot(tv_data,180/pi*beta_truth(3,:)*3600,'b-','DisplayName','Z Truth')
plot(tv_data,180/pi*beta_est(1,:)*3600,'c--','DisplayName','X Est')
plot(tv_data,180/pi*beta_est(2,:)*3600,'y--','DisplayName','Y Est')
plot(tv_data,180/pi*beta_est(3,:)*3600,'m--','DisplayName','Z Est')
legend('Location','eastoutside')
xlabel('Time (h)')
ylabel('Bias (deg/h)')
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Bias 1 Error')
hold on
title('Bias 1 Error')
plot(tv_data,(beta_truth(1,:) - beta_est(1,:))*3600*180/pi)
% sigma = permute(P(4,4,:).^0.5*180/pi*3600,[1 3 2]);
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Bias 2 Error')
hold on
title('Bias 2 Error')
plot(tv_data,(beta_truth(2,:) - beta_est(2,:))*3600*180/pi)
% sigma = permute(P(5,5,:).^0.5*180/pi*3600,[1 3 2]);
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off

figure('Name','Bias 3 Error')
hold on
title('Bias 3 Error')
plot(tv_data,(beta_truth(3,:) - beta_est(3,:))*3600*180/pi)
% sigma = permute(P(6,6,:).^0.5*180/pi*3600,[1 3 2]);
% plot(tv,3*sigma.*ones(1,N),'--')
% plot(tv,-3*sigma.*ones(1,N),'--')
xlabel('Time (h)')
ylabel('Angular Rate (deg/h)')
% ylim(mean(abs(sigma))*[-4 4])
grid on
set(gca,'Color',[0.9; 0.9; 0.9]);
hold off