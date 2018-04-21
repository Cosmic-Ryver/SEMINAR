classdef attitude_simulation < handle
    
    properties (SetAccess = private, GetAccess = public)
        params
        physics_engine
        estimator
        controller
        initialState
        data_store
        pStepSize
    end
    
    methods
        function sim = attitude_simulation( simulation_param_script, ...
            orbital_param_script, physical_param_script, ...
            environment_param_script, physics_engine_script )
        
            % fundamental parameters
            sim.params = param_container(sim, simulation_param_script, ...
                environment_param_script, physical_param_script, ...
                orbital_param_script);
            
            % controller
            sim.controller     = controller(sim);
            
            % physics engine
            sim.physics_engine = physics_engine(sim, physics_engine_script);
            
            % Initial state
            sim.initialState(1:3)        = sim.params.orbital.r;
            sim.initialState(4:6)        = sim.params.orbital.v;
            sim.initialState(7:10)       = sim.params.orbital.q;
            sim.initialState(11:13)      = sim.params.orbital.om;
            for i = 1:sim.controller.actuators.rwheels.N
                sim.initialState((14 + (i-1)*3):(13 + i*3)) = ...
                    sim.controller.actuators.rwheels.Hw(:,i);
            end
            sim.initialState = sim.initialState';
            
            % estimator
            sim.estimator      = estimator(sim);
            
            % update environmental conditions
            sim.params.environment.update(0,sim.initialState);
            
%             % update controller
%             sim.controller = sim.controller.update(sim.controller,...
%                 sim.estimator,sim.params);
            

            initializeDataStore(sim);
            
            sim.pStepSize = min([sim.params.environment.updateInterval,...
                sim.estimator.updateInterval,sim.data_store.updateInterval]);
        end
        
        function setControlSignal(obj,q_c,om_c,om_dot_c)
            obj.controller.control_signal.q_c = q_c;
            obj.controller.control_signal.om_c = om_c;
            obj.controller.control_signal.om_dot_c = om_dot_c;
        end
        
        function plotDataStore(obj)
            figure('Name','Orbit')
            hold on
            title('Orbit')
            plot3(obj.data_store.r_truth(1,:),obj.data_store.r_truth(2,:),obj.data_store.r_truth(3,:));
            [Xe, Ye, Ze] = sphere(35);
            surf(Xe*6378000,Ye*6378000,Ze*6378000,ones(36))
            hold off

            figure('Name','Velocity')
            hold on
            title('Velocity')
            plot(obj.data_store.tv,obj.data_store.v_truth(1,:),'r-','DisplayName','X Axis')
            plot(obj.data_store.tv,obj.data_store.v_truth(2,:),'g-','DisplayName','Y Axis')
            plot(obj.data_store.tv,obj.data_store.v_truth(3,:),'b-','DisplayName','Z Axis')
            legend('Location','northeast')
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            xlabel('Time (h)')
            ylabel('Velocity (m/s)')
            hold off

            figure('Name','Attitude Quaternion','Units','normalized','Position',[5/8 0.5 3/8 1/3])
            hold on
            title('Attitude Quaternion')
            plot(obj.data_store.tv,obj.data_store.q_truth(1,:),'r-','DisplayName','q1 truth');
            plot(obj.data_store.tv,obj.data_store.q_truth(2,:),'g-','DisplayName','q2 truth');
            plot(obj.data_store.tv,obj.data_store.q_truth(3,:),'b-','DisplayName','q3 truth');
            plot(obj.data_store.tv,obj.data_store.q_truth(4,:),'k-','DisplayName','q4 truth');
            plot(obj.data_store.tv,obj.data_store.q_est(1,:),'c--','DisplayName','q1 est');
            plot(obj.data_store.tv,obj.data_store.q_est(2,:),'y--','DisplayName','q2 est');
            plot(obj.data_store.tv,obj.data_store.q_est(3,:),'m--','DisplayName','q3 est');
            plot(obj.data_store.tv,obj.data_store.q_est(4,:),'w--','DisplayName','q4 est');
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
            plot(obj.data_store.tv,obj.data_store.dq_c(1,:),'r-','DisplayName','q1');
            plot(obj.data_store.tv,obj.data_store.dq_c(2,:),'g-','DisplayName','q2');
            plot(obj.data_store.tv,obj.data_store.dq_c(3,:),'b-','DisplayName','q3');
            legend('Location','northeast');
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            xlabel('Time (h)')
            ylabel('Quaternion Element Value')
            ylim([-1, 1])
            hold off

            figure('Name','Error Quaternion Scalar')
            hold on
            title('Error Quaternion Scalar')
            plot(obj.data_store.tv,obj.data_store.dq_c(4,:),'k-');
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            xlabel('Time (h)')
            ylabel('Quaternion Element Value')
            ylim([0, 1])
            hold off

            figure('Name','Pointing Error')
            hold on
            title('Pointing Error')
            plot(obj.data_store.tv,obj.data_store.c_ang_er*180/pi*3600)
            xlabel('Time (h)')
            ylabel('Angle (arcsec)')
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            hold off
            
            figure('Name','Roll Pointing Error')
            hold on
            title('Roll Pointing Error')
            plot(obj.data_store.tv,2*obj.data_store.dq_c(1,:)*1e6)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
            xlabel('Time (h)')
            ylabel('Angle (urad)')
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            hold off

            figure('Name','Pitch Pointing Error')
            hold on
            title('Pitch Pointing Error')
            plot(obj.data_store.tv,2*obj.data_store.dq_c(2,:)*1e6)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
            xlabel('Time (h)')
            ylabel('Angle (urad)')
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            hold off

            figure('Name','Yaw Pointing Error')
            hold on
            title('Yaw Pointing Error')
            plot(obj.data_store.tv,2*obj.data_store.dq_c(3,:)*1e6)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
            xlabel('Time (h)')
            ylabel('Angle (urad)')
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            hold off

            figure('Name','Angular Rate','Units','normalized','Position',[5/8 0.5 3/8 1/3])
            hold on
            title('Angular Rate')
            plot(obj.data_store.tv,180/pi*obj.data_store.om_truth(1,:)*3600,'r-','DisplayName','X Truth');
            plot(obj.data_store.tv,180/pi*obj.data_store.om_truth(2,:)*3600,'g-','DisplayName','Y Truth');
            plot(obj.data_store.tv,180/pi*obj.data_store.om_truth(3,:)*3600,'b-','DisplayName','Z Truth');
            plot(obj.data_store.tv,180/pi*obj.data_store.om_est(1,:)*3600,'c--','DisplayName','q1 Est');
            plot(obj.data_store.tv,180/pi*obj.data_store.om_est(2,:)*3600,'y--','DisplayName','q1 Est');
            plot(obj.data_store.tv,180/pi*obj.data_store.om_est(3,:)*3600,'m--','DisplayName','q1 Est');
            legend('Location','eastoutside')
            xlabel('Time (h)')
            ylabel('Angular Rate (deg/h)')
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            hold off

            figure('Name','Control Torques')
            hold on
            title('Control Torques')
            plot(obj.data_store.tv,obj.data_store.L_ctr(1,:),'r-','DisplayName','Roll axis');
            plot(obj.data_store.tv,obj.data_store.L_ctr(2,:),'g-','DisplayName','Pitch axis');
            plot(obj.data_store.tv,obj.data_store.L_ctr(3,:),'b-','DisplayName','Yaw axis');
            legend('Location','northeast')
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            xlabel('Time (h)')
            ylabel('Torque (N*m)')
            hold off

            figure('Name','Wheel Momentum')
            hold on
            title('Wheel Momentum')
            for i = 1:size(obj.controller.actuators.rwheels.config,2)
                dispName = sprintf('Wheel %d',i);
                plot(obj.data_store.tv,obj.data_store.WheelMomentum(i,:),'DisplayName',dispName);
            end
            legend('Location','northeast')
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            xlabel('Time (h)')
            ylabel('Angular Momentum (N*m*s)')
            hold off
            
            figure('Name','Wheel Torques')
            hold on
            title('Wheel Torques')
            for i = 1:size(obj.controller.actuators.rwheels.config,2)
                dispName = sprintf('Wheel %d',i);
                plot(obj.data_store.tv,obj.data_store.WheelTorque(i,:),'DisplayName',dispName);
            end
            legend('Location','northeast')
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            xlabel('Time (h)')
            ylabel('Torque (N*m)')
            hold off
            

            figure('Name','Roll Estimate Error')
            hold on
            title('Roll Estimate Error')
            plot(obj.data_store.tv,2*obj.data_store.dq_est(1,:)*1e6)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
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
            plot(obj.data_store.tv,2*obj.data_store.dq_est(2,:)*1e6)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
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
            plot(obj.data_store.tv,2*obj.data_store.dq_est(3,:)*1e6)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
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
            plot(obj.data_store.tv,obj.data_store.est_ang_er*180/pi*3600)
            xlabel('Time (h)')
            ylabel('Angle (arcsec)')
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            hold off

            figure('Name','Rate Estimate 1 Error')
            hold on
            title('Rate Estimate 1 Error')
            plot(obj.data_store.tv,(obj.data_store.om_truth(1,:) - obj.data_store.om_est(1,:))*3600*180/pi)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
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
            plot(obj.data_store.tv,(obj.data_store.om_truth(2,:) - obj.data_store.om_est(2,:))*3600*180/pi)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
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
            plot(obj.data_store.tv,(obj.data_store.om_truth(3,:) - obj.data_store.om_est(3,:))*3600*180/pi)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
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
            plot(obj.data_store.tv,180/pi*obj.data_store.beta_truth(1,:)*3600,'r-','DisplayName','X Truth')
            plot(obj.data_store.tv,180/pi*obj.data_store.beta_truth(2,:)*3600,'g-','DisplayName','Y Truth')
            plot(obj.data_store.tv,180/pi*obj.data_store.beta_truth(3,:)*3600,'b-','DisplayName','Z Truth')
            plot(obj.data_store.tv,180/pi*obj.data_store.beta_est(1,:)*3600,'c--','DisplayName','X Est')
            plot(obj.data_store.tv,180/pi*obj.data_store.beta_est(2,:)*3600,'y--','DisplayName','Y Est')
            plot(obj.data_store.tv,180/pi*obj.data_store.beta_est(3,:)*3600,'m--','DisplayName','Z Est')
            legend('Location','eastoutside')
            xlabel('Time (h)')
            ylabel('Bias (deg/h)')
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            hold off

            figure('Name','Bias 1 Error')
            hold on
            title('Bias 1 Error')
            plot(obj.data_store.tv,(obj.data_store.beta_truth(1,:) - obj.data_store.beta_est(1,:))*3600*180/pi)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
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
            plot(obj.data_store.tv,(obj.data_store.beta_truth(2,:) - obj.data_store.beta_est(2,:))*3600*180/pi)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
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
            plot(obj.data_store.tv,(obj.data_store.beta_truth(3,:) - obj.data_store.beta_est(3,:))*3600*180/pi)
            plot(obj.data_store.tv,zeros(1,obj.params.simulation.N_data),'k')
            % sigma = permute(P(6,6,:).^0.5*180/pi*3600,[1 3 2]);
            % plot(tv,3*sigma.*ones(1,N),'--')
            % plot(tv,-3*sigma.*ones(1,N),'--')
            xlabel('Time (h)')
            ylabel('Angular Rate (deg/h)')
            % ylim(mean(abs(sigma))*[-4 4])
            grid on
            set(gca,'Color',[0.9; 0.9; 0.9]);
            hold off
        end
    end
    
    methods (Access = private)
        function initializeDataStore(obj)
            N_data = obj.params.simulation.N_data;
            obj.data_store.idx        = 1;
            obj.data_store.updateTime = 0;
            obj.data_store.updateInterval = obj.params.simulation.dt_data;
            obj.data_store.tv         = NaN(1,N_data);
            obj.data_store.beta_truth = NaN(3,N_data);
            obj.data_store.beta_est   = NaN(3,N_data);
            obj.data_store.L_ctr      = NaN(3,N_data);
            obj.data_store.WheelMomentum = NaN(size(obj.controller.actuators.rwheels.config,2),N_data);
            obj.data_store.WheelTorque = NaN(size(obj.controller.actuators.rwheels.config,2),N_data);
            obj.data_store.qy1        = NaN(4,N_data);
            obj.data_store.qy2        = NaN(4,N_data);
            obj.data_store.omy        = NaN(3,N_data);
            nP                        = length(obj.estimator.estimate.uncertainty);
            obj.data_store.P          = NaN(nP,nP,N_data);
            obj.data_store.q_c        = NaN(4,N_data);
            obj.data_store.r_truth    = NaN(3,N_data);
            obj.data_store.v_truth    = NaN(3,N_data);
            obj.data_store.q_truth    = NaN(4,N_data);
            obj.data_store.q_est      = NaN(4,N_data);
            obj.data_store.dq_c       = NaN(4,N_data);
            obj.data_store.dq_est     = NaN(4,N_data);
            obj.data_store.om_truth   = NaN(3,N_data);
            obj.data_store.om_est     = NaN(3,N_data);
            obj.data_store.est_ang_er = NaN(1,N_data);
            obj.data_store.c_ang_er   = NaN(1,N_data);
        end
        
        function status = checkForEnvironmentUpdate(obj,t)
            if t >= obj.params.environment.updateTime
                status = true;
            else
                status = false;
            end
        end
        
        function status = checkForEstimatorUpdate(obj,t)
            if t >= obj.estimator.updateTime
                status = true;
            else
                status = false;
            end
        end
        
        function status = checkForStoreDataPoint(obj,t)
            if t >= obj.data_store.updateTime
                status = true;
            else
                status = false;
            end
        end
        
        function executeEnvironmentUpdate(obj,t,x)
            % update environmental conditions
            obj.params.environment.update(t,x);
            
            %update update time
            obj.params.environment.updateTime = ...
                obj.params.environment.updateTime + ...
                obj.params.environment.updateInterval;
        end
        
        function executeEstimatorUpdate(obj,t,x)
            
            % update estimator truth
            obj.estimator.truth.update(x, t);
            
            % generate measurements
            obj.estimator.sensors.measure(t);

            % propagate estimate
            if t ~= 0
                obj.estimator.estimate.propagate(t);
            end

            % update estimate
            obj.estimator.estimate.update(t);
            
            % update controller
            obj.controller.update();
            
            % update update time
            obj.estimator.updateTime = obj.estimator.updateTime + ...
                obj.estimator.updateInterval;
        end
        
        function executeStoreDataPoint(obj,t,x)
            % get index to store data at
            i = obj.data_store.idx;
            
            % store data
            obj.data_store.beta_truth(:,i) = obj.estimator.truth.error;
            obj.data_store.beta_est(:,i)   = obj.estimator.estimate.error;
            obj.data_store.P(:,:,i)        = obj.estimator.estimate.uncertainty;
            
            % torque
            L                              = obj.controller.actuators.actuate();
            obj.data_store.L_ctr(:,i)      = sum(L,2);
            
            % momentum
            momentum                       = reshape(x(14:end),3,[]);
            for j = 1:size(momentum,2)
                obj.data_store.WheelMomentum(j,i)   = ...
                    norm(momentum(:,j))*sign(dot(momentum(:,j),...
                    obj.controller.actuators.rwheels.config(:,j)));
                obj.data_store.WheelTorque(j,i) = ...
                    norm(L(:,j))*sign(dot(L(:,j),...
                    obj.controller.actuators.rwheels.config(:,j)));
            end
            
            % measurements
            qy12                           = obj.estimator.sensors.sensor{1}.measurement;
            obj.data_store.qy1(:,i)        = qy12{1};
            obj.data_store.qy2(:,i)        = qy12{2};
            obj.data_store.omy(:,i)        = obj.estimator.sensors.sensor{2}.measurement;
            
            obj.data_store.q_c(:,i)        = obj.controller.control_signal.q_c;
            obj.data_store.tv(i)           = t/3600;
            obj.data_store.r_truth(:,i)    = x(1:3);
            obj.data_store.v_truth(:,i)    = x(4:6);
            obj.data_store.q_truth(:,i)    = x(7:10);
            obj.data_store.q_est(:,i)      = obj.estimator.estimate.state(7:10);
            obj.data_store.dq_c(:,i)       = quat_prod(x(7:10),...
                                                quat_inv(obj.data_store.q_c(:,i)));
            obj.data_store.dq_est(:,i)     = quat_prod(quat_inv(obj.data_store.q_truth(:,i)),...
                                                obj.data_store.q_est(:,i));
            obj.data_store.om_truth(:,i)   = x(11:13);
            obj.data_store.om_est(:,i)     = obj.estimator.estimate.state(11:13);
            
            % pointing errors
            A_truth                        = quat2CTM(obj.data_store.q_truth(:,i));
            A_c                            = quat2CTM(obj.data_store.q_c(:,i));
            A_est                          = quat2CTM(obj.data_store.q_est(:,i));
            point_truth                    = A_truth * [1;0;0];
            point_c                        = A_c * [1;0;0];
            point_est                      = A_est * [1;0;0];
            obj.data_store.est_ang_er(i)   = ...
                acos(dot(point_truth,point_est)/...
                (norm(point_truth)*norm(point_est)));
            obj.data_store.c_ang_er(i)     = ...
                acos(dot(point_truth,point_c)/...
                (norm(point_truth)*norm(point_c)));
            
            % update index
            obj.data_store.idx = i + 1;
            
            % update update time
            obj.data_store.updateTime = obj.data_store.updateTime + ...
                obj.data_store.updateInterval;
            
            % update progressbar
            progressbar(t/obj.params.simulation.te);
        end
    end
    
    methods
        function executeMulti(sim)
            
            % ode inputs
            ti      = sim.params.simulation.ti;
            dt      = sim.pStepSize;
            te      = sim.params.simulation.te;
            x       = sim.initialState;
            odeFcn  = sim.physics_engine.pOdeFcn;
            odeOpts = sim.params.simulation.odeOpts;
            
            % initial update
            sim.odeOutputFcn(ti,x,'init');
            
            % simulation loop
            for t = (ti + dt):dt:te
                tspan = [t - dt, t];
                [~, X] = ode45(odeFcn,tspan,x,odeOpts);
                x = X(end,:)';
                x(7:10) = x(7:10)/norm(x(7:10)); % normallize quaternion
                sim.odeOutputFcn(t,x,[]);
            end
            
            % clean up & plotting
            sim.odeOutputFcn([],[],'done');
            
        end
        
        function executeSingle(sim)
            
            % ode inputs
            ti      = sim.params.simulation.ti;
            dt      = sim.pStepSize;
            te      = sim.params.simulation.te;
            x       = sim.initialState;
            odeFcn  = sim.physics_engine.pOdeFcn;
            odeOpts = sim.params.simulation.odeOpts;
            
            % update options
            odeOpts = odeset(odeOpts,'Refine',1,'MaxStep',dt*0.1,...
                'OutputFcn',@sim.odeOutputFcn);
            
            % single-integration simulation
            ode45(odeFcn,[ti, te],x,odeOpts);
        end
        
        function status = odeOutputFcn(sim,t,x,flag)
            
            if isempty(flag) % normal update routine
                
                % environment update
                if sim.checkForEnvironmentUpdate(t)
                    sim.executeEnvironmentUpdate(t,x);
                end
                
                % estimate/controller update
                if sim.checkForEstimatorUpdate(t)
                    sim.executeEstimatorUpdate(t,x);
                end
                
                % data store update
                if sim.checkForStoreDataPoint(t)
                    sim.executeStoreDataPoint(t,x);
                end
                
            elseif isequal(flag,'init') % initial update
                sim.estimator.truth.t = t(1);
                sim.executeEnvironmentUpdate(t(1),x);
                sim.executeEstimatorUpdate(t(1),x);
                sim.executeStoreDataPoint(t(1),x);
                
            else % closing update
                sim.plotDataStore();
            end
            
            % status needed for ode solver
            status = 0;
        end
    end
end