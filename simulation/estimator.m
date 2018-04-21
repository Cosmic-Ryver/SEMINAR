classdef estimator < sim_component
    
    properties
        estimate
        truth
        sensors
        updateTime
        updateInterval
    end
    methods

        function obj = estimator(aOwningSimHandle)

            obj.pOwningSimHandle = aOwningSimHandle;
            
            initialState = obj.pOwningSimHandle.initialState;
            
            %%%%%%%%%%%
            % Initial conditions

            % preallocate
            obj.estimate.state = zeros(13,1);
            
            % initiallize truth
            obj.truth.state = initialState;

            % initial gyro bias estimate (rad/s)
            beta_est_0 = [0.1/3600; 0.1/3600; 0.1/3600] * pi/180;
            obj.estimate.error = beta_est_0;

            % initial time estimate
            obj.estimate.t = 0;
            
            % initial attitude error covariance matrix
            P_dv_0     = ((6/3600)*(pi/180))^2*eye(3);

            % initial gyro bias error covariance matrix
            P_dbeta_0  = ((0.02/3600)*(pi/180))^2*eye(3);

            % Initial error covariance matrix
            obj.estimate.uncertainty = blkdiag(P_dv_0, P_dbeta_0);

            % initial attitude error (sampled from initial estimate distribution)
            dv_0            = (diag(P_dv_0).^0.5).*randn(3,1);

            % initial attitude (error x estimate; normalized)
            q_truth_0  = initialState(7:10);
            q_est_0    = q_truth_0 + quat_prod([dv_0/2;0],q_truth_0);
            q_est_0    = q_est_0/norm(q_est_0);
            obj.estimate.state(7:10) = q_est_0;

            % initial gyro bias (error + estimate)
            obj.truth.error = beta_est_0 + diag(P_dbeta_0).^0.5.*randn(3,1);

            % initial time
            obj.truth.t = 0;

            %%%%%%%%%
            % Measurement model

            % Measurement model
            h = @(m) 0;

            % Measurement sensitivity
            H = @(m) [eye(3), zeros(3); eye(3), zeros(3)];

            %%%%%%%%%%
            % Sensor parameters

            % Star tracker covariance (rad^2) (based on terma T1)
            R1 = diag(([1.5 1.5 9.5]*(1/3600*pi/180)).^2);
            R2 = diag(([1.5 9.5 1.5]*(1/3600*pi/180)).^2);
            R  = blkdiag(R1,R2);

            % Gyro rate standard deviation (based on SIRU-E from NG)
            sigma_v = 0.00005/3600*pi/180;

            % Gyro bias standard deviation (based on SIRU-E from NG)
            sigma_u = 0.0005/3600*pi/180;

            % timing
            rates = [1, 4, 4, 4]; % Hz
            frequencies = rates.^-1; % s
            obj.sensors.base_frequency = min(frequencies);
            
            % star trackers 1 & 2
            idx = 1;
            obj.sensors.sensor{idx}.rate = rates(idx);
            obj.sensors.sensor{idx}.updateInterval = frequencies(idx);
            obj.sensors.sensor{idx}.updateTime     = 0;
            obj.sensors.sensor{idx}.measure = @(varargin) ...
                { star_tracker(obj.truth.state(7:10),R1), ...
                  star_tracker(obj.truth.state(7:10),R2) };

            % gyro
            idx = 2;
            obj.sensors.sensor{idx}.rate = rates(idx);
            obj.sensors.sensor{idx}.updateInterval = frequencies(idx);
            obj.sensors.sensor{idx}.updateTime     = 0;
            obj.sensors.sensor{idx}.measure = @(varargin) ...
                biased_gyro(obj.truth.state(11:13),sigma_v, ...
                obj.truth.error);
            
            % clock
            idx = 3;
            obj.sensors.sensor{idx}.rate = rates(idx);
            obj.sensors.sensor{idx}.updateInterval = frequencies(idx);
            obj.sensors.sensor{idx}.updateTime     = 0;
            obj.sensors.sensor{idx}.measure = @(varargin) ...
                obj.truth.t;
            
            % wheel momentum psuedo-sensor
            idx = 4;
            obj.sensors.sensor{idx}.rate           = rates(idx);
            obj.sensors.sensor{idx}.updateInterval = frequencies(idx);
            obj.sensors.sensor{idx}.updateTime     = 0;
            obj.sensors.sensor{idx}.measure        = @(varargin) ...
                obj.truth.state(14:end);

            % all
            obj.sensors.measure = @(t) obj.measure(t);

            %%%%%%%%%%%%%
            % Covariance dynamics

            F = @(om) [-skew(om), -eye(3); zeros(3), zeros(3)];

            G = blkdiag(-eye(3),eye(3));

            Q = blkdiag(sigma_v^2*eye(3),sigma_u^2*eye(3));

            %%%%%%%%%%%%%
            % Truth update and propagation

            obj.truth.update = @(x_tru, t) ...
                obj.update_truth(x_tru, t, sigma_u);

            obj.estimate.propagate = @(t) ...
                obj.propagate_estimate(t,F,G,Q,2);

            %%%%%%%%%%%%%
            % Measurement update
            % unique updator for each sensor
            
            % attitude & bias
            idx = 1;
            obj.estimate.updaters{idx}.update = @(varargin) ...
                obj.filter_update(1, R, h, H);
            obj.estimate.updaters{idx}.updateTime = 0;
            obj.estimate.updaters{idx}.updateInterval = ...
                obj.sensors.sensor{idx}.updateInterval;
            
            % angular rate
            idx = 2;
            obj.estimate.updaters{idx}.update = @(varargin) ...
                obj.gyro_update(idx);
            obj.estimate.updaters{idx}.updateTime = 0;
            obj.estimate.updaters{idx}.updateInterval = ...
                obj.sensors.sensor{idx}.updateInterval;
            
            % time
            idx = 3;
            obj.estimate.updaters{idx}.update = @(varargin) ...
                obj.clock_update(idx);
            obj.estimate.updaters{idx}.updateTime = 0;
            obj.estimate.updaters{idx}.updateInterval = ...
                obj.sensors.sensor{idx}.updateInterval;
            
            % wheel momentum
            idx = 4;
            obj.estimate.updaters{idx}.update = @(varargin) ...
                obj.wheel_momentum_update(idx);
            obj.estimate.updaters{idx}.updateTime = 0;
            obj.estimate.updaters{idx}.updateInterval = ...
                obj.sensors.sensor{idx}.updateInterval;

            % all
            obj.estimate.update = @(t) ...
                obj.update_estimate(t);

            %%%%%%%%%%%%%%%%
            % Set initial update time and update interval

            obj.updateTime = 0;
            obj.updateInterval = obj.sensors.base_frequency;

            %%%%%%%%%%%%%%%
            % Initiallize rate estimate

            obj.sensors.sensor{2}.measurement = ...
                obj.sensors.sensor{2}.measure(obj);
            obj.estimate.updaters{2}.update(obj);

        end

        
        
        function measure(estimator, t)

            N = length(estimator.sensors.sensor);
            for i = 1:N
                if t >= estimator.sensors.sensor{i}.updateTime
                    estimator.sensors.sensor{i}.measurement = ...
                        estimator.sensors.sensor{i}.measure(estimator);
                    estimator.sensors.sensor{i}.updataTime = ...
                        estimator.sensors.sensor{i}.updateTime + ...
                        estimator.sensors.sensor{i}.updateInterval;
                end
            end
        end

        function update_truth(estimator, x_tru, t, sigma_u)
            dt = t - estimator.truth.t;
            estimator.truth.error = estimator.truth.error + ...
                sigma_u*dt^0.5*randn(3,1);
            estimator.truth.t = t;
            estimator.truth.state = x_tru;
        end

        function propagate_estimate(estimator,t,F,G,Q,gyro_idx)
            tspan  = [estimator.estimate.t t];
            q_est  = estimator.estimate.state(7:10);
            om_est = estimator.estimate.state(11:13);
            om_new = estimator.sensors.sensor{gyro_idx}.measurement - ...
                estimator.estimate.error;
            dim    = length(G);
            Pr     = @(p) reshape(p,dim,dim);
            P      = estimator.estimate.uncertainty;
            Xn     = cell(2,1);
            om_interp_fnc = @(t) om_est + (om_new - om_est)*...
                (t - tspan(1))/diff(tspan);

            for i = 1:2
                if i == 1
                    opts  = odeset('RelTol',1e-8,'AbsTol',1e-10);
                    [ ~, X ] = ode45(@(t, x) ...
                        0.5*quat_prod([om_interp_fnc(t); 0],x/norm(x)), ...
                        tspan, q_est, opts);
                    Xn{i} = X;
                else
                    rP = @(P) reshape(P,dim^2,1);
                    P_est = rP(P);
                    opts   = odeset('RelTol',1e-10,'AbsTol',1e-13);
                    [~,X] = ode45(@(t,x) rP(F(om_interp_fnc(t))*Pr(x) +...
                        Pr(x)*F(om_interp_fnc(t))' + G*Q*G'),...
                            tspan,P_est,opts);
                    Xn{i} = X;
                end
            end

            estimator.estimate.state(7:10) = Xn{1}(end,:)'/norm(Xn{1}(end,:));
            estimator.estimate.uncertainty = Pr(Xn{2}(end,:)');
        end

        function gyro_update(estimator, sensorIdx)
            estimator.estimate.state(11:13) = ...
                estimator.sensors.sensor{sensorIdx}.measurement - ...
                estimator.estimate.error;
        end

        function clock_update(estimator, sensorIdx)
                estimator.estimate.t = ...
                estimator.sensors.sensor{sensorIdx}.measurement;
        end
        
        function wheel_momentum_update(estimator, sensorIdx)
                estimator.estimate.Hw = ...
                estimator.sensors.sensor{sensorIdx}.measurement;
        end

        function filter_update(estimator, sensorIdx, R, h, H)
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

        function update_estimate(estimator, t)

            N = length(estimator.sensors.sensor);
            for i = 1:N
                if t >= estimator.estimate.updaters{i}.updateTime
                    estimator.estimate.updaters{i}.update(estimator);
                    estimator.estimate.updaters{i}.updateTime = ...
                        estimator.estimate.updaters{i}.updateTime + ...
                        estimator.estimate.updaters{i}.updateInterval;
                end
            end
        end
    end
end