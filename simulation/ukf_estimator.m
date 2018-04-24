classdef ukf_estimator < sim_component
    
    properties
        estimate
        truth
        sensors
        updateTime
        updateInterval
        dgStore
        xiPerpStore
    end
    methods

        function obj = ukf_estimator(aOwningSimHandle)

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
            t0 = obj.pOwningSimHandle.params.simulation.ti;
            obj.estimate.t = t0;
            
            % initial pos/vel estimate
            obj.estimate.state(1:6) = initialState(1:6);
            
            % initial attitude error covariance matrix
            P_dv_0     = ((6/3600)*(pi/180))^2*eye(3);

            % initial gyro bias error covariance matrix
            P_dbeta_0  = ((0.02/3600)*(pi/180))^2*eye(3);

            % Initial error covariance matrix
            obj.estimate.uncertainty = blkdiag(P_dv_0, P_dbeta_0);

            % initial attitude error (sampled from initial estimate distribution)
            dg_0            = (diag(P_dv_0).^0.5).*randn(3,1);

            % initial attitude (error x estimate; normalized)
            q_truth_0  = initialState(7:10);
            q_est_0    = quat_prod(CTM2quat(dg2CTM(dg_0)),q_truth_0);
            obj.estimate.state(7:10) = q_est_0;

            % initial gyro bias (error + estimate)
            obj.truth.error = beta_est_0 + diag(P_dbeta_0).^0.5.*randn(3,1);

            % initial time
            obj.truth.t = t0;

            %%%%%%%%%%
            % Sensor parameters
            
            %%%%%%%%%
            % Measurement model

            % Measurement model
            h = @(m) 0;

            % Measurement sensitivity
            H = @(m) [eye(3), zeros(3); eye(3), zeros(3)];

  
            % Star tracker covariance (rad^2) (based on terma T1)
            R1 = diag(([1.5 1.5 9.5]*(1/3600*pi/180)).^2);
            R2 = diag(([1.5 9.5 1.5]*(1/3600*pi/180)).^2);
            R  = blkdiag(R1,R2);

            % Gyro rate standard deviation (based on SIRU-E from NG)
            sigma_v = 0.00005/3600*pi/180;

            % Gyro bias standard deviation (based on SIRU-E from NG)
            sigma_u = 0.0005/3600*pi/180;

            Q = blkdiag(sigma_v^2*eye(3),sigma_u^2*eye(3));
            
            % timing
            rates = [1, 4, 4, 4, 4]; % Hz
            frequencies = rates.^-1; % s
            obj.sensors.base_frequency = min(frequencies);
            
            % star trackers 1 & 2
            idx = 1;
            obj.sensors.sensor{idx}.rate = rates(idx);
            obj.sensors.sensor{idx}.updateInterval = frequencies(idx);
            obj.sensors.sensor{idx}.updateTime     = t0;
            obj.sensors.sensor{idx}.measure = @(varargin) ...
                { star_tracker(obj.truth.state(7:10),R1), ...
                  star_tracker(obj.truth.state(7:10),R2) };

            % gyro
            idx = 2;
            obj.sensors.sensor{idx}.rate = rates(idx);
            obj.sensors.sensor{idx}.updateInterval = frequencies(idx);
            obj.sensors.sensor{idx}.updateTime     = t0;
            obj.sensors.sensor{idx}.measure = @(varargin) ...
                biased_gyro(obj.truth.state(11:13),sigma_v, ...
                obj.truth.error);
            
            % clock
            idx = 3;
            obj.sensors.sensor{idx}.rate = rates(idx);
            obj.sensors.sensor{idx}.updateInterval = frequencies(idx);
            obj.sensors.sensor{idx}.updateTime     = t0;
            obj.sensors.sensor{idx}.measure = @(varargin) ...
                obj.truth.t;
            
            % wheel momentum psuedo-sensor
            idx = 4;
            obj.sensors.sensor{idx}.rate           = rates(idx);
            obj.sensors.sensor{idx}.updateInterval = frequencies(idx);
            obj.sensors.sensor{idx}.updateTime     = t0;
            obj.sensors.sensor{idx}.measure        = @(varargin) ...
                obj.truth.state(14:end);
            
            % position/velocity psuedo-sensor
            idx = 5;
            obj.sensors.sensor{idx}.rate           = rates(idx);
            obj.sensors.sensor{idx}.updateInterval = frequencies(idx);
            obj.sensors.sensor{idx}.updateTime     = t0;
            obj.sensors.sensor{idx}.measure        = @(varargin) ...
                obj.truth.state(1:6);

            % all
            obj.sensors.measure = @(t) obj.measure(t);

            %%%%%%%%%%%%%
            % Truth update and propagation

            obj.truth.update = @(x_tru, t) ...
                obj.update_truth(x_tru, t, sigma_u);

            obj.estimate.propagate = @(t) ...
                obj.propagate_estimate(t,Q,2);

            %%%%%%%%%%%%%
            % Measurement update
            % unique updator for each sensor
            
            % attitude & bias
            idx = 1;
            obj.estimate.updaters{idx}.update = @(varargin) ...
                obj.filter_update(1,R,h,H);
            obj.estimate.updaters{idx}.updateTime = t0;
            obj.estimate.updaters{idx}.updateInterval = ...
                obj.sensors.sensor{idx}.updateInterval;
            
            % angular rate
            idx = 2;
            obj.estimate.updaters{idx}.update = @(varargin) ...
                obj.gyro_update(idx);
            obj.estimate.updaters{idx}.updateTime = t0;
            obj.estimate.updaters{idx}.updateInterval = ...
                obj.sensors.sensor{idx}.updateInterval;
            
            % time
            idx = 3;
            obj.estimate.updaters{idx}.update = @(varargin) ...
                obj.clock_update(idx);
            obj.estimate.updaters{idx}.updateTime = t0;
            obj.estimate.updaters{idx}.updateInterval = ...
                obj.sensors.sensor{idx}.updateInterval;
            
            % wheel momentum
            idx = 4;
            obj.estimate.updaters{idx}.update = @(varargin) ...
                obj.wheel_momentum_update(idx);
            obj.estimate.updaters{idx}.updateTime = t0;
            obj.estimate.updaters{idx}.updateInterval = ...
                obj.sensors.sensor{idx}.updateInterval;
            
            % wheel momentum
            idx = 5;
            obj.estimate.updaters{idx}.update = @(varargin) ...
                obj.pos_vel_update(idx);
            obj.estimate.updaters{idx}.updateTime = t0;
            obj.estimate.updaters{idx}.updateInterval = ...
                obj.sensors.sensor{idx}.updateInterval;

            % all
            obj.estimate.update = @(t) ...
                obj.update_estimate(t);

            %%%%%%%%%%%%%%%%
            % Set initial update time and update interval

            obj.updateTime = t0;
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

        function propagate_estimate(obj,t,Q,gyro_idx)
            
            % preliminaries
            tspan  = [obj.estimate.t t];
            q_est  = obj.estimate.state(7:10);
            om_est = obj.estimate.state(11:13);
            om_raw = obj.sensors.sensor{gyro_idx}.measurement;
            P      = obj.estimate.uncertainty;
            B_est  = obj.estimate.error;

            % fetch sigma points
            [X, W] = getSigmaPoints(zeros(6,1),P(1:6,1:6),0);
            
            % preallocate
            nSP = size(X,2);
            Xdg = X(1:3,:);
            XdB  = X(4:6,:);
            Xdq = NaN(4,nSP);
            Xq  = NaN(4,nSP);
            
            Xq(:,1) = q_est;
            for i = 2:nSP
                XmagSqr    = norm(Xdg(1:3,i))^2;
                Xdq(4,i)   = -sqrt(1 + XmagSqr)/(1 + XmagSqr);
                Xdq(1:3,i) = Xdq(4,i)*Xdg(1:3,i);
                Xq(:,i)    = quat_prod(Xdq(:,i),q_est);
            end
            
            Xqm = NaN(4,nSP);
            M   = zeros(4,4);
            opts  = odeset('RelTol',1e-8,'AbsTol',1e-10);
            for i = 1:nSP
                
                om_new = om_raw - (B_est + XdB(:,i));
                om_interp_fnc = @(t) om_est + (om_new - om_est)*...
                    (t - tspan(1))/diff(tspan);
                
                [ ~, sol ] = ode45(@(t, x) ...
                    0.5*quat_prod([om_interp_fnc(t); 0],x/norm(x)), ...
                    tspan, Xq(:,i), opts);
                qXmi     = sol(end,:)';
                
                Xqm(:,i) = qXmi/norm(qXmi);
                
                M        = M + W(i)*(Xqm(:,i)*Xqm(:,i)') - eye(4);
            end
            M        = 4*M;
            [V, D]   = eig(M);
            d        = diag(D);
            [~, idx] = max(d);
            
            q_est_km_guess = V(:,idx)/norm(V(:,idx));

            obj.estimate.state(7:10) = newton_raphson(...
                @(x,varargin)obj.propMMSE(x,Xqm,W,nSP),...
                q_est_km_guess,[]);
            
            Pq_km = zeros(6);
            q_km_inv = quat_inv(obj.estimate.state(7:10));
            for i = 1:nSP
                dq = quat_prod(Xqm(:,i),q_km_inv);
                dg = dq(1:3)/dq(4);
                Pq_km = Pq_km + W(i)*([dg;X(4:6,i)]*...
                    [dg;X(4:6,i)]') + Q;
            end
            
            obj.estimate.uncertainty(1:6,1:6) = Pq_km;
        end
        
        function [out] = propMMSE(estimator, q, q_sigPt, Wm, nSP)
            out = 0;
            q_inv = quat_inv(q);
            dg    = zeros(3,nSP);
            for i = 1:nSP
                dq      = quat_prod(q_sigPt(:,i),q_inv);
                dg(:,i) = 2*dq(1:3)/dq(4);
                out     = out + Wm(i)*norm(dg(:,i))^2;
            end
            estimator.dgStore = dg;
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
        
        function pos_vel_update(estimator, sensorIdx)
                estimator.estimate.state(1:6) = ...
                estimator.sensors.sensor{sensorIdx}.measurement;
        end

%         function filter_update(obj, sensorIdx, H, R)
%             q_km = obj.estimate.state(7:10);
%             beta_km = obj.estimate.error;
%             P_km = obj.estimate.uncertainty;
%             qy = obj.sensors.sensor{sensorIdx}.measurement;
%             nq = length(qy);
%             y = zeros(3*nq,1);
%             for i = 1:nq
%                 dqi = quat_prod(qy{i},quat_inv(q_km));
%                 yi  = 2 * dqi(1:3)/dqi(4);
%                 y((1 + (i-1)*3):(i*3)) = yi;
%             end
% 
%             P_aug = blkdiag(P_km, H, R);
%             n     = length(P_aug);
%             nSP   = 2*n + 1;
%             kappa = 1;
%             sqrtnP_aug = sqrtm((n+kappa).*P_aug);
%             
%             sigPts = zeros(n,nSP);
%             sigPts(:,2:(n + 1)) = sqrtnP_aug;
%             sigPts(:,(n + 2):end) = -sqrtnP_aug;
%             
%             w   = zeros(1,nSP);
%             w(1) = kappa/(n+kappa);
%             w(2:end) = 0.5/(n+kappa) * ones(1,nSP-1);
%             
%             CTM_ECI2EST = quat2CTM(q_km);
%             r = CTM_ECI2EST'*[1;0;0];
%             
%             Y = zeros(nq*3 + 3,nSP);
%             for i = 1:nSP
%                 CTM_EST2dg = dg2CTM(sigPts(1:3,i));
%                 for j = 1:nq
%                     jIdxs = (1 + (j - 1)*3):(j*3);
%                     jIdxs = jIdxs + 9;
%                     Y(jIdxs - 6,i) = dg2CTM(sigPts(jIdxs,i))*CTM_EST2dg*...
%                         CTM_ECI2EST*r;
%                 end
%                 Y(1:3,i) = sigPts(4:6,i) + sigPts(7:9,i);
%             end
%             
%             yhat = zeros(3,nq);
%             Z   = zeros(3 + nq*3,nSP);
%             bndFcns{1} = @(x,varargin) x(1) + 1;
%             bndFcns{2} = @(x,varargin) x(2) + 2*pi;
%             bndFcns{3} = @(x,varargin) 1 - x(1);
%             bndFcns{4} = @(x,varargin) 2*pi - x(2);
%             x          = zeros(2,nq);
%             for j = 1:nq
%                 jIdxs = (1 + (j - 1)*3):(j*3);
%                 jIdxs = jIdxs + 3;
%                 x(:,j) = interior_point_optim(...
%                     @(x,varargin)obj.updMMSE(x,Y(jIdxs,:),w,nSP),...
%                     bndFcns,[0.5;pi/2]);
%                 x(1,j) = x(1,j);
%                 yhat(3,j) = x(1,j);
%                 rxy   = sqrt(1 - abs(x(1,j)));
%                 if sign(x(1,j)) < 0
%                     x(2,j)  = sign(x(1))*x(2,j);
%                 end
%                 yhat(2,j) = rxy*sin(x(2,j));
%                 yhat(1,j) = rxy*cos(x(1,j));
%                 dbiashat = newton_raphson(@(x,varargin)obj.biasMMSE(x,Y(1:3,:),w,nSP),[0;0;0],[]);
%             end
%             
%             Pzz = zeros(3 + nq*3);
%             Pxz = zeros(6,3 + nq*3);
%             for i = 1:nSP
%                 for j = 1:nq
%                     jIdxs = (1 + (j - 1)*3):(j*3);
%                     jIdxs = jIdxs + 3;
%                     obj.updMMSE(x(:,j),Y(jIdxs,:),w,nSP);
%                     Z(jIdxs,i) = obj.xiPerpStore(:,i);
%                 end
%                 Z(1:3,i) = dbiashat - Y(1:3,i);
%                 Pzz = Pzz + w(i)*4*(Z(:,i)*Z(:,i)');
%                 Pxz = Pxz + w(i)*2*(sigPts(1:6,i)*Z(:,i)');
%             end
%             
%             K = Pxz*pinv(Pzz);
%             
%             P_k = P_km - K*Pzz*K';
%             
%             eta = zeros(3 + nq*3,1);
%             for j = 1:nq
%                 eta(1:3) = dbiashat;
%                 jIdxs = (1 + (j - 1)*3):(j*3);
%                 eta(jIdxs + 3) = 2*cross(yhat(:,j),y(jIdxs))/...
%                     (1 + dot(yhat(:,j),y(jIdxs)));
%             end
%             
%             er_k = zeros(6,1) + K*eta;
%             
%             % extract errors from error-state vector
%             dg_k    = er_k(1:3);
%             dbeta_k = er_k(4:6);
% 
%             % update attitude quaternion
%             q_k = quat_prod(CTM2quat(dg2CTM(dg_k)),q_km);
% 
%             % update gyro bias
%             beta_k = beta_km + dbeta_k;
% 
%             % set updated info in struct
%             obj.estimate.state(7:10) = q_k;
%             obj.estimate.error = beta_k;
%             obj.estimate.uncertainty = P_k;
%         end
        
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


        function [out] = biasMMSE(obj, x, Y, w, nSP)
            out = 0;
            for i = 1:nSP
                out = out + w(i)*norm(x - Y(:,i))^2;
            end
        end
        
        function [out] = updMMSE(obj, x, Y, w, nSP)
            out = 0;
            xi = zeros(3,1);
            xi(3) = x(1);
            rxy   = sqrt(1 - abs(xi(3)));
            if sign(xi(3)) < 0
                x(2)  = sign(xi(3))*x(2);
            end
            xi(2) = rxy*sin(x(2));
            xi(1) = rxy*cos(x(2));
            
            xi = xi/norm(xi);
            obj.xiPerpStore = NaN(3,nSP);
            for i = 1:nSP
                xXiYi = cross(xi,Y(:,i));
                if xXiYi == 0
                    obj.xiPerpStore(:,i) = zeros(3,1);
                    continue
                end
                xiPerp = xXiYi/(1 + dot(xi,Y(:,i)));
                out = out + norm(w(i)*xiPerp.*xiPerp);
                obj.xiPerpStore(:,i) = xiPerp;
            end
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