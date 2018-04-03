function [ dxdt ] = zonal_harmonics_perturbation( t, x, mu, rE, degree )
% zonal_harmonics_perturbation calculate the perturbation due to the
%   Earth's shape

J = [                0;
      1.08262668355e-3;
     -2.53265648533e-6;
     -1.61962159137e-6;
     -2.27296082869e-7;
      5.40681239107e-7];

if degree > length(J)
    warning('requested higher harmonic than was available, defaulting to max')
    degree = length(J);
end

dxdt  = zeros(length(x),1);
r     = x(1:3);
r_mag = norm(r);
r_hat = r/r_mag;

if degree >= 2
    dxdt(4:6) = dxdt(4:6) - 1.5*J(2)*mu/r_mag^2*(rE/r_mag)^2*...
        [(1 - 5*r_hat(3)^2)*r_hat(1);
         (1 - 5*r_hat(3)^2)*r_hat(2);
         (3 - 5*r_hat(3)^2)*r_hat(3)];
end
if degree >= 3
    dxdt(4:6) = dxdt(4:6) - 0.5*J(3)*mu/r_mag^2*(rE/r_mag)^3*...
        [5*(7*r_hat(3)^3 - 3*r_hat(3))*r_hat(1);
         5*(7*r_hat(3)^3 - 3*r_hat(3))*r_hat(2);
              30*r_hat(3)^2 - 35*r_hat(3)^4 - 3];
end
if degree >= 4
    dxdt(4:6) = dxdt(4:6) - 0.625*J(4)*mu/r_mag^2*(rE/r_mag)^4*...
    [   (3 - 42*r_hat(3)^2 + 63*r_hat(3)^4)*r_hat(1);
        (3 - 42*r_hat(3)^2 + 63*r_hat(3)^4)*r_hat(2);
     -(15 - 70*r_hat(3)^2 + 63*r_hat(3)^4)*r_hat(3)];
end
if degree >= 5
    dxdt(4:6) = dxdt(4:6) - 0.125*J(5)*mu/r_mag^2*(rE/r_mag)^5*...
    [3*(35*r_hat(3) - 210*r_hat(3)^3 + 231*r_hat(3)^5)*r_hat(1);
     3*(35*r_hat(3) - 210*r_hat(3)^3 + 231*r_hat(3)^5)*r_hat(1);
          15 - 315*r_hat(3)^2 + 945*r_hat(3)^4 - 693*r_hat(3)^6];
end
if degree >= 6
    dxdt(4:6) = dxdt(4:6) + 0.0625*J(6)*mu/r_mag^2*(rE/r_mag)^6*...
    [(35 - 945*r_hat(3)^2 + 3465*r_hat(3)^4 - 3003*r_hat(3)^6)*r_hat(1);
     (35 - 945*r_hat(3)^2 + 3465*r_hat(3)^4 - 3003*r_hat(3)^6)*r_hat(1);
     2205*r_hat(3)^3 - 4851*r_hat(3)^5 + 3003*r_hat(3)^7 - 315*r_hat(3)];
end

end

