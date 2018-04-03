function [ L ] = hybrid( q, om, J, q_c, om_c, om_dot_c, kp, kd )
%HYBRID Summary of this function goes here

dq = quat_prod(q, quat_inv(q_c));
A_c = quat2CTM(q_c);
A   = quat2CTM(q);
dA  = A*A_c';
dom = om - dA*om_c;

tol = 1e-10;
if abs(dq(4)) > tol
    h = sign(dq(4));
elseif dq(4) > 0
    h = 1;
else
    h = -1;
end

L = cross(dA*om_c,J*dA*om_c) + J*dA*om_dot_c - kp*h*dq(1:3) - kd*dom;

end

