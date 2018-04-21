function [ L, h ] = hybrid( q, om, J, q_c, om_c, om_dot_c, h, delta, kp, kd )
%HYBRID Summary of this function goes here

q  = q/norm(q);
dq = quat_prod(q, quat_inv(q_c));
A_c = quat2CTM(q_c);
A   = quat2CTM(q);
dA  = A*A_c';
dom = om - dA*om_c;

if h*dq(4) < -delta
    h = sign(dq(4));
end

% allow for scaling of kp according to dq
if length(kp)~=1
    dq4_er = 1 - abs(dq(4));
    if dq4_er == 0
        kp = 0;
    else
        dom_max = max(abs(dom));
        if (dq4_er < 1e-3) && (dom_max < 5e-6) % pointing err low, max rate less than ~1 deg/hr
            y = dq4_er^-2;
            dkp = diff(kp);
            kp = kp(1) + (dkp*y - dkp)/(y + dkp);
        else
            kp = kp(1);
        end
    end
end

L = cross(dA*om_c,J*dA*om_c) + J*dA*om_dot_c - kp*h*dq(1:3) - kd*dom;

end

