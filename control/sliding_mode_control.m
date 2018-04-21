function [ L ] = sliding_mode_control( q, om, H, J, q_c, om_c, om_dot_c, k, G, eta )

q  = q/norm(q);
dq = quat_prod(q, quat_inv(q_c));

s = (om - om_c) + k * sign(dq(4))*dq(1:3);
s_bar = zeros(3,1);

for i = 1:3
    if s(i) > eta(i)
        s_bar(i) = 1;
    elseif s(i) < -eta(i)
        s_bar(i) = -1;
    else
        s_bar(i) = s(i)/eta(i);
    end
end

L = J*((k/2)*(abs(dq(4))*(om_c - om) - ...
    cross(sign(dq(4))*dq(1:3),om + om_c)) + om_dot_c - G*s_bar) + ...
    cross(om,J*om + H);

end

