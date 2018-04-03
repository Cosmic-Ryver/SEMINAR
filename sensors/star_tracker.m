function [ q_z ] = star_tracker( q_tru, R )
% STAR_TRACKER Summary of this function goes here

dv  = (diag(R).^0.5).*randn(3,1);
q_z = q_tru + quat_prod([dv/2; 0], q_tru);
q_z = q_z/norm(q_z);

end