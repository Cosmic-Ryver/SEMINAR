function [CTM] = dg2CTM(dg)
%DG2CTM 

CTM = eye(3) - 2*skew(dg) + 2*skew(dg)^2;

end

