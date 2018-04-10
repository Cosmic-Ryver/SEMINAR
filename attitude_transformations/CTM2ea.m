function ea = CTM2ea(CTM)
% CTM2ea convert coordinate transformation matrix to 3-2-1 transformation 
%   Euler angles
%
% Gus Buonviri, 1/28/18
% Mississippi State University
%
% INPUTS:
%
%   CTM = [3 x 3] {numeric, array} coordinate transformation matrix.
%
% OUTPUTS:
%
%   ea = [3 x 1] {column vector, numeric} 3-2-1 transformation Euler angles
%       in radians. Order is roll-pitch-yaw.
%

ea = zeros(3,1);

ea(2) = asin(-CTM(1,3));

if abs(ea(2)) - pi/2 == 0     % singularity @ pitch = +/-pi/2
    ea(1) = 0;
    ea(3) = atan2(-CTM(3,2) - CTM(2,1), CTM(2,2) - CTM(3,1));
% elseif abs(ea(2) + pi/2) < 1e-15 % singularity @ pitch = -pi/2
%     ea(1) = 0;
%     ea(3) = atan2(CTM(3,2) - CTM(2,1), CTM(2,2) + CTM(3,1));
else                             % no singularity
    signC2 = sign(cos(ea(2)));
    ea(1)  = atan2(signC2*CTM(2,3), signC2*CTM(3,3));
    ea(3)  = atan2(signC2*CTM(1,2), signC2*CTM(1,1));
end

end