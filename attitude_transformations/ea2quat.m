function quat = ea2quat(ea)
% ea2quat convert 3-2-1 transformation Euler angles to a quaternion 
%   represented transformation
%
% Gus Buonviri, 1/28/18
% Mississippi State University
%
% INPUTS:
%
%   ea = [3 x 1] {numeric, column vector} 3-2-1 transformation Euler angles
%       in radians. Order is roll-pitch-yaw.
%
% OUTPUTS:
%
%   quat = [4 x 1] {numeric, column vector} quaternion represented
%       transformation, where the vector precedes the scalar.
%
%

s1o2 = sin(ea(1)/2);
s2o2 = sin(ea(2)/2);
s3o2 = sin(ea(3)/2);
c1o2 = cos(ea(1)/2);
c2o2 = cos(ea(2)/2);
c3o2 = cos(ea(3)/2);

quat = [s1o2*c2o2*c3o2 + c1o2*s2o2*s3o2;
        c1o2*s2o2*c3o2 - s1o2*c2o2*s3o2;
        c1o2*c2o2*s3o2 + s1o2*s2o2*c3o2;
        s1o2*s2o2*s3o2 - c1o2*c2o2*c3o2];

quat = quat/norm(quat);

end