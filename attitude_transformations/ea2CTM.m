function CTM = ea2CTM(ea)
% ea2CTM convert 3-2-1 transformation Euler angles to coordinate 
%       transformation matrix
%
% Gus Buonviri, 1/28/18
% Mississippi State University
%
% INPUTS:
%
%   ea = [3 x 1] {column vector, numeric} 3-2-1 transformation Euler angles
%       in radians. Angles correspond to roll, pitch, and yaw.
%
% OUTPUTS:
%
%   Tmatrix = [3 x 3] {numeric, array} coordinate transformation matrix.
%
%

s1   = sin(ea(1));
s2   = sin(ea(2));
s3   = sin(ea(3));
c1   = cos(ea(1));
c2   = cos(ea(2));
c3   = cos(ea(3));
s1c3 = s1*c3;
c1s3 = c1*s3;
s1s3 = s1*s3;
c1c3 = c1*c3;
       
CTM = [         c2*c3,          c2*s3,   -s2;
       s1c3*s2 - c1s3, s1s3*s2 + c1c3, s1*c2;
       c1c3*s2 + s1s3, c1s3*s2 - s1c3, c1*c2];

CTM(abs(CTM)<1e-15) = 0;

end