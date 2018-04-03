function [ T ] = ECI2LVLH( r, v )
% GCI2LVLH compute the CTM for the transformation from the ECI frame to the
%   LVLH frame
%
% Gus Buonviri, 2/25/18
% Mississippi State University
%
% INPUTS:
%
%   r = [3 x 1] {column vector, numeric} position in ECI frame.
%
%   v = [3 x 1] {column vector, numeric} velocity in the ECI frame.
%
% OUTPUTS:
%
%   T = [3 x 3] {array, numeric} coordinate transformation matrix for the
%       transformation from the Earth Centered Inertial frame to the Local-
%       Vertical/Local-Horizontal frame.
%
% REFERENCE: Markley, F. Landis, and John L. Crassidis. Fundamentals of 
%       spacecraft attitude determination and control. Vol. 33. New York: 
%       Springer, 2014. Pg 36. Eq 2.78-79.
%
%

% Specific angular momentum
h = cross(r,v);

% LVLH axis unit vectors
o3 = -r/norm(r);
o2 = -h/norm(h);
o1 = cross(o2,o3);

% CTM from ECI to LVLH
T = [o1, o2, o3];

end