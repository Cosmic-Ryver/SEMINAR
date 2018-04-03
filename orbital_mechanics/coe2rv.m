%{
Gus Buonviri
1/26/15
Adv Orb: Assign 1
%}
function [r,v] = coe2rv (a,ecc,inc,raand,aop,ta,mu)
O = raand;                                   %shorthand notation
o = aop;                                   %shorthand notation
h = sqrt(a*(1-ecc^2)*mu);                      %angular momentum
rx = ((h^2/mu)*1/(1+ecc*cos(ta)))*[cos(ta);sin(ta);0]; %perifocal position
vx = (mu/h)*[-sin(ta);ecc+cos(ta);0];                  %perifocal velocity
QxX = [-sin(O)*cos(inc)*sin(o)+cos(O)*cos(o),...       %coord transf matrix
    -sin(O)*cos(inc)*cos(o)-cos(O)*sin(o),sin(O)*sin(inc);...
    cos(O)*cos(inc)*sin(o)+sin(O)*cos(o),...
    cos(O)*cos(inc)*cos(o)-sin(O)*sin(o),-cos(O)*sin(inc);sin(inc)*sin(o),...
    sin(inc)*cos(o),cos(inc)];                   %end coord transf matrix
r = QxX*rx;                                  %coord transf opperation
v = QxX*vx;                                  %coord transf opperation