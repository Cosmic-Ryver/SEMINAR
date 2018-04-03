%{
Gus Buonviri
1/26/15
Adv Orb: Assign 1
%}
function [a,ecc,inc,raand,aop,ta] = rv2coe (r,v,mu)
rmag = norm(r);                              %magnitude of position vector
vmag = norm(v);                              %magnitude of velocity vector
vr = dot(r,v)/rmag;                          %radial velocity
h = cross(r,v);                              %angular momentum vector
hmag = norm(h);                              %angular momentum magnitude
inc = acos(h(3)/hmag);                         %inclination
k = [0,0,1];                                 %intermediate vector
n = cross(k,h);                              %intermediate vector
nmag = norm(n);                              %magnitude of n vector
if n(2) >= 0;                                %checking quadrant
    raand = acos(n(1)/nmag);                 %calculating R.A.A.Nd.
else
    raand = 2*pi-acos(n(1)/nmag);            %calculating R.A.A.Nd.
end
evec = (1/mu)*(((vmag^2-(mu/rmag))*r)-(rmag*vr*v)); %vector form of eccent.
ecc = norm(evec);                              %ecentricity
if evec(3)>= 0;                              %checking magnitude
    aop = acos(dot(n,evec)/(nmag*ecc));      %calculating arg of perigee
else
    aop = 2*pi-acos(dot(n,evec)/(nmag*ecc)); %calculating arg of perigee
end
if vr >= 0;                                  %checking quadrant
    ta = acos(dot(evec,r)/(ecc*rmag));         %calculating true anomaly
else
    ta = 2*pi-acos(dot(evec,r)/(ecc*rmag));    %calculating true anomaly
end
a = (hmag^2/mu)*1/(1-ecc^2);                   %semi-major axis