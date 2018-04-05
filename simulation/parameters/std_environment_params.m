muE    = 3.986004418e14;
muMoon = 4.9048695e12;
rE     = 6378137;
update = @(t, x, params) setfield(params,{1},'environment',{1},'b',...
    IGRF(x(1:3), t/86400 + params.simulation.jd0, 13,'ECI'));