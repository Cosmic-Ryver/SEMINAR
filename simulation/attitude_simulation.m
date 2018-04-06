classdef attitude_simulation < handle
    
    properties (
    methods sim = attitude_simulation( param_scripts )
        sim.spacecraft( param_scripts{1} );
        sim.orbit( param_scripts{2} );
        sim.environment( param_scripts{3} );
        
    end
end