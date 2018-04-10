classdef (Abstract) sim_component < handle
    properties (SetAccess = protected, GetAccess = public)
        pOwningSimHandle(1,1)
    end
end