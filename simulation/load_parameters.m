function [ params ] = load_parameters(param_script,param_type)
% LOAD_PHYSICAL_PARAMS Run script specifying parameters and return relevant
%   parameters
%
% Gus Buonviri, 2/26/18
% Mississippi State University
%
% INPUTS:
%
%   param_script = {char array} name of parameter specifying script
%
% OUTPUTS:
%
%   
%

persistent paramsFolderOnPath
if isempty(paramsFolderOnPath)
    thisDir = fileparts(which('load_parameters.m'));
    path2Add = [thisDir filesep 'parameters' filesep];
    addpath(path2Add);
    paramsFolderOnPath = true;
end

switch param_type
    case 'physical'
        run(param_script)
        params.m             = m;
        params.J             = J;
        params.res_dipole    = res_dipole;
        params.surface_model = surface_model;
        params.Cd            = Cd;
    case 'orbital'
        run(param_script)
        params.a         = a;
        params.ecc       = ecc;
        params.inc       = inc;
        params.raand     = raand;
        params.aop       = aop;
        params.true_anom = true_anom;
        params.r         = r;
        params.v         = v;
    case 'environment'
        run(param_script)
        params.muE              = muE;
        params.muMoon           = muMoon;
        params.rE               = rE;
        params.AU               = AU;
        params.update           = update;
        params.updateTime       = updateTime;
        params.updateInterval   = updateInterval;
    case 'simulation'
        run(param_script)
        params.ti      = ti;
        params.dt_data = dt_data;
        params.te      = te;
        params.tv_data = tv_data;
        params.N_data  = N_data;
        params.jd0     = jd0;
        params.odeOpts = odeOpts;
    case 'kin_dyn_engine'
        params = feval(param_script);
    case 'physics_engine'
        params = feval(param_script);
    case 'estimator'
        params = feval(param_script);
    case 'controller'
        params = feval(param_script);
end
end

