%% HEADER
%
% Main script for simulation execution.
%
% Gus Buonviri, 3/5/2018
% Mississippi State University
%
%
%% Clean workspace

clc; clear all; close all;

%% LOAD PARAMETERS
%
% All major simulation parameters should be specified in their own version
% controlled scripts. The relevant variables from these scripts should be
% loaded into the simulation via the appropriate functions.

%%%% PARAM SCRIPT NAMES %%%%
orbital_param_script     = 'std_orbital_params.m';
physical_param_script    = 'test_physical_params.m';
simulation_param_script  = 'test_simulation_params.m';
environment_param_script = 'std_environment_params';
kin_dyn_engine_script    = 'std_physics_engine';
estimator_script         = 'truth_estimator';
controller_script        = 'std_controller';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Create simulation object
simulation = attitude_simulation(simulation_param_script, ...
    orbital_param_script, physical_param_script, ...
    environment_param_script, kin_dyn_engine_script, ...
    estimator_script, controller_script);

%% Execute the integration
[t,X] = ode45(@(t,x)simulation.odeFcn(t,x),...
    [simulation.params.simulation.ti simulation.params.simulation.te], ...
    simulation.initialState, simulation.params.simulation.odeOpts);