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
orbital_param_script     = 'trk_orbital_params.m';
physical_param_script    = 'std_physical_params.m';
simulation_param_script  = 'short_trk_simulation_params.m';
environment_param_script = 'std_environment_params';
kin_dyn_engine_script    = 'simple_physics_engine';
controller_param_script  = 'trk_controller.m';
estimator_type_enum      = estimator_type_enumeration.estimator;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Create simulation object
simulation = attitude_simulation(simulation_param_script, ...
    orbital_param_script, physical_param_script, ...
    environment_param_script, kin_dyn_engine_script, ...
    controller_param_script, estimator_type_enum);

%% Execute the integration
simulation.executeMulti();