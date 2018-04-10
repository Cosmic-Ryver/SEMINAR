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
physical_param_script    = 'std_physical_params.m';
simulation_param_script  = 'std_simulation_params.m';
environment_param_script = 'std_environment_params';
kin_dyn_engine_script    = 'simple_physics_engine';
controller_script        = 'std_controller';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Create simulation object
simulation = attitude_simulation(simulation_param_script, ...
    orbital_param_script, physical_param_script, ...
    environment_param_script, kin_dyn_engine_script, ...
    controller_script);

%% Set control attitude

% scan orientations
latt = [zeros(1,13) + (90-36)*pi/180, zeros(1,13) + (-90+36)*pi/180];
long = [0:360/13:360*12/13, 0:360/13:360*12/13]*pi/180;
q_c = zeros(4,26);
for i = 1:26
    q_c(:,i) = quat_prod(ea2quat([-23.14*pi/180;0;long(i)]),ea2quat([0;latt(i);0]));
end

simulation.setControlSignal(quat_inv(q_c(:,11)),zeros(3,1),zeros(3,1));

%% Execute the integration
[t,X] = ode45(@(t,x)simulation.odeFcn(t,x),...
    [simulation.params.simulation.ti simulation.params.simulation.te], ...
    simulation.initialState, simulation.params.simulation.odeOpts);