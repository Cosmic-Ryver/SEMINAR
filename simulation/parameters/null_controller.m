function [ controller ] = null_controller()

controller.actuators.actuate = @(controller) zeros(3,1);
controller.actuators.rwheels.initial_momentum = [];
controller.control_signal = [0;0;0;1];
controller.update = @(controller,estimator,params) controller;

end