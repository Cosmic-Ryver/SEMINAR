function [ controller ] = null_controller()

% controller.actuator = @(controller) controller.actuator_signal;
controller.actuator = @(controller) zeros(3,1);
controller.actuator_signal = 0;
controller.control_signal = ea2quat([pi;pi*3/8;pi]);
controller.update = @(controller,estimator,params) ...
    setfield(controller,'actuator_signal',-hybrid(estimator.estimate.state(7:10),...
    estimator.estimate.state(11:13),params.physical.J,...
    controller.control_signal,zeros(3,1),zeros(3,1),1,1));

end