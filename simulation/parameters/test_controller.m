function [ controller ] = test_controller()

controller.actuator = @(controller) controller.signal;
controller.signal = 0;
controller.update = @(controller,estimator,params) ...
    setfield(controller,'signal',hybrid(estimator.estimate.state(7:10),...
    estimator.estimate.state(11:13),params.physical.J,...
    ea2quat([pi;pi*3/8;pi]),zeros(3,1),zeros(3,1),1,1));

end