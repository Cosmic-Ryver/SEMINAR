classdef estimator_type
    methods (Static = true)
        function [oEstimatorObj] = loadEstimator(aSimHandle,aEstimatorEnum)
            
            switch aEstimatorEnum
                case estimator_type_enumeration.estimator
                    oEstimatorObj = estimator(aSimHandle);
                case estimator_type_enumeration.calibrator
                    error('Not yet implemented')
                    oEstimatorObj = calibration_estimator(aSimHandle);
                case estimator_type_enumeration.ukf_estimator
                    oEstimatorObj = ukf_estimator(aSimHandle);
                otherwise
                    error('Bad estimator enum input')
            end
        end
    end
end

