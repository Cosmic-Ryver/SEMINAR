function [ om_z ] = biased_gyro( om_tru, sigma_v, beta_tru )
% biased_gyro noisey gyro with bias

om_z = om_tru + sigma_v*randn(3,1) + beta_tru;

end