function [X,W,Wc] = getSigmaPoints(m,P,lambda)

N = length(m);

X  = NaN(N,2*N + 1);
W  = NaN(1,2*N + 1);

sqrtP = sqrtm((N + lambda)*P);
X(:,1) = m;
X(:,2:(N+1))   = m + sqrtP;
X(:,(N+2):end) = m - sqrtP;

W(1)     = lambda/(N + lambda);
W(2:end) = 1/(2*(N + lambda)) * ones(1,2*N);

end

