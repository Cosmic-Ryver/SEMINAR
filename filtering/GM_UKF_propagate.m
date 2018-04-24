function [ output_args ] = GM_UKF_propagate( m, P, Q, f, Fx, Fq, varargin )

N = length(m);

X  = NaN(2*N + 1,1);
Wm = NaN(2*N + 1,1);
Wc = NaN(2*N + 1,1);

lambda = alpha^2*(N + kappa) - N;

sqrtP = sqrtm(P);
X(1) = m;
for i = 1:n
    term = sqrt(n + lambda)*sqrtP(:,i);
    X(1 + i)   = m + term;
    X(1 + 2*i) = m - term;
end

Wm(1)           = lambda/(N + lambda);
Wm(2:(1 + 2*N)) = 1/(2*(N + lambda)) + zeros(2*N,1);

Wc(1)           = Wm(1) + (1 - alpha^2 + beta);
Wc(2:(1 + 2*N)) = Wm(2:(1 + 2*N));

end

