function [x] = interior_point_optim(fcn, bndFcns, x0, varargin)
%NEWTON_RAPHSON 

maxiter = 1000;
iter    = 0;

n       = length(x0);
m       = length(bndFcns);
dX      = 1e-5*ones(n,1);
tol     = 1e-8*ones(n,1);
mu      = 0.1;
lambda0 = zeros(m,1);
c       = zeros(m,1);
err     = delBoundaryFunction(fcn,bndFcns,x0,mu,dX,varargin{:});
x       = x0;
while any(err > tol)
    
    iter = iter + 1;
    if iter > maxiter
        error('Could not optimize function within iteration limit')
    end
    
    lambdaold = lambda0;
    xold = x0;
    x0 = x;
    for i = 1:m
        bndFcni   = bndFcns{i};
        c(i)      = bndFcni(x0,varargin{:});
        lambda0(i) = mu/bndFcni(x0,varargin{:});
    end
    g      = gradient(fcn,x0,dX,varargin{:});
    A      = jacobian(bndFcns,x0,dX,varargin{:});
    W      = hessian(@(X,varargin)...
                boundaryFunction(fcn,bndFcns,X,mu,varargin{:}),...
                x0,dX,varargin{:});
    LAMBDA = diag(lambda0);
    C      = diag(c);
    
    p = [W, -A'; LAMBDA*A, C]\[-g + A'*lambda0; mu - C*lambda0];
    
    px      = p(1:n);
    plambda = p((n+1):end);
    
    alpha  = 1;
    lambda = lambda0 + alpha*plambda;
    if any(lambda < 0)
        [low, idx] = min(lambda);
        alpha = (plambda(idx) - low + 1e-10)/plambda(idx);
        lambda = lambda0 + alpha*plambda;
        if any(lambda < 0)
            keyboard;
        end
    end
    x = x0 + alpha*px;
    
    err = delBoundaryFunction(fcn,bndFcns,x0,mu,dX,varargin{:});
end

end

function [out] = boundaryFunction(fcn,bndFcns,x,mu,varargin)

m = length(bndFcns);

total = 0;
for i = 1:m
    bndFcni = bndFcns{i};
    total = total + log(bndFcni(x,varargin{:}));
end

out = fcn(x,varargin{:}) - mu*total;

end

function [out] = delBoundaryFunction(fcn,bndFcns,x,mu,dX,varargin)

m = length(bndFcns);

total = zeros(length(x),1);
for i = 1:m
    bndFcni = bndFcns{i};
    delci = gradient(bndFcni,x,dX,varargin{:})/...
        bndFcni(x,varargin{:});
    delci(isnan(delci)) = 0;
    total = total + delci;
end

out = gradient(fcn,x,dX,varargin{:}) - mu*total;

end

function [DelX] = gradient(fcn,X,dX,varargin)
    N    = length(X);
    DelX = NaN(N,1);
    for i = 1:N
        DelX(i) = centralDifference(fcn,X,i,dX,varargin{:});
    end
end

function [J] = jacobian(fcns,x,dX,varargin)

m = length(fcns);
n = length(x);
J = zeros(m,n);
for i = 1:m
    fcni = fcns{i};
    J(i,:) = gradient(fcni,x,dX,varargin{:})';
end

end

function [H] = hessian(fcn,X,dX,varargin)
    N = length(X);
    H = zeros(N);
    for i = 1:N
        DXiFcn = @(X, varargin) centralDifference(fcn,X,i,dX,varargin{:});
        for j = 1:(i-1)
            H(i,j) = centralDifference(DXiFcn,X,j,dX,varargin{:});
        end
        H(i,i) = centralDifference(DXiFcn,X,i,dX,varargin{:});
    end
    H = H + tril(H)';
end

function [DX] = centralDifference(fcn,X,idx,dX,varargin)
N = length(X);
mask      = zeros(N,1);
mask(idx) = 1;
dXi       = mask.*dX;
Xu        = X + dXi/2;
Xl        = X - dXi/2;
DX        = (fcn(Xu,varargin{:}) - fcn(Xl,varargin{:}))/dXi(idx);
end