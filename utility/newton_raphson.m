function [x] = newton_raphson(fcn, x0, opts, varargin)
%NEWTON_RAPHSON 

defaultOpts.dX     = abs(x0*1e-15) + 1e-15;
defaultOpts.dALPHA = 1e-15;

if isempty(opts)
    dX     = defaultOpts.dX;
    dALPHA = defaultOpts.dALPHA;
else
    dX     = opts.dX;
    dALPHA = opts.dALPHA;
end

itmax  = 2;
DelF   = @(X,varargin) gradient(fcn,X,dX,varargin{:});
H      = @(X,varargin) hessian(fcn,X,dX,varargin{:});
x      = x0;
for k = 1:itmax
    
    x0    = x;
    h     = -H(x0,varargin{:})\DelF(x0,varargin{:});
    alpha = fzero(@(ALPHA) (fcn(x0 + (ALPHA + dALPHA/2)*h, varargin{:}) - ...
        fcn(x0 + (ALPHA - dALPHA/2)*h, varargin{:}))/dALPHA,10);
    alpha = double(max(alpha));
    
    if alpha<0
        disp('Error: negative step size')
        break
    end
    
    x = x0 + alpha*h;
end

end

function [DelX] = gradient(fcn,X,dX,varargin)
    N    = length(X);
    DelX = NaN(N,1);
    for i = 1:N
        DelX(i) = centralDifference(fcn,X,i,dX,varargin{:});
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