function [Ls_b] = minimax_max(W,L_b,H_w,kappa)

% preliminaries
n        = size(W,2);
xWiWj    = cell(n,1);
vij      = cell(n,1);
sijk     = cell(n,1);
denom    = cell(n,1);
wij      = cell(n,1);
prev_max = 0;

% determine which two wheels will define the momentum polyhedron face
%   normal (these wheels will have unique vector magnitudes, while all
%   others will share the same magnitude)
for i = 1:n
    xWiWj{i} = cell(i-1,1);
    vij{i}   = cell(i-1,1);
    sijk{i}  = cell(i-1,1);
    denom{i} = cell(i-1,1);
    wij{i}   = cell(i-1,1);
    for j = 1:(i-1)
        sijk{i}{j} = cell(n,1);
        xWiWj{i}{j} = cross(W(:,i),W(:,j));
        vij{i}{j}   = zeros(3,1);
        for k = 1:n
            if (k ~= i) && (k ~= j)
                sijk{i}{j}{k} = dot(xWiWj{i}{j},W(:,k));
                vij{i}{j}     = vij{i}{j} + W(:,k)*sign(sijk{i}{j}{k});
            end
        end
        denom{i}{j} = dot(xWiWj{i}{j},vij{i}{j});
        wij{i}{j} = xWiWj{i}{j}/denom{i}{j};
        test_val = abs(dot(wij{i}{j},L_b));
        if test_val > prev_max
            iIdx = i;
            jIdx = j;
            prev_max = test_val;
        end
    end
end

% Generate transformation matrix from body vector to wheel magnitudes
Wjk = zeros(n,3);
for k = 1:n
    if k == iIdx
        Wjk(k,:) = cross(W(:,jIdx),vij{iIdx}{jIdx})'/denom{iIdx}{jIdx};
    elseif k == jIdx
        Wjk(k,:) = cross(vij{iIdx}{jIdx},W(:,iIdx))'/denom{iIdx}{jIdx};
    else
        Wjk(k,:) = wij{iIdx}{jIdx}'*sign(sijk{iIdx}{jIdx}{k});
    end
end

% preliminaries
H_b      = W*H_w;
xWiWj    = cell(n,1);
vij      = cell(n,1);
sijk     = cell(n,1);
denom    = cell(n,1);
wij      = cell(n,1);
prev_max = 0;

% determine which two wheels will define the momentum polyhedron face
%   normal (these wheels will have unique vector magnitudes, while all
%   others will share the same magnitude)
for i = 1:n
    xWiWj{i} = cell(i-1,1);
    vij{i}   = cell(i-1,1);
    sijk{i}  = cell(i-1,1);
    denom{i} = cell(i-1,1);
    wij{i}   = cell(i-1,1);
    for j = 1:(i-1)
        sijk{i}{j} = cell(n,1);
        xWiWj{i}{j} = cross(W(:,i),W(:,j));
        vij{i}{j}   = zeros(3,1);
        for k = 1:n
            if (k ~= i) && (k ~= j)
                sijk{i}{j}{k} = dot(xWiWj{i}{j},W(:,k));
                vij{i}{j}     = vij{i}{j} + W(:,k)*sign(sijk{i}{j}{k});
            end
        end
        denom{i}{j} = dot(xWiWj{i}{j},vij{i}{j});
        wij{i}{j} = xWiWj{i}{j}/denom{i}{j};
        test_val = abs(dot(wij{i}{j},H_b));
        if test_val > prev_max
            iIdx = i;
            jIdx = j;
            prev_max = test_val;
        end
    end
end

% Generate transformation matrix from body vector to wheel magnitudes
Wij = zeros(n,3);
for k = 1:n
    if k == iIdx
        Wij(k,:) = cross(W(:,jIdx),vij{iIdx}{jIdx})'/denom{iIdx}{jIdx};
    elseif k == jIdx
        Wij(k,:) = cross(vij{iIdx}{jIdx},W(:,iIdx))'/denom{iIdx}{jIdx};
    else
        Wij(k,:) = wij{iIdx}{jIdx}'*sign(sijk{iIdx}{jIdx}{k});
    end
end

% Transform body vector into wheel magnitudes
L_w = Wjk*L_b + kappa*(Wij*W - eye(n))*H_w;

% Apply wheel magnitudes to body pointing directions
Ls_b = repmat(L_w',3,1).*W;

end