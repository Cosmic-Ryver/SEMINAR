function [Vecs_b] = minimax_min(W,Vec_b)

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
        test_val = abs(dot(wij{i}{j},Vec_b));
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
Vec_w = Wij*Vec_b;

% Apply wheel magnitudes to body pointing directions
Vecs_b = repmat(Vec_w',3,1).*W;



end