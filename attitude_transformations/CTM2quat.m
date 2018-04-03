function quat = CTM2quat(CTM)
% CTM2quat convert coordinate transformation matrix to quaternion
%   represented transformation
%
% Gus Buonviri, 1/28/18
% Mississippi State University
%
% INPUTS:
%
%   CTM = [3 x 3] {numeric, array} coordinate transformation matrix.
%
% OUTPUTS:
%
%   quat = [4 x 1] {numeric, column vector} quaternion represented
%       transformation, where the vector precedes the scalar.
%
%

trCTM = trace(CTM);

% idx determines method that minimizes numerical error
[~, idx] = max([CTM(1,1), CTM(2,2), CTM(3,3), trCTM]);

if idx == 1
    qIdxQuat = 0.25*[1 + 2*CTM(1,1) - trCTM;
                        CTM(1,2) + CTM(2,1);
                        CTM(1,3) + CTM(3,1);
                        CTM(2,3) - CTM(3,2)];
elseif idx == 2
    qIdxQuat = 0.25*[   CTM(2,1) + CTM(1,2);
                     1 + 2*CTM(2,2) - trCTM;
                        CTM(2,3) + CTM(3,2);
                        CTM(3,1) - CTM(1,3)];
elseif idx == 3
    qIdxQuat = 0.25*[   CTM(3,1) + CTM(1,3);
                        CTM(3,2) + CTM(2,3);
                     1 + 2*CTM(3,3) - trCTM;
                        CTM(1,2) - CTM(2,1)];
else
    qIdxQuat = 0.25*[CTM(2,3) - CTM(3,2);
                     CTM(3,1) - CTM(1,3);
                     CTM(1,2) - CTM(2,1);
                               1 + trCTM];
end

qIdx = sqrt(qIdxQuat(idx));

quat = qIdxQuat/qIdx;

quat = quat/norm(quat);

end