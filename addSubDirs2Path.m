% addSubDirs2Path add the subdirectories of the repository to the matlab
%   path

oldDir = pwd;
newDir = fileparts(which('addSubDirs2Path.m'));
cd(newDir);
charList = ls;

for i = 1:size(charList,1)
    item = deblank(charList(i,:));
    
    if (~isequal(item,'.') && ~isequal(item,'..') && ~isequal(item,'.git'))
        if exist(item,'dir') == 7
            addpath(item);
        end
    end
end

cd(oldDir);

clear oldDir newDir charList i item