function [ s ] = setfields( s, varargin )
%SETFIELDS multiple setfield calls

for i = 1:(length(varargin)/2)
    if iscell(varargin{i*2 - 1})
        str2eval = 'setfield(s,';
        for j = 1:length(varargin{i*2 - 1})
            str2set = sprintf('{1},varargin{i*2 - 1}{%f},',j);
            str2eval = [str2eval str2set];
        end
        str2eval = [str2eval 'varargin{i*2})'];
        s = eval(str2eval);
    else
        s = setfield(s,varargin{i*2 - 1},varargin{i*2});
    end
end
end

