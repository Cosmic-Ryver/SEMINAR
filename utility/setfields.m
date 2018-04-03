function [ s ] = setfields( s, varargin )
%SETFIELDS multiple setfield calls

for i = 1:(length(varargin)/2)
    s = setfield(s,varargin{i*2 - 1},varargin{i*2});
end
end

