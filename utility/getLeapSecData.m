function [ YMD, jd ] = getLeapSecData()
% GETLEAPSECDATA retrieve dates to which leap seconds were added

persistent YMDint jdint

if isempty(YMDint) % check persistent variables

    thisDir         = fileparts(which('getLeapSecData.m'));
    leapSecDataPath = [thisDir filesep 'data' filesep 'LeapSec.mat'];
    if exist(leapSecDataPath,'file') == 7
        load(leapSecDataPath);
        [ty, tm, ~, ~, ~, ~] = datevec(now);
        if tm > 6
            tm = 6;
        else
            tm = 12;
            ty = ty - 1;
        end
        checkTime = datenum(ty,tm,0);
        if checkTime > timeStamp
            [YMD, jd] = netReadLeapSecData(leapSecDataPath);
        end
    else
        [YMD, jd] = netReadLeapSecData(leapSecDataPath);
    end
    
    YMDint = YMD;
    jdint  = jd;
        
else
    % output persistent variables
    YMD = YMDint;
    jd  = jdint;
    
end

end

function [YMD, jd] = netReadLeapSecData(dataFilePath)

charArray = urlread('http://maia.usno.navy.mil/ser7/tai-utc.dat');
strVec    = strsplit(charArray,'\n');
strVec    = strVec(1:(end - 1));

% month ascii
M = [74    65    78;
     70    69    66;
     77    65    82;
     65    80    82;
     77    65    89;
     74    85    78;
     74    85    76;
     65    85    71;
     83    69    80;
     79    67    84;
     78    79    86;
     68    69    67]';

N = length(strVec);
YMD = NaN(N,3);
jd  = NaN(N,1);
for i = 1:N
    A = sscanf(strVec{i},'%5g%4s%3g =JD %9g',26);
    YMD(i,1) = A(1);
    m = 1;
    while ~all(A(2:4) == M(:,m))
        m = m + 1;
        if m > 12
            error('Failed while parsing web data')
        end
    end
    YMD(i,2) = m;
    YMD(i,3) = A(5);
    jd(i)    = A(6);
end

timeStamp = now;
save(dataFilePath,'YMD','jd','timeStamp');

end