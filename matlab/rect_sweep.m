function [x, y] = rect_sweep(start, stop, width, interval)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
tic
measures = abs(start - stop);
a = 0;
% Switch so that always long side is on horizontal axis, we flip later
if(measures(1) < measures(2))
    a = 1;
    measures(2) = measures(1);
    measures(1) = abs(start(2) - stop(2));
end
nh = ceil(measures(1) / interval);
nv = ceil(measures(2) / width);
if mod(nv,2) == 0
    nv = nv + 1;
end
x = zeros(1,nh*nv);
y = zeros(1,nh*nv);
for i=1:nv
    if mod(i,2) == 1
        x(1+(i-1)*nh:i*nh)=linspace(0,measures(1),nh);
    else
        x(1+(i-1)*nh:i*nh)=linspace(measures(1),0,nh);
    end
    y(1+(i-1)*nh:i*nh) = ones(1,nh) * (i-1) * measures(2) / (nv-1);
end
if a == 1
    temp = x;
    x = y;
    y = temp;
end
if(start(1) < stop(1) && start(2) < stop(2))
    x = start(1) + x;
    y = start(2) + y;
elseif(start(1) < stop(1) && ~(start(2) < stop(2)))
    x = start(1) + x;
    y = start(2) - y;
elseif(~(start(1) < stop(1)) && start(2) < stop(2))
    x = start(1) - x;
    y = start(2) + y;
else
    x = start(1) - x;
    y = start(2) - y;
end
toc
end

