function [x,y] = spiral_sweep(center, outer, width, interval)
%UNTITLED2 Summary of this function goes here
% interval should be > width
% k = R / width
tic
shifted = outer - center;
shifted_th = atan(shifted(2)/shifted(1));
if(shifted(1) < 0)
    shifted_th = shifted_th+pi;
end
if(shifted_th < 0)
    shifted_th = shifted_th + 2*pi;
end
R = sqrt(sum(shifted.^2));
O = 2*R*pi/width;
alpha = width / (4*pi);
L = (O*sqrt(O^2+1) + log(O+sqrt(O^2+1)));

l = linspace(0,L,floor(alpha*L/interval) + 1);
theta = zeros(size(l));
delta = O / size(l,2);

for i=2:size(l,2)
    angle = theta(i-1)+delta;
    df = sqrt(angle^2+1);
    f = angle*df + log(angle + df) - l(i);
    c = 0;
    while abs(f) > 0.01
        angle = angle - f / (2*df);
        df = sqrt(angle^2+1);
        f = angle*df + log(angle + df) - l(i);
        c = c + 1;
    end
    theta(i) = angle;
    delta = theta(i) - theta(i-1);
end
shift = shifted_th - theta(end) + floor(theta(end)/(2*pi))*2*pi;
x = 2 * alpha * (theta) .* cos(theta + shift) + center(1);
y = 2 * alpha * (theta) .* sin(theta + shift) + center(2);
toc
end
