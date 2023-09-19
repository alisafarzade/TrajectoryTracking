clc
clear
le1 = 0.35;
k1 = 1.3;


dt = 0.01;
tspan = 0:dt:2*pi;

xc1d = 0.3 * cos(tspan);
yc1d = 0.3 * sin(tspan);
xc1 = 0.3 * cos(tspan)+0.4;
yc1 = 0.3 * sin(tspan)-0.1;

theta1d = unwrap(atan2(cos(tspan), -sin(tspan)));

x12d = xc1d .* cos(theta1d) + yc1d .* sin(theta1d);
x13d = xc1d .* sin(theta1d) - yc1d .* cos(theta1d);
x12 = xc1 .* cos(theta1d) + yc1 .* sin(theta1d);
x13 = xc1 .* sin(theta1d) - yc1 .* cos(theta1d);

z11 = x13d - x13;
alpha1 = x12d + ((le1^2 - z11.^2) * k1 .* z11);
    
z12 = alpha1 - x12;

A = 0.35^2 - z11.^2;
B = 0.35^2 - z12.^2;

figure(1);
subplot(3, 3, 1);
plot(xc1d, yc1d,'.-r', xc1, yc1,'b');
grid on
title('xc yc');

subplot(3, 3, 2);
plot(x12d, x13d, '.-r', x12, x13, 'b');
grid on
title('x2 x3');

subplot(3, 3, 3);
plot(tspan, x12d, '.-r', tspan, x12, 'b');
grid on
title('x2d x2');

subplot(3, 3, 4);
plot(tspan, x13d, '.-r', tspan, x13, 'b');
grid on
title('x3d x3');


subplot(3, 3, 5);
plot(tspan, z11, '.-r');
grid on
title('z1');

subplot(3, 3, 6);
plot(tspan, z12, '.-r');
grid on
title('z2');

subplot(3, 3, 7);
plot(tspan, A, '.-r');
grid on
title('A');

subplot(3, 3, 8);
plot(tspan, B, '.-r');
grid on
title('B');

subplot(3, 3, 9);
plot(tspan, B./A, '.-r');
grid on
title('B/A');
