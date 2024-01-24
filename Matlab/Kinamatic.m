clc

ls1 = 0.55;
ls2 = 1.35;
ls31 = 2.75;
ls32 = 1.65;

le1 = 0.35;
le2 = 0.35;
le31 = 1.5;
le32 = 1.2;

k1 = 1.5;
k2 = 1.5;
k3 = 1.3;
k41 = 2;
k42 = 2;
Kv1 = diag([50 50]);
ks1 = 0.1;



dt = 0.0001;
tspan = 0:dt:200;


%% i = 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xc1d = 0.8 * cos(tspan);
yc1d = sin(tspan);
theta1d = unwrap(atan2(cos(tspan), -0.8 * sin(tspan)));

% v1d = sqrt((-0.8*sin(tspan)).^2 + (cos(tspan)).^2);
w1d = (-sin(tspan).*(-0.8 * sin(tspan)) - (-0.8 * cos(tspan)).*cos(tspan))./((-0.8*sin(tspan)).^2 + (cos(tspan)).^2);

x11d = theta1d;
x12d = xc1d .* cos(theta1d) + yc1d .* sin(theta1d);
x13d = xc1d .* sin(theta1d) - yc1d .* cos(theta1d);

%% i = 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xc2d = cos(tspan);
yc2d = 0.8 * sin(tspan);
theta2d = unwrap(atan2(0.8 * cos(tspan), -sin(tspan)));

% v2d = sqrt((-sin(tspan)).^2 + (0.8*cos(tspan)).^2);
w2d = (-0.8 * sin(tspan).*(-sin(tspan)) - (-cos(tspan)).*(0.8 * cos(tspan)))./((-sin(tspan)).^2 + (0.8*cos(tspan)).^2);

x21d = theta2d;
x22d = xc2d .* cos(theta2d) + yc2d .* sin(theta2d);
x23d = xc2d .* sin(theta2d) - yc2d .* cos(theta2d);

%% i = 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xc3d = 1.2 * cos(tspan);
yc3d = sin(tspan);
theta3d = unwrap(atan2(cos(tspan), -1.2 * sin(tspan)));

% v3d = sqrt((-1.2*sin(tspan)).^2 + (cos(tspan)).^2);
w3d = (-sin(tspan).*(-1.2 * sin(tspan)) - (-1.2 * cos(tspan)).*cos(tspan))./((-1.2*sin(tspan)).^2 + (cos(tspan)).^2);

x31d = theta3d;
x32d = xc3d .* cos(theta3d) + yc3d .* sin(theta3d);
x33d = xc3d .* sin(theta3d) - yc3d .* cos(theta3d);



Xi11 = zeros(1, length(tspan));
z11 = zeros(1, length(tspan));
alpha1 = zeros(1, length(tspan));
z12 = zeros(1, length(tspan));
Xi12 = zeros(1, length(tspan));
v1 = zeros(1, length(tspan));
w1 = zeros(1, length(tspan));
xc1 = zeros(1, length(tspan));
yc1 = zeros(1, length(tspan));
x11 = zeros(1, length(tspan));
x12 = zeros(1, length(tspan));
x13 = zeros(1, length(tspan));

Xi21 = zeros(1, length(tspan));
z21 = zeros(1, length(tspan));
alpha2 = zeros(1, length(tspan));
z22 = zeros(1, length(tspan));
Xi22 = zeros(1, length(tspan));
v2 = zeros(1, length(tspan));
w2 = zeros(1, length(tspan));
xc2 = zeros(1, length(tspan));
yc2 = zeros(1, length(tspan));
x21 = zeros(1, length(tspan));
x22 = zeros(1, length(tspan));
x23 = zeros(1, length(tspan));


% i = 3
Xi31 = zeros(1, length(tspan));
z31 = zeros(1, length(tspan));
alpha3 = zeros(1, length(tspan));
z32 = zeros(1, length(tspan));
Xi32 = zeros(1, length(tspan));
v3 = zeros(1, length(tspan));
w3 = zeros(1, length(tspan));
xc3 = zeros(1, length(tspan));
yc3 = zeros(1, length(tspan));
x31 = zeros(1, length(tspan));
x32 = zeros(1, length(tspan));
x33 = zeros(1, length(tspan));

NoiseRatio = 1;
xNoise = randi([-NoiseRatio NoiseRatio],1,length(tspan))/100;
yNoise = randi([-NoiseRatio NoiseRatio],1,length(tspan))/100;
wNoise = randi([-NoiseRatio NoiseRatio],1,length(tspan))/100;


x11(1) = pi/2-0.2;
x12(1) = 0;
x13(1) = 1;

yc1(1) = 0;
xc1(1) = 1;
Xi11(1) = 0;
Xi12(1) = 0;
alpha1(1) = 0;

% i = 2
x21(1) = pi/2-0.2;
x22(1) = 0;
x23(1) = 1.2;
x23dot(1) = 0;

yc2(1) = 0;
xc2(1) = 1.2;
Xi21(1) = 0;
Xi22(1) = 0;
alpha2(1) = 0;


% i = 3
x31(1) = pi/2-0.2;
x32(1) = 0;
x33(1) = 1.4;
x33dot(1) = 0;

yc3(1) = 0;
xc3(1) = 1.4;
Xi31(1) = 0;
Xi32(1) = 0;
alpha3(1) = 0;



for q = 1 : length(tspan)

    Xi11(q + 1) = w1d(q) + (k3 * (x11d(q) - x11(q)));
    Xi21(q + 1) = w2d(q) + (k3 * (x21d(q) - x21(q)));
    Xi31(q + 1) = w3d(q) + (k3 * (x31d(q) - x31(q)));
    
    z11(q) = x13d(q) - x13(q);
    z21(q) = x23d(q) - x23(q);
    z31(q) = x33d(q) - x33(q);
    alpha1(q + 1) = x12d(q) + ((le1^2 - z11(q)^2) * k1 * z11(q) * w1d(q));
    alpha2(q + 1) = x22d(q) + ((le1^2 - z21(q)^2) * k1 * z21(q) * w2d(q));
    alpha3(q + 1) = x32d(q) + ((le1^2 - z31(q)^2) * k1 * z31(q) * w3d(q));
    
    z12(q) = alpha1(q + 1) - x12(q);
    z22(q) = alpha2(q + 1) - x22(q);
    z32(q) = alpha3(q + 1) - x32(q);
    
    Xi12(q + 1) = ((alpha1(q + 1) - alpha1(q))/dt) + (le2^2 - z12(q)^2) * k2 * z12(q) * w1d(q)^2 + ((le2^2 - z12(q)^2)/(le1^2 - z11(q)^2)) * z11(q) * w1d(q);
    Xi22(q + 1) = ((alpha2(q + 1) - alpha2(q))/dt) + (le2^2 - z22(q)^2) * k2 * z22(q) * w2d(q)^2 + ((le2^2 - z22(q)^2)/(le1^2 - z21(q)^2)) * z21(q) * w2d(q);
    Xi32(q + 1) = ((alpha3(q + 1) - alpha3(q))/dt) + (le2^2 - z32(q)^2) * k2 * z32(q) * w3d(q)^2 + ((le2^2 - z32(q)^2)/(le1^2 - z31(q)^2)) * z31(q) * w3d(q);

    
    v1(q) = (x13(q) * Xi11(q + 1)) + Xi12(q + 1);
    v2(q) = (x23(q) * Xi21(q + 1)) + Xi22(q + 1);
    v3(q) = (x33(q) * Xi31(q + 1)) + Xi32(q + 1);
    w1(q) = Xi11(q + 1);
    w2(q) = Xi21(q + 1);
    w3(q) = Xi31(q + 1);
    
    xc1(q + 1) = xc1(q) + dt * v1(q) * cos(x11(q));%+ xNoise(q);
    xc2(q + 1) = xc2(q) + dt * v2(q) * cos(x21(q));%+ xNoise(q);
    xc3(q + 1) = xc3(q) + dt * v3(q) * cos(x31(q));%+ xNoise(q);
    yc1(q + 1) = yc1(q) + dt * v1(q) * sin(x11(q));%+ yNoise(q);    
    yc2(q + 1) = yc2(q) + dt * v2(q) * sin(x21(q));%+ yNoise(q);    
    yc3(q + 1) = yc3(q) + dt * v3(q) * sin(x31(q));%+ yNoise(q);    
    x11(q + 1) = x11(q) + dt * w1(q);% + wNoise(q);
    x21(q + 1) = x21(q) + dt * w2(q);% + wNoise(q);
    x31(q + 1) = x31(q) + dt * w3(q);% + wNoise(q);
    
    
    x12(q + 1) = xc1(q + 1) * cos(x11(q + 1)) + yc1(q + 1) * sin(x11(q + 1));
    x22(q + 1) = xc2(q + 1) * cos(x21(q + 1)) + yc2(q + 1) * sin(x21(q + 1));
    x32(q + 1) = xc3(q + 1) * cos(x31(q + 1)) + yc3(q + 1) * sin(x31(q + 1));
    x13(q + 1) = xc1(q + 1) * sin(x11(q + 1)) - yc1(q + 1) * cos(x11(q + 1));
    x23(q + 1) = xc2(q + 1) * sin(x21(q + 1)) - yc2(q + 1) * cos(x21(q + 1));
    x33(q + 1) = xc3(q + 1) * sin(x31(q + 1)) - yc3(q + 1) * cos(x31(q + 1));
    
end


% figure(1);
% plot(tspan, x11(1:length(x11)-1), 'red', tspan, x11d, 'blue');
% grid on
% title('x11 and x11d');
% legend('x11', 'x11d');



figure(1);
subplot(1, 2, 1);
plot(x12, x13,'.-r', x12d, x13d, 'blue');
grid on
title('homeomorphism mapped');
xlabel('x12 (m)')
ylabel('x13 (m)')
legend('actual', 'desired');

subplot(1, 2, 2);
plot(xc1, yc1, '.-r', xc1d, yc1d, 'blue');
grid on
title('real coordinates');
xlim([-1.5 1.5]);
ylim([-1.5 1.5]);
xlabel('x (m)')
ylabel('y (m)')
legend('actual', 'desired');

figure(2);
subplot(1, 2, 1);
plot(x22, x23,'.-r', x22d, x23d, 'blue');
grid on
title('homeomorphism mapped');
xlabel('x22 (m)')
ylabel('x23 (m)')
legend('actual', 'desired');


subplot(1, 2, 2);
plot(xc2, yc2, '.-r', xc2d, yc2d, 'blue');
grid on
title('real coordinates');
xlim([-1.5 1.5]);
ylim([-1.5 1.5]);
xlabel('x (m)')
ylabel('y (m)')
legend('actual', 'desired');

figure(3);
subplot(1, 2, 1);
plot(x32, x33,'.-r', x32d, x33d, 'blue');
grid on
title('homeomorphism mapped');
xlabel('x32 (m)')
ylabel('x33 (m)')
legend('actual', 'desired');

subplot(1, 2, 2);
plot(xc3, yc3, '.-r', xc3d, yc3d, 'blue');
grid on
title('real coordinates');
xlim([-1.5 1.5]);
ylim([-1.5 1.5]);
xlabel('x (m)')
ylabel('y (m)')
legend('actual', 'desired');

% figure(1);
% subplot(1, 2, 1);
% plot(x12d, x13d, 'blue');
% grid on
% title('homeomorphism mapped');
% 
% 
% subplot(1, 2, 2);
% plot(xc1d, yc1d, 'blue');
% grid on
% title('real coordinates');

xc1e = xc1(1:length(xc1)-1) - xc1d;
yc1e = yc1(1:length(yc1)-1) - yc1d;

xc2e = xc2(1:length(xc2)-1) - xc2d;
yc2e = yc2(1:length(yc2)-1) - yc2d;

xc3e = xc3(1:length(xc3)-1) - xc3d;
yc3e = yc3(1:length(yc3)-1) - yc3d;

figure(4);
subplot(3, 2, 1);
plot(tspan, xc1e(1:length(tspan)));
grid on
title('xc1e');
ylim([-0.1 0.25]);
xlabel('time (s)')
ylabel('x Error (m)')

subplot(3, 2, 2);
plot(tspan, yc1e(1:length(tspan)));
grid on
title('yc1e');
ylim([-0.1 0.05]);
xlabel('time (s)')
ylabel('y Error (m)')

subplot(3, 2, 3);
plot(tspan, xc2e(1:length(tspan)));
grid on
title('xc2e');
ylim([-0.1 0.25]);
xlabel('time (s)')
ylabel('x Error (m)')

subplot(3, 2, 4);
plot(tspan, yc2e(1:length(tspan)));
grid on
title('yc2e');
ylim([-0.1 0.05]);
xlabel('time (s)')
ylabel('y Error (m)')

subplot(3, 2, 5);
plot(tspan, xc3e(1:length(tspan)));
grid on
title('xc3e');
ylim([-0.1 0.25]);
xlabel('time (s)')
ylabel('x Error (m)')

subplot(3, 2, 6);
plot(tspan, yc3e(1:length(tspan)));
grid on
title('yc3e');
ylim([-0.1 0.05]);
xlabel('time (s)')
ylabel('y Error (m)')

figure(5);
plot(tspan, x11d(1:length(tspan)), tspan, x11(1:length(tspan)));
grid on
title('Theta');
legend( 'desired', 'actual');
xlim([0 10]);
xlabel('time (s)')
ylabel('Theta (rad)')

figure(6);
plot(tspan, x21d(1:length(tspan)), tspan, x21(1:length(tspan)));
grid on
title('Theta');
legend( 'desired', 'actual');
xlim([0 10]);
xlabel('time (s)')
ylabel('Theta (rad)')

figure(7);
plot(tspan, x31d(1:length(tspan)), tspan, x31(1:length(tspan)));
grid on
title('Theta');
legend( 'desired', 'actual');
xlim([0 10]);
xlabel('time (s)')
ylabel('Theta (rad)')

V = max(max(v1), abs(min(v1)))
W = max(max(w1), abs(min(w1)))

((V + (0.13*W))/0.06) * 9.55






