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
k3 = 0.5;
k41 = 2;
k42 = 2;
Kv1 = diag([50 50]);
ks1 = 0.1;



dt = 0.2;
tspan = 0:dt:200;


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



xc1d = 0.8 * sin(tspan);
yc1d = 0.3 * sin(2*tspan);
theta1d = unwrap(atan2(0.6 * cos(2.*tspan), 0.8 * cos(tspan)));

% v1d = sqrt((-0.8*sin(tspan)).^2 + (cos(tspan)).^2);
% w1d = (-sin(tspan).*(-0.8 * sin(tspan)) - (-0.8 * cos(tspan)).*cos(tspan))./((-0.8*sin(tspan)).^2 + (cos(tspan)).^2);
w1d = -((24*cos(tspan).*sin(2.*tspan) - 12*sin(tspan).*cos(2.*tspan))) ./ (9*cos(2.*tspan).^2+16*cos(tspan).^2);

x11d = theta1d;
x12d = xc1d .* cos(theta1d) + yc1d .* sin(theta1d);
x13d = xc1d .* sin(theta1d) - yc1d .* cos(theta1d);



NoiseRatio = 1;
xNoise = randi([-NoiseRatio NoiseRatio],1,length(tspan))/100;
yNoise = randi([-NoiseRatio NoiseRatio],1,length(tspan))/100;
wNoise = randi([-NoiseRatio NoiseRatio],1,length(tspan))/100;


x11(1) = pi/4-0.1;
x12(1) = 0;
x13(1) = 0;

yc1(1) = 0;
xc1(1) = 0;
Xi11(1) = 0;
Xi12(1) = 0;
alpha1(1) = 0;


for q = 1 : length(tspan)

    Xi11(q + 1) = w1d(q) + (k3 * (x11d(q) - x11(q)));
    
    z11(q) = x13d(q) - x13(q);
    alpha1(q + 1) = x12d(q) + ((le1^2 - z11(q)^2) * k1 * z11(q) * w1d(q));
    
    z12(q) = alpha1(q + 1) - x12(q);
    
    Xi12(q + 1) = ((alpha1(q + 1) - alpha1(q))/dt) + (le2^2 - z12(q)^2) * k2 * z12(q) * w1d(q)^2 + ((le2^2 - z12(q)^2)/(le1^2 - z11(q)^2)) * z11(q) * w1d(q);

    
    v1(q) = (x13(q) * Xi11(q + 1)) + Xi12(q + 1);
    w1(q) = Xi11(q + 1);
    
    xc1(q + 1) = xc1(q) + dt * v1(q) * cos(x11(q)) + xNoise(q);
    yc1(q + 1) = yc1(q) + dt * v1(q) * sin(x11(q)) + yNoise(q);    
    x11(q + 1) = x11(q) + dt * w1(q);% + wNoise(q);
    
    
    x12(q + 1) = xc1(q + 1) * cos(x11(q + 1)) + yc1(q + 1) * sin(x11(q + 1));
    x13(q + 1) = xc1(q + 1) * sin(x11(q + 1)) - yc1(q + 1) * cos(x11(q + 1));
    
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


subplot(1, 2, 2);
plot(xc1, yc1, '.-r', xc1d, yc1d, 'blue');
grid on
title('real coordinates');


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

figure(2);
subplot(1, 2, 1);
plot(tspan, xc1e(1:length(tspan)));
grid on
title('xc1e');

subplot(1, 2, 2);
plot(tspan, yc1e(1:length(tspan)));
grid on
title('yc1e');

V = max(max(v1), abs(min(v1)))
W = max(max(w1), abs(min(w1)))

((V + (0.13*W))/0.06) * 9.55





