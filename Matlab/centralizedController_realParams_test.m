clc
clear


ls1 = 0.55;
ls2 = 1.35;
ls31 = 2.75;
ls32 = 1.65;

le1 = 0.35;
le2 = 0.35;
le31 = 1.5;
le32 = 1.2;

k1 = 1.3;
k2 = 1.5;
k3 = 1.5;
k41 = 0.5;
k42 = 0.5;
Kv1 = diag([20 20]);
ks1 = 0.1;
GAMAofW = diag(50*ones(2160,1));

RHO = 2
GAMAofbeta = [0 , 0;
                0 , 0];
neurons = 2160;
centers = cartesian(linspace(-1.5, 1.5, 4), linspace(-2, 2, 5), linspace(0, 2, 3), linspace(-1.5, 1.5, 4), linspace(-1, 1, 3), linspace(0, 2, 3));
% centers = cartesian(linspace(-1.5, 1.5, 3), linspace(-2, 2, 3), linspace(0, 2, 3), linspace(-1.5, 1.5, 3), linspace(-1, 1, 2), linspace(0, 2, 2));
% LANDA = [x13(q), 1; 1, 0];

% m = 9;
% J = 5;
% R = 0.2;
% r = 0.05;
% % ng = 10;
% kt = 0.2639;
% kb = 0.019;
% ra= 1.6;

m = 4;
% J = 2.06;
J = 0.062;
R = 0.125;
r = 0.06;
ng = 60.5;
kt = 0.3;
kb = 0.020475;
ra= 23.5;

ku1 = (ng*kt)/ra;
ku2 = ng*kb*ku1;
             
dt = 0.01;
tspan = 0:dt:200;

%% i = 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic

xc1d = 0.30 * cos(tspan);
yc1d = 0.45 * sin(tspan);
theta1d = unwrap(atan2(0.45 * cos(tspan), -0.3 * sin(tspan)));

% v1d = sqrt((-0.8*sin(tspan)).^2 + (cos(tspan)).^2);
w1d = 6 *( ((sin(tspan).^2) + (cos(tspan).^2))./(4*(sin(tspan)).^2 + 9*(cos(tspan)).^2) );

x11d = theta1d;
x12d = xc1d .* cos(theta1d) + yc1d .* sin(theta1d);
x13d = xc1d .* sin(theta1d) - yc1d .* cos(theta1d);

%% i = 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xc2d = 0.30 * cos(tspan);
yc2d = 0.45 * sin(tspan);
theta2d = unwrap(atan2(0.45 * cos(tspan), -0.3 * sin(tspan)));

% v2d = sqrt((-sin(tspan)).^2 + (0.8*cos(tspan)).^2);
w2d = 6 *( ((sin(tspan).^2) + (cos(tspan).^2))./(4*(sin(tspan)).^2 + 9*(cos(tspan)).^2) );

x21d = theta2d;
x22d = xc2d .* cos(theta2d) + yc2d .* sin(theta2d);
x23d = xc2d .* sin(theta2d) - yc2d .* cos(theta2d);

%% i = 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xc3d = 0.30 * cos(tspan);
yc3d = 0.45 * sin(tspan);
theta3d = unwrap(atan2(0.45 * cos(tspan), -0.3 * sin(tspan)));

% v3d = sqrt((-1.2*sin(tspan)).^2 + (cos(tspan)).^2);
w3d = 6 *( ((sin(tspan).^2) + (cos(tspan).^2))./(4*(sin(tspan)).^2 + 9*(cos(tspan)).^2) );

x31d = theta3d;
x32d = xc3d .* cos(theta3d) + yc3d .* sin(theta3d);
x33d = xc3d .* sin(theta3d) - yc3d .* cos(theta3d);

figure(1)
plot(x12d, x13d, 'red', x22d, x23d , 'blue', x32d, x33d, 'black');
title('x12d vs x13d, x22d  vs  x23d, x32d vs x33d');

%% Preallocations
% i = 1

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
Xi11dot = zeros(1, length(tspan));
Xi12dot = zeros(1, length(tspan));
x13dot = zeros(1, length(tspan));
XiVirtual1 = zeros(2, length(tspan));
Xidotvirtual1 = zeros(2, length(tspan));
XiActual1 = zeros(2, length(tspan));
XidotActual1 = zeros(2, length(tspan));
f1 = zeros(2, length(tspan));
u1 = zeros(2, length(tspan));
z13 = zeros(2, length(tspan));
W1 = cell(1, length(tspan));
PHIvec1 = zeros(neurons, 1);
NNoutput1 = zeros(2, length(tspan));
beta1 = zeros(2, length(tspan));

% i = 2
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
Xi21dot = zeros(1, length(tspan));
Xi22dot = zeros(1, length(tspan));
x23dot = zeros(1, length(tspan));
XiVirtual2 = zeros(2, length(tspan));
Xidotvirtual2 = zeros(2, length(tspan));
XiActual2 = zeros(2, length(tspan));
XidotActual2 = zeros(2, length(tspan));
f2 = zeros(2, length(tspan));
u2 = zeros(2, length(tspan));
z23 = zeros(2, length(tspan));
W2 = cell(1, length(tspan));
PHIvec2 = zeros(neurons, 1);
NNoutput2 = zeros(2, length(tspan));
beta2 = zeros(2, length(tspan));

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
Xi31dot = zeros(1, length(tspan));
Xi32dot = zeros(1, length(tspan));
x33dot = zeros(1, length(tspan));
XiVirtual3 = zeros(2, length(tspan));
Xidotvirtual3 = zeros(2, length(tspan));
XiActual3 = zeros(2, length(tspan));
XidotActual3 = zeros(2, length(tspan));
f3 = zeros(2, length(tspan));
u3 = zeros(2, length(tspan));
z33 = zeros(2, length(tspan));
W3 = cell(1, length(tspan));
PHIvec3 = zeros(neurons, 1);
NNoutput3 = zeros(2, length(tspan));
beta3 = zeros(2, length(tspan));

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% i = 1
x11(1) = pi/2;
x12(1) = 0;
% x13(1) = 1;
x13(1) = 0.5;
x13dot(1) = 0;

yc1(1) = 0;
% xc1(1) = 1;
xc1(1) = 0.5;
Xi11(1) = 0;
Xi12(1) = 0;
XiActual1(1:2, 1) = [0; 0];
XidotActual1(1:2, 1) = [0; 0];
alpha1(1) = 0;
beta1(1:2, 1) = [1/r; R/r];
u1(1:2, 1) = [0; 0];
W1{1} = ones(neurons, 2);

% i = 2
x21(1) = pi/2;
x22(1) = 0;
% x23(1) = 1.2;
x23(1) = 0.45;
x23dot(1) = 0;

yc2(1) = 0;
% xc2(1) = 1.2;
xc2(1) = 0.45;
Xi21(1) = 0;
Xi22(1) = 0;
XiActual2(1:2, 1) = [0; 0];
XidotActual2(1:2, 1) = [0; 0];
alpha2(1) = 0;
beta2(1:2, 1) = [1/r; R/r];
u2(1:2, 1) = [0; 0];
W2{1} = ones(neurons, 2);

% i = 3
x31(1) = pi/2;
x32(1) = 0;
% x33(1) = 1.4;
x33(1) = 0.4;
x33dot(1) = 0;

yc3(1) = 0;
% xc3(1) = 1.4;
xc3(1) = 0.4;
Xi31(1) = 0;
Xi32(1) = 0;
XiActual3(1:2, 1) = [0; 0];
XidotActual3(1:2, 1) = [0; 0];
alpha3(1) = 0;
beta3(1:2, 1) = [1/r; R/r];
u3(1:2, 1) = [0; 0];
W3{1} = ones(neurons, 2);


tau = (1/ku1) * [0.1*sin(tspan); 0.1*cos(tspan)];

for q = 1 : length(tspan)
    q
    Xi11(q + 1) = w1d(q) + (k3 * (x11d(q) - x11(q)));
    Xi21(q + 1) = w2d(q) + (k3 * (x21d(q) - x21(q)));
    Xi31(q + 1) = w3d(q) + (k3 * (x31d(q) - x31(q)));
    
    z11(q) = x13d(q) - x13(q);
    alpha1(q + 1) = x12d(q) + ((le1^2 - z11(q)^2) * k1 * z11(q) * w1d(q));
    z21(q) = x23d(q) - x23(q);
    alpha2(q + 1) = x22d(q) + ((le1^2 - z21(q)^2) * k1 * z21(q) * w2d(q));
    z31(q) = x33d(q) - x33(q);
    alpha3(q + 1) = x32d(q) + ((le1^2 - z31(q)^2) * k1 * z31(q) * w3d(q));
    
    z12(q) = alpha1(q + 1) - x12(q);
    z22(q) = alpha2(q + 1) - x22(q);
    z32(q) = alpha3(q + 1) - x32(q);
    
    Xi12(q + 1) = ((alpha1(q + 1) - alpha1(q))/dt) + (le2^2 - z12(q)^2) * k2 * z12(q) * w1d(q)^2 + ((le2^2 - z12(q)^2)/(le1^2 - z11(q)^2)) * z11(q) * w1d(q);
    Xi22(q + 1) = ((alpha2(q + 1) - alpha2(q))/dt) + (le2^2 - z22(q)^2) * k2 * z22(q) * w2d(q)^2 + ((le2^2 - z22(q)^2)/(le1^2 - z21(q)^2)) * z21(q) * w2d(q);
    Xi32(q + 1) = ((alpha3(q + 1) - alpha3(q))/dt) + (le2^2 - z32(q)^2) * k2 * z32(q) * w3d(q)^2 + ((le2^2 - z32(q)^2)/(le1^2 - z31(q)^2)) * z31(q) * w3d(q);
    
    Xi11dot(q) = (Xi11(q + 1) - Xi11(q))/dt;
    Xi12dot(q) = (Xi12(q + 1) - Xi12(q))/dt; 
    Xi21dot(q) = (Xi21(q + 1) - Xi21(q))/dt;
    Xi22dot(q) = (Xi22(q + 1) - Xi22(q))/dt; 
    Xi31dot(q) = (Xi31(q + 1) - Xi31(q))/dt;
    Xi32dot(q) = (Xi32(q + 1) - Xi32(q))/dt; 
    

    XiVirtual1(:,q) = [Xi11(q); Xi12(q)];    
    Xidotvirtual1(:,q) = [Xi11dot(q); Xi12dot(q)];
    XiVirtual2(:,q) = [Xi21(q); Xi22(q)];    
    Xidotvirtual2(:,q) = [Xi21dot(q); Xi22dot(q)];
    XiVirtual3(:,q) = [Xi31(q); Xi32(q)];    
    Xidotvirtual3(:,q) = [Xi31dot(q); Xi32dot(q)];

    
    inputVec1 = [Xidotvirtual1(:,q);  XiVirtual1(:,q); x13dot(q); x13(q)];
    inputVec2 = [Xidotvirtual2(:,q);  XiVirtual2(:,q); x23dot(q); x23(q)];
    inputVec3 = [Xidotvirtual3(:,q);  XiVirtual3(:,q); x33dot(q); x33(q)];
       
    
    
    
    z13(:,q) = XiVirtual1(:,q) - XiActual1(:,q);
    z23(:,q) = XiVirtual2(:,q) - XiActual2(:,q);
    z33(:,q) = XiVirtual3(:,q) - XiActual3(:,q);
    
    U1 = [u1(1,q) + u1(2,q),     0;
         0                 ,     u1(1,q) - u1(2,q)];
    U2 = [u2(1,q) + u2(2,q),     0;
         0                 ,     u2(1,q) - u2(2,q)];
    U3 = [u3(1,q) + u3(2,q),     0;
         0                 ,     u3(1,q) - u3(2,q)];
    beta1(:,q + 1) = beta1(:,q) + dt * (GAMAofbeta *   U1 * z13(:,q));
    beta2(:,q + 1) = beta2(:,q) + dt * (GAMAofbeta *   U2 * z23(:,q));
    beta3(:,q + 1) = beta3(:,q) + dt * (GAMAofbeta *   U3 * z33(:,q));
    es_r1 = 1/beta1(1,q + 1);
    es_R1 = es_r1 * beta1(2,q + 1);
    es_r2 = 1/beta2(1,q + 1);
    es_R2 = es_r2 * beta2(2,q + 1);
    es_r3 = 1/beta3(1,q + 1);
    es_R3 = es_r3 * beta3(2,q + 1);
    
    B1hat1 = [(x13(q) + es_R1)/es_r1, (x13(q) - es_R1)/es_r1;
               1/es_r1              , 1/es_r1];
    B1hat2 = [(x23(q) + es_R2)/es_r2, (x23(q) - es_R2)/es_r2;
               1/es_r2               , 1/es_r2];
    B1hat3 = [(x33(q) + es_R3)/es_r3, (x33(q) - es_R3)/es_r3;
               1/es_r3              , 1/es_r3];
           
    for o = 1 : neurons
        
        PHIvec1(o, 1) = PHI(inputVec1, centers(o, :));
        PHIvec2(o, 1) = PHI(inputVec2, centers(o, :));
        PHIvec3(o, 1) = PHI(inputVec3, centers(o, :));
        
    end
    
    %%%%%%%
    
    W1{q + 1} = W1{q} + dt * (  (GAMAofW * PHIvec1 * z13(:,q).') - (RHO * ((W1{q} - W2{q})))  );
    W2{q + 1} = W2{q} + dt * (  (GAMAofW * PHIvec2 * z23(:,q).') - (RHO * ((W2{q} - W1{q}) + (W2{q} - W3{q})))  );
    W3{q + 1} = W3{q} + dt * (  (GAMAofW * PHIvec3 * z33(:,q).') - (RHO * ((W3{q} - W2{q})))  );
    
    %%%%%%%
    
    NNoutput1(:,q) = W1{q + 1}.' * PHIvec1;
    NNoutput2(:,q) = W2{q + 1}.' * PHIvec2;
    NNoutput3(:,q) = W3{q + 1}.' * PHIvec3;
    
    f1(:,q) = funF(Xidotvirtual1(:,q), XiVirtual1(:,q), XiActual1(:,q), x13dot(q), x13(q), v1(q), w1(q));
    f2(:,q) = funF(Xidotvirtual2(:,q), XiVirtual2(:,q), XiActual2(:,q), x23dot(q), x23(q), v2(q), w2(q));
    f3(:,q) = funF(Xidotvirtual3(:,q), XiVirtual3(:,q), XiActual3(:,q), x33dot(q), x33(q), v3(q), w3(q));
%     [(x13(q) + R)/r, (x13(q) - R)/r; 1/r, 1/r]
    u1(:,q + 1) = B1hat1 \ ( f1(:,q) + Kv1*z13(:,q) + ks1*sign(z13(:,q)) + ((pinv(z13(:,q).')) * ( (z13(1,q)*(Xidotvirtual1(1,q) - XidotActual1(1,q))/(le31^2 - z13(1,q)^2)) + (z13(2,q)*(Xidotvirtual1(2,q) - XidotActual1(2,q))/(le32^2 - z13(2,q)^2)) + ((k41*z13(1,q)^2)/(le31^2 - z13(1,q)^2)) + ((k42*z13(2,q)^2)/(le32^2 - z13(2,q)^2))  )));
    u2(:,q + 1) = B1hat2 \ ( f2(:,q) + Kv1*z23(:,q) + ks1*sign(z23(:,q)) + ((pinv(z23(:,q).')) * ( (z23(1,q)*(Xidotvirtual2(1,q) - XidotActual2(1,q))/(le31^2 - z23(1,q)^2)) + (z23(2,q)*(Xidotvirtual2(2,q) - XidotActual2(2,q))/(le32^2 - z23(2,q)^2)) + ((k41*z23(1,q)^2)/(le31^2 - z23(1,q)^2)) + ((k42*z23(2,q)^2)/(le32^2 - z23(2,q)^2))  )));
    u3(:,q + 1) = B1hat3 \ ( f3(:,q) + Kv1*z33(:,q) + ks1*sign(z33(:,q)) + ((pinv(z33(:,q).')) * ( (z33(1,q)*(Xidotvirtual3(1,q) - XidotActual3(1,q))/(le31^2 - z33(1,q)^2)) + (z33(2,q)*(Xidotvirtual3(2,q) - XidotActual3(2,q))/(le32^2 - z33(2,q)^2)) + ((k41*z33(1,q)^2)/(le31^2 - z33(1,q)^2)) + ((k42*z33(2,q)^2)/(le32^2 - z33(2,q)^2))  )));
    
    XiActual1(:,q + 1) =  XiActual1(:,q) + dt * ( ((1/ku1) * ([m * x13(q)^2 + J, m * x13(q); m * x13(q), m])) \ ([(x13(q) + R)/r, (x13(q) - R)/r; 1/r, 1/r] * u1(:,q + 1) - (1/ku1) * ([m * x13(q) * x13dot(q), 0; m * x13dot(q), 0]) * XiActual1(:,q) - ((2*ku2)/(ku1*r^2)) * ([x13(q)^2 + R^2, x13(q); x13(q), 1]) * XiActual1(:,q) - [x13(q), 1; 1, 0] * [30 * v1(q) + 4 * sign(v1(q)); 30 * w1(q) + 4 * sign(w1(q))] - (1/ku1) * [x13(q), 1; 1, 0] * [0.1*sin(tspan(q)); 0.1*cos(tspan(q))]));
    XiActual2(:,q + 1) =  XiActual2(:,q) + dt * ( ((1/ku1) * ([m * x23(q)^2 + J, m * x23(q); m * x23(q), m])) \ ([(x23(q) + R)/r, (x23(q) - R)/r; 1/r, 1/r] * u2(:,q + 1) - (1/ku1) * ([m * x23(q) * x23dot(q), 0; m * x23dot(q), 0]) * XiActual2(:,q) - ((2*ku2)/(ku1*r^2)) * ([x23(q)^2 + R^2, x23(q); x23(q), 1]) * XiActual2(:,q) - [x23(q), 1; 1, 0] * [30 * v2(q) + 4 * sign(v2(q)); 30 * w2(q) + 4 * sign(w2(q))] - (1/ku1) * [x23(q), 1; 1, 0] * [0.1*sin(tspan(q)); 0.1*cos(tspan(q))]));
    XiActual3(:,q + 1) =  XiActual3(:,q) + dt * ( ((1/ku1) * ([m * x33(q)^2 + J, m * x33(q); m * x33(q), m])) \ ([(x33(q) + R)/r, (x33(q) - R)/r; 1/r, 1/r] * u3(:,q + 1) - (1/ku1) * ([m * x33(q) * x33dot(q), 0; m * x33dot(q), 0]) * XiActual3(:,q) - ((2*ku2)/(ku1*r^2)) * ([x33(q)^2 + R^2, x33(q); x33(q), 1]) * XiActual3(:,q) - [x33(q), 1; 1, 0] * [30 * v3(q) + 4 * sign(v3(q)); 30 * w3(q) + 4 * sign(w3(q))] - (1/ku1) * [x33(q), 1; 1, 0] * [0.1*sin(tspan(q)); 0.1*cos(tspan(q))]));


    
%     v1(q) = (x13(q) * Xi11(q + 1)) + Xi12(q + 1);
%     w1(q) = Xi11(q + 1);
    
    
    v1(q) = (x13(q) * XiActual1(1,q)) + XiActual1(2,q);
    w1(q) = XiActual1(1,q);
    v2(q) = (x23(q) * XiActual2(1,q)) + XiActual2(2,q);
    w2(q) = XiActual2(1,q);
    v3(q) = (x33(q) * XiActual3(1,q)) + XiActual3(2,q);
    w3(q) = XiActual3(1,q);    
    
    xc1(q + 1) = xc1(q) + dt * v1(q) * cos(x11(q));
    yc1(q + 1) = yc1(q) + dt * v1(q) * sin(x11(q));
    x11(q + 1) = x11(q) + dt * w1(q);
    xc2(q + 1) = xc2(q) + dt * v2(q) * cos(x21(q));
    yc2(q + 1) = yc2(q) + dt * v2(q) * sin(x21(q));
    x21(q + 1) = x21(q) + dt * w2(q);
    xc3(q + 1) = xc3(q) + dt * v3(q) * cos(x31(q));
    yc3(q + 1) = yc3(q) + dt * v3(q) * sin(x31(q));
    x31(q + 1) = x31(q) + dt * w3(q);
    
    x12(q + 1) = xc1(q + 1) * cos(x11(q + 1)) + yc1(q + 1) * sin(x11(q + 1));
    x13(q + 1) = xc1(q + 1) * sin(x11(q + 1)) - yc1(q + 1) * cos(x11(q + 1));
    x22(q + 1) = xc2(q + 1) * cos(x21(q + 1)) + yc2(q + 1) * sin(x21(q + 1));
    x23(q + 1) = xc2(q + 1) * sin(x21(q + 1)) - yc2(q + 1) * cos(x21(q + 1));
    x32(q + 1) = xc3(q + 1) * cos(x31(q + 1)) + yc3(q + 1) * sin(x31(q + 1));
    x33(q + 1) = xc3(q + 1) * sin(x31(q + 1)) - yc3(q + 1) * cos(x31(q + 1));
    
    x13dot(q + 1) = (x13(q + 1) - x13(q))/dt;
    XidotActual1(:,q + 1) = (XiActual1(:,q + 1) - XiActual1(:,q))/dt;
    x23dot(q + 1) = (x23(q + 1) - x23(q))/dt;
    XidotActual2(:,q + 1) = (XiActual2(:,q + 1) - XiActual2(:,q))/dt;
    x33dot(q + 1) = (x33(q + 1) - x33(q))/dt;
    XidotActual3(:,q + 1) = (XiActual3(:,q + 1) - XiActual3(:,q))/dt;    
    
end




x12e = x12(1:length(x12)-1) - x12d;
x13e = x13(1:length(x13)-1) - x13d;
x22e = x22(1:length(x22)-1) - x22d;
x23e = x23(1:length(x23)-1) - x23d;
x32e = x32(1:length(x32)-1) - x32d;
x33e = x33(1:length(x33)-1) - x33d;

figure(2);
subplot(3, 1, 1);
plot(tspan, x11(1:length(x11)-1), 'red', tspan, x11d, 'blue');
grid on
title('x11 and x11d');
legend('x11', 'x11d');
subplot(3, 1, 2);
plot(tspan, x21(1:length(x21)-1), 'red', tspan, x21d, 'blue');
grid on
title('x21 and x21d');
legend('x21', 'x21d');
subplot(3, 1, 3);
plot(tspan, x31(1:length(x31)-1), 'red', tspan, x31d, 'blue');
grid on
title('x31 and x31d');
legend('x31', 'x31d');

figure(3);
subplot(1, 3, 1);
plot(x12, x13, 'red', x12d, x13d, 'blue');
grid on
title('x12 vs x13  and  x12d vs x13d');
subplot(1, 3, 2);
plot(x22, x23, 'red', x22d, x23d, 'blue');
grid on
title('x22 vs x23  and  x22d vs x23d');
subplot(1, 3, 3);
plot(x32, x33, 'red', x32d, x33d, 'blue');
grid on
title('x32 vs x33  and  x32d vs x33d');

figure(4);
plot(tspan, x12e, 'red', tspan, x22e, 'blue', tspan, x32e ,'black');
grid on
title('x2e');

figure(5);
plot(tspan, x13e, 'red', tspan, x23e, 'blue', tspan, x33e ,'black');
grid on
title('x3e');

figure(6);
subplot(1, 2, 1);
plot(tspan(ceil(9*length(tspan)/10):length(tspan)), f1(1,ceil(9*length(tspan)/10):length(tspan)), 'r', tspan(ceil(9*length(tspan)/10):length(tspan)), NNoutput1(1,ceil(9*length(tspan)/10):length(tspan)));
grid on
title('f1 and NNoutput');
legend('f1', 'NNoutput');
subplot(1, 2, 2);
plot(tspan(ceil(9*length(tspan)/10):length(tspan)), f1(2,ceil(9*length(tspan)/10):length(tspan)), 'r', tspan(ceil(9*length(tspan)/10):length(tspan)), NNoutput1(2,ceil(9*length(tspan)/10):length(tspan)));
grid on
title('f2 and NNoutput');
legend('f2', 'NNoutput');

figure(7);
subplot(1, 2, 1);
plot(tspan(ceil(9*length(tspan)/10):length(tspan)), f2(1,ceil(9*length(tspan)/10):length(tspan)), 'r', tspan(ceil(9*length(tspan)/10):length(tspan)), NNoutput2(1,ceil(9*length(tspan)/10):length(tspan)));
grid on
title('f1 and NNoutput');
legend('f1', 'NNoutput');
subplot(1, 2, 2);
plot(tspan(ceil(9*length(tspan)/10):length(tspan)), f2(2,ceil(9*length(tspan)/10):length(tspan)), 'r', tspan(ceil(9*length(tspan)/10):length(tspan)), NNoutput2(2,ceil(9*length(tspan)/10):length(tspan)));
grid on
title('f2 and NNoutput');
legend('f2', 'NNoutput');

figure(8);
subplot(1, 2, 1);
plot(tspan(ceil(9*length(tspan)/10):length(tspan)), f3(1,ceil(9*length(tspan)/10):length(tspan)), 'r', tspan(ceil(9*length(tspan)/10):length(tspan)), NNoutput3(1,ceil(9*length(tspan)/10):length(tspan)));
grid on
title('f1 and NNoutput');
legend('f1', 'NNoutput');
subplot(1, 2, 2);
plot(tspan(ceil(9*length(tspan)/10):length(tspan)), f3(2,ceil(9*length(tspan)/10):length(tspan)), 'r', tspan(ceil(9*length(tspan)/10):length(tspan)), NNoutput3(2,ceil(9*length(tspan)/10):length(tspan)));
grid on
title('f2 and NNoutput');
legend('f2', 'NNoutput');


sum(abs(x12e))
sum(abs(x22e))
sum(abs(x32e))

sum(abs(x13e))
sum(abs(x23e))
sum(abs(x33e))

max(abs(x12e(ceil(5*length(tspan)/10):length(tspan))))
max(abs(x22e(ceil(5*length(tspan)/10):length(tspan))))
max(abs(x32e(ceil(5*length(tspan)/10):length(tspan))))

max(abs(x13e(ceil(5*length(tspan)/10):length(tspan))))
max(abs(x23e(ceil(5*length(tspan)/10):length(tspan))))
max(abs(x33e(ceil(5*length(tspan)/10):length(tspan))))

norm(W1{length(tspan)+1}(:,1))
norm(W2{length(tspan)+1}(:,1))
norm(W3{length(tspan)+1}(:,1))
norm(W1{length(tspan)+1}(:,2))
norm(W2{length(tspan)+1}(:,2))
norm(W3{length(tspan)+1}(:,2))

WBaar1 = W1(ceil(9*length(tspan)/10):length(tspan));
WBaar2 = W2(ceil(9*length(tspan)/10):length(tspan));
WBaar3 = W3(ceil(9*length(tspan)/10):length(tspan));

WStar1 = W1{length(tspan)};
WStar2 = W2{length(tspan)};
WStar3 = W3{length(tspan)};

toc

function a = PHI(inputVec, C)

    a = exp(-((norm(inputVec - C.'))^2)/(1.4^2));

end

function C = cartesian(varargin)
    args = varargin;
    n = nargin;

    [F{1:n}] = ndgrid(args{:});

    for i=n:-1:1
        G(:,i) = F{i}(:);
    end

    C = unique(G , 'rows');
end

function a = funF(Xidotvirtual, Xivirtual, XiActual, x13dot, x13, v1, w1)

%     m = 9;
%     J = 5;
%     R = 0.2;
%     r = 0.05;
%     ng = 10;
%     kt = 0.2639;
%     kb = 0.019;
%     ra= 1.6;
%     ku1 = (ng*kt)/ra;
%     ku2 = ng*kb*ku1;
    m = 4;
%     J = 2.06;
    J = 0.062;
    R = 0.125;
    r = 0.06;
    ng = 60.5;
    kt = 0.3;
    kb = 0.020475;
    ra= 23.5;
    ku1 = (ng*kt)/ra;
    ku2 = ng*kb*ku1;


    
    M1 = (1/ku1) * ([m * x13^2 + J,     m * x13;
                     m * x13,           m]);
                 
    C1 = (1/ku1) * ([m * x13 * x13dot,     0;
                     m * x13dot,           0]);
                 
    X1 = ((2*ku2)/(ku1*r^2)) * ([x13^2 + R^2,     x13;
                                 x13,             1]);
                           
    F = [30 * v1 + 4 * sign(v1);
         30 * w1 + 4 * sign(w1)];
    F1 = ([x13,     1;
           1,       0]) * F;
                           
    a = M1 * Xidotvirtual + C1 * Xivirtual + X1 * XiActual + F1;

end


