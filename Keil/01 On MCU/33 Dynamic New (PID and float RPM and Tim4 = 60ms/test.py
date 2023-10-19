z13 = XiVirtual1 - XiActual1;

B1hat1 = [(x13 + es_R1)/es_r1, (x13 - es_R1)/es_r1;
               1/es_r1              , 1/es_r1];


W1 = W1 + dt * (  (GAMAofW * PHIvec1 * z13.')); %% - (RHO * ((W1{q} - W2{q})))  );


NNoutput1 = W1.' * PHIvec1;

f1 = funF(Xidotvirtual1, XiVirtual1, XiActual1, x13dot, x13, v1, w1);

u1 = B1hat1 \ ( NNoutput1 + Kv1*z13 + ks1*sign(z13) + ((pinv(z13.')) * ( (z13(1,q)*(Xidotvirtual1(1,q) - XidotActual1(1,q))/(le31^2 - z13(1,q)^2)) + (z13(2,q)*(Xidotvirtual1(2,q) - XidotActual1(2,q))/(le32^2 - z13(2,q)^2)) + ((k41*z13(1,q)^2)/(le31^2 - z13(1,q)^2)) + ((k42*z13(2,q)^2)/(le32^2 - z13(2,q)^2))  )));



XiActual1 =  XiActual1 + dt * ( ((1/ku1) * ([m * x13^2 + J, m * x13; m * x13, m])) \ ([(x13 + R)/r, (x13 - R)/r; 1/r, 1/r] * u1 - (1/ku1) * ([m * x13 * x13dot, 0; m * x13dot, 0]) * XiActual1 - ((2*ku2)/(ku1*r^2)) * ([x13^2 + R^2, x13; x13, 1]) * XiActual1 - [x13, 1; 1, 0] * [30 * v1 + 4 * sign(v1); 30 * w1 + 4 * sign(w1)] - (1/ku1) * [x13, 1; 1, 0] * [0.1*sin(tspan); 0.1*cos(tspan)]));




function a = funF(Xidotvirtual, Xivirtual, XiActual, x13dot, x13, v1, w1)

    m = 9;
    J = 5;
    R = 0.2;
    r = 0.05;
    ng = 10;
    kt = 0.2639;
    kb = 0.019;
    ra= 1.6;
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