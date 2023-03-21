m1 = 2500;
m2 = 320;
k1 = 80000;
k2 = 500000;
b1 = 350;
b2 = 15020;

%transfer function
syms s x1s x2s
x1s = x2s*((b1*s+k2)/(m1*s^2 + b1*s + k1));
ws = ((m2*s^2+(b1+b2)*s+(k1+k2))*x2s-(b1*s+k1)*x1s)/(b2*s+k2);
x2_tf = x2s/ws;
pretty(collect(x2_tf,s));

syms s x1s x2s 
x2s = x1s/((b1*s+k2)/(m1*s^2 + b1*s + k1));
ws= ((m2*s^2+(b1+b2)*s+(k1+k2))*x2s-(b1*s+k1)*x1s)/(b2*s+k2);
x1_tf = x1s/ws;
pretty(collect(x1_tf,s));

% Plot step response
[num, den] = numden(x1_tf);
x1_tf = tf(sym2poly(num), sym2poly(den));
[num, den] = numden(x2_tf);
x2_tf = tf(sym2poly(num), sym2poly(den));

figure(1);
step(x1_tf);
title('Figure x1_tf');
figure(2);
step(x2_tf);
title('Figure x2_tf');