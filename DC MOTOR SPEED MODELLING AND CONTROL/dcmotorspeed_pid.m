J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;
s =tf('s');
p_m = K/((J*L*s^2)+(K^2 +L*b+J*R)*s + R*b);

%propotional
Kp = 100;
C = pid(Kp);
sys_cl = feedback(C*p_m,1);
t = 0:0.01:4;
step(sys_cl,t)
%%% linearSystemAnalyzer('step',sys_cl,0:0.01:2)
figure
grid
title('Step Response with Proportional Control')

%pid
kp=100; ki=200; kd=10;
C = pid(kp,ki,kd);
sys_closed = feedback(C*p_m,1);
step(sys_closed,[0:0.01:4])
title('PID Control with Kp Ki Kd')