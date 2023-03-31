J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;
%transfer function
syms s Is theta_s 
Is = ((s*J+b)/K) * theta_s;
Vs = (R+L*s)*Is + K*s*theta_s;
mspeed_tf = theta_s/Vs;
pretty(collect(mspeed_tf,s))

%state space
A = [-b/J   K/J
    -K/L   -R/L];
B = [0
    1/L];
C = [1   0];
D = 0;
motor_ss = ss(A,B,C,D)

%analysis
s =tf('s');
p_m = K/((J*L*s^2)+(K^2 +L*b+J*R)*s + R*b);
%%% t=0:0.1:5;
%%% step(p_m,t)
rP_motor = 0.1/(0.5*s+1);
linearSystemAnalyzer('step', p_m,rP_motor, 0:0.01:5);