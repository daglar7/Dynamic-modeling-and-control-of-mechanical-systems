mk = 0.42;
m = 4.42;
l = 0.42;
I = 0.08433;
Fr = 6.5;
C = 0.06;
g=9.81;

%dynamic modelling
syms s Rs th_s F_s g 
th_s = ((mk*l*s^2)/(mk*l*g+I*s^2+C*s))*Rs;
F_s = (m*s^2+Fr*s)*Rs - mk*l*s^2*th_s;
Ts = th_s/F_s;
x=(collect(Ts,s));
pretty(x)

syms s Rs th_s F_s g Kp Ki Kd
th_s = ((mk*l*s^2)/(mk*l*g+I*s^2+C*s))*Rs;
F_s = (m*s^2+Fr*s)*Rs - s^2*mk*l*th_s;
g2 = th_s/F_s;
g1 = Kp+(Ki/s)+Kd*s;
Gs = (g1*g2)/(1+g1*g2);
y=(collect(Gs,s));
pretty((vpa(y, 5)))

%state space numerical values
q= -l^2*mk^2+I*m;
x1=(-C*m-Fr*I)/q;
x2=(-m*g*l*mk-C*Fr)/q;
x3= (-mk*g*l)/q;
F3=l*mk/q;

%%
%Case1: Underdamped PerformanceUnderdamped Performance
num=[1 0];
den=[1.93 4.61 41.15 63.77];
G1=tf(num,den);
kd=115.36; kp=2109.14 ; ki=27838.13  ; 
G2=pid(kp,ki,kd);
gs = feedback(G1*G2,1);
step(gs);
%Case2: Critically Damped Performance
num=[1 0];
den=[1.93 4.61 41.15 63.77];
G1=tf(num,den);
kd=85.448; kp=902.22 ; ki=2794.59  ; 
G2=pid(kp,ki,kd);
gs = feedback(G1*G2,1);
step(gs)
%%
%State-Variable Feedback Design Method 
%Case 1: Underdamped performance
A=[0 1 0;0 0 1;-2.3808 -23.5311 -5.0655];	
B=[0;0; 0.5164], C=[1 0 0], D=0;    
Cm = ctrb(A,B);	% Calculate the controllability matrix.
Rank = rank(Cm)	% Check if the system is controllable.
z =0.492; 		% Damping ratio as obtained in chapter 6.
wn =18.05;		% Natural frequency as obtained in chapter 6.
[num, den] = ord2(wn, z); % Produce a 2nd-order system that meets the response requirements.
r = roots(den);		% Assign the dominant poles to a vector ”r”.
poles = [r(1) r(2) -44.4];	% Place dominant & remaining poles.
K = acker(A, B, poles);	% Compute the state-variable controller gains.
Tss = ss(A-B*K, B, C, D); % The new system.
Kr = 1/dcgain(Tss);	% The scaling factor to eliminate steady state error.
Tss_scaled = ss(A-B*K, Kr.*B, C, D); % Scaled system to eliminate steady-state error.
poles = eig(A-B*K);	% Display the poles of the new system.
step(Tss_scaled)
%Case 2: Critically damped performance
A=[0 1 0;0 0 1;-2.3808 -23.5311 -5.0655];	
B=[0;0; 0.5164], C=[1 0 0], D=0;
poles = [-6.666 -6.666 -33.33];	% Place the desired poles as obtained in chapter 7.
K = acker(A, B, poles);	% Compute the state-variable controller gains.
Tss = ss(A-B*K, B, C, D); % The new system.
Kr = 1/dcgain(Tss);	% The scaling factor to eliminate steady state error.
Tss_scaled = ss(A-B*K, Kr.*B, C, D); % Scaled system to eliminate steady-state error.
poles = eig(A-B*K);	% Display the poles of the new system.
step(Tss_scaled)	% Calculate and display the step response of the system.
%%
%Full-Order State Observer Design
A=[0 1 0;0 0 1;-2.3808 -23.5311 -5.0655];	
B=[0;0; 0.5164], C=[1 0 0], D=0;  
Om = obsv(A,C)	% Calculate the observability matrix.
Rank = rank(Om)	% Check if the system is observable.
z =0.492; 		% Damping ratio as obtained in chapter 6.
wn =18.05;		% Natural frequency as obtained in chapter 6.
[num, den] = ord2(wn, z); % Produce a 2nd-order system that meets the response requirements.
r = roots(den);		% Assign dominant poles to a vector ”r”.
poles = [r' 10*real(r(1))];  % Place the desired poles.
ob_poles = 10*poles;	% Increase 10 times the observer poles.
L = place(A',C', ob_poles).'	% Calculate observer gains for the original SS.
K = acker(A, B, poles);	% Compute & display controller gains.
Tss = ss(A-B*K, B, C, D);	% Form the new SS system LTI object.
step(Tss)		% Produce compensated system step response
%%
%LQR
A=[0 1 0;0 0 1;-2.3808 -23.5311 -5.0655];	
B=[0;0; 0.5164], C=[1 0 0], D=0; 
Q = [100 0 0; 0 1 0; 0 0 1];  
R = [0.01]; 
K = lqr(A, B, Q, R);
sys=ss(A-B*K, B*K(1), C, D);
step(sys)


