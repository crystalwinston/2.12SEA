%% clear workspace
clear all; close all; clc;

%% define parameters
% moment of inertia of both masses
J1 = 0.00040984488; % kg*m^2 0.1025; 
J2 = 0.832987; % kg*m^2
b = 0; % is there any damping in the motor? 

% spring constant
Ks = 350; %one band (Nm/rad)

%% define state space model
% state variables:
%           x1 = theta1
%           x2 = theta2
%           x3 = theta1_dot
%           x4 = theta2_dot

% A = [0 0 1 0;
%     0 0 0 1;
%     -(Ks-b)/J1 Ks/(J1) 0 0;
%     Ks/J2 -Ks/(J2) 0 0];

% assuming j2 to be correct, then k = 106.5474 and j1 = 0.0628
wp = 6.8*2*pi;
wz = 1.8*2*pi;
A = [0 0 1 0; 
    0 0 0 1;
    -(wp^2-wz^2) (wp^2-wz^2) 0 0;
    (wz)^2 -1*(wz)^2 0 0];

B = [0 0 26.9*4/.0905 0]';
C = [1 0 0 0];
D = 0;

seaSS = ss(A,B,C,D);
%% determine if controllable and order and rlocus
% check controllability
contrl = rank(ctrb(A,B)); % rank = order --> controllable

% check observability
observ = rank(obsv(seaSS)); % rank = order --> observable

% look at rlocus
figure(1); rlocus(seaSS);
figure(4); bode(seaSS);
%% use lqr to find pole locations and simulate
Q = C'*C; Q(1,1) = 1000;
R = 1;
Klqr = lqr(A,B,Q,R);

Alqr = [(A-B*Klqr)];
Blqr = [B];
Clqr = [C];
Dlqr = [D];

t = 0:.001:3;
seaCLLQRSS = ss(Alqr, Blqr, Clqr, Dlqr);
figure(2); step(seaCLLQRSS,t);
figure(3); rlocus(seaCLLQRSS);

%% now place pole at zero for integrator and recalc k
p5 = 0;
lqrPoles = pole(seaCLLQRSS);
allPoles = [lqrPoles' p5];

Aint = [0 0 1 0 0;
    0 0 0 1 0;
    (Ks-b)/J1 -Ks/(4*J1) 0 0 0;
    Ks/J2 -Ks/(4*J2) 0 0 0;
    0 1 0 0 0];
Bint = [0 0 1/J1 0 0]'; 

kInt = place(Aint,Bint, allPoles');

