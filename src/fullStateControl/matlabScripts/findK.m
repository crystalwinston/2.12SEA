%% clear workspace
clear all; close all; clc;

%% define parameters
% moment of inertia of both masses
J1 = 0.000109127658; % kg*m^2
J2 = 0.0728591393; % kg*m^2

% spring constant
Ks = 350; %one band (Nm/rad)


%% define state space model
% state variables:
%           x1 = theta1 - 1/4*theta2
%           x2 = theta1_dot
%           x3 = theta2_dot
A = [0 0 1 0;
    0 0 0 1;
    -Ks/J1 Ks/(4*J1) 0 0;
    -Ks/J2 Ks/(4*J2) 0 0];
B = [0 0 1/J1 0]';
C = [1 0 0 0; 0 1 0 0];
D = 0;


% A = [0 1 -1/4
%     Ks/J1 0 0 
%     Ks/J2 0 0];
% B = [0 ; 1/J1 ; 0];
% C = [1  0  0];
% D = 0;
seaSS = ss(A,B,C,D);

%% determine if controllable and order
seaOrder = order(seaSS);

% check controllability
contrl = rank(ctrb(A,B)); % rank = order --> controllable

% check observability
observ = rank(obsv(seaSS)); % rank = order --> observable

%% look at rlocus for both outputs
figure(1); seaSS1 = ss(A,B,C(1,:),D); rlocus(seaSS1);
figure(2); seaSS2 = ss(A,B,C(2,:),D); rlocus(seaSS2);

% %% choose pole locations and calculate k
% p1 = -100+10i;
% p2 = -100-10i;
% p3 = -200;
% p4 = -300;
% K = place(A,B,[p1, p2, p3, p4]);

% %% simulate unforced cl system to check response to step
% t = 0:0.001:0.1; % sampling rate = 1000 Hz
% seaCLSS = ss(A-B*K,B,C,D);
% figure(1); step(seaCLSS,t)

%% design observer


%% use lqr to find pole locations
Q = C'*C; Q(1,1) = 500;
R = 1;
Klqr = lqr(A,B,Q,R);

Alqr = [(A-B*Klqr)];
Blqr = [B];
Clqr = [C(1,:)];
Dlqr = [D];

t = 0:.001:3;

seaCLLQRSS = ss(Alqr, Blqr, Clqr, Dlqr);
figure(2); step(seaCLLQRSS,t)

% %% find N
% Cn = C;
% seaNSS = ss(A,B,Cn,0);
% N = rscale(seaNSS,Klqr);



