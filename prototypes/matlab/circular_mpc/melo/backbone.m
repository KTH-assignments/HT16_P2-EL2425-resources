%% EL2425 F1 Tenth Project
clear all;
close all;
clc

%% Parameters definition

% Model Parameters
params.Ts = 0.01; % Sampling Time (both of MPC and Simulated Vehicle)
params.nstates = 3; % number of states
params.ninputs = 1; % number of inputs
params.l_f = 0.17; % meter
params.l_r = 0.16; % meter
% params.l_q = params.l_r / (params.l_r + params.l_f);

% Control Parameters
params.N = 20; % The horizon
params.Q = [1 0 0; 0 1 0; 0 0 1];
params.R = 0.01;

% Initial Conditions
params.x0 = 0; % Initial "x" Coordinate
params.y0 = -1.55; % Initial "y" Coordinate
params.v0 = 1; % Initial Speed
params.psi0 = 0; % Initial Heading Angle

params.N_max = 100;

syms xkn ykn vkn psikn xk yk vk psik Ts a V lr lf  delta
states = {'xpos', 'ypos', 'vel', 'psi'};

xkn = xk + Ts*V*cos(psik + atan((lr/(lr+lf))*tan(delta)));
ykn = yk + Ts*V*sin(psik + atan((lr/(lr+lf))*tan(delta)));
vkn = vk + Ts*a;
psikn = psik + Ts*(V/lr)*sin(atan((lr/(lr+lf))*tan(delta)));

A(1,1) = diff(xkn, xk);
A(1,2) = diff(xkn, yk);
A(1,3) = diff(xkn, vk);
A(1,4) = diff(xkn, psik);

A(2,1) = diff(ykn, xk);
A(2,2) = diff(ykn, yk);
A(2,3) = diff(ykn, vk);
A(2,4) = diff(ykn, psik);

A(3,1) = diff(vkn, xk);
A(3,2) = diff(vkn, yk);
A(3,3) = diff(vkn, vk);
A(3,4) = diff(vkn, psik);

A(4,1) = diff(psikn, xk);
A(4,2) = diff(psikn, yk);
A(4,3) = diff(psikn, vk);
A(4,4) = diff(psikn, psik);

B(1,1) = diff(xkn, a);
B(1,2) = diff(xkn, delta);

B(2,1) = diff(ykn, a);
B(2,2) = diff(ykn, delta);

B(3,1) = diff(vkn, a);
B(3,2) = diff(vkn, delta);

B(4,1) = diff(psikn, a);
B(4,2) = diff(psikn, delta);

A_valued = subs(A, [Ts V lr lf], [params.Ts params.v0 params.l_r params.l_f]);
B_valued = subs(B, [Ts V lr lf], [params.Ts params.v0 params.l_r params.l_f]);

A_valued = A_valued(:,[1:2,4]);
A_valued = A_valued([1:2,4],:);
B_valued = B_valued(:,2);
B_valued = B_valued([1:2,4]);

% params.linear_A = subs(A_valued, [psik delta], [mov_ref(k,3) u(k-1)]);
% params.linear_B = subs(B_valued, [psik delta], [mov_ref(k,3) u(k-1)]);
