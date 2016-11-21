function [A_valued B_valued] = get_matrices(params)

  %% EL2425 F1 Tenth Project
  clear all;
  close all;
  clc

  %% Parameters definition

  % Model Parameters
  Ts = params.Ts;
  nstates = params.nstates;
  ninputs = params.ninputs;
  l_f = params.l_f;
  l_r = params.l_r;

  % Control Parameters
  N = params.N;
  Q = params.Q;
  R = params.R;

  % Initial Conditions
  x0 = params.x0;
  y0 = params.y0;
  v0 = params.v0;
  psi0 = params.psi0;

  N_max = params.N_max;

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

  A_valued = subs(A, [Ts V lr lf], [Ts v0 l_r l_f]);
  B_valued = subs(B, [Ts V lr lf], [Ts v0 l_r l_f]);

  A_valued = A_valued(:,[1:2,4]);
  A_valued = A_valued([1:2,4],:);
  B_valued = B_valued(:,2);
  B_valued = B_valued([1:2,4]);

  % params.linear_A = subs(A_valued, [psik delta], [mov_ref(k,3) u(k-1)]);
  % params.linear_B = subs(B_valued, [psik delta], [mov_ref(k,3) u(k-1)]);
end
