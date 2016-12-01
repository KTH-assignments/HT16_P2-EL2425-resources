%% EL2425 F1 Tenth Project
function [A, B] = Symbolic_Kinematic_Model(params)

  syms xkn ykn psikn xk yk psik Ts lr lf delta
  
  xkn = xk + Ts*params.v0*cos(psik + atan((lr/(lr+lf))*tan(delta)));
  ykn = yk + Ts*params.v0*sin(psik + atan((lr/(lr+lf))*tan(delta)));
  psikn = psik + Ts*(params.v0/lr)*sin(atan((lr/(lr+lf))*tan(delta)));

  A(1,1) = diff(xkn, xk);
  A(1,2) = diff(xkn, yk);
  A(1,3) = diff(xkn, psik);

  A(2,1) = diff(ykn, xk);
  A(2,2) = diff(ykn, yk);
  A(2,3) = diff(ykn, psik);

  A(3,1) = diff(psikn, xk);
  A(3,2) = diff(psikn, yk);
  A(3,3) = diff(psikn, psik);

  B(1,1) = diff(xkn, delta);

  B(2,1) = diff(ykn, delta);

  B(3,1) = diff(psikn, delta);

end