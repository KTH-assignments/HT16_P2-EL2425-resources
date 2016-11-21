%% EL2425 F1 Tenth Project
function [A, B] = Symbolic_Kinematic_Model()

syms xkn ykn vkn psikn xk yk vk psik Ts a V lr lf  delta Vref tau

xkn = xk + Ts*V*cos(psik + atan((lr/(lr+lf))*tan(delta)));
ykn = yk + Ts*V*sin(psik + atan((lr/(lr+lf))*tan(delta)));
vkn = (Vref - vk) / tau;
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

B(1,1) = diff(xkn, Vref);
B(1,2) = diff(xkn, delta);

B(2,1) = diff(ykn, Vref);
B(2,2) = diff(ykn, delta);

B(3,1) = diff(vkn, Vref);
B(3,2) = diff(vkn, delta);

B(4,1) = diff(psikn, Vref);
B(4,2) = diff(psikn, delta);

% A = subs(A, [Ts V lr lf tau], [params.Ts params.v0 params.l_r params.l_f params.tau]);
% B = subs(B, [Ts V lr lf tau], [params.Ts params.v0 params.l_r params.l_f params.tau]);

end

%% Elimimating "vk" from the states and "Vref" from the inputs
% A_valued = A_valued(:,[1:2,4]);
% A_valued = A_valued([1:2,4],:);
% B_valued = B_valued(:,2);
% B_valued = B_valued([1:2,4]);

%% "A" and "B" matrices without the state "vk" and input "Vref" A = (3 by 3), B = (3 by 1)
% if (k == 1)
% params.linear_A = [1 0 -sin(mov_ref(k,3) + atan((16*tan(0))/33))/100;0 1 cos(mov_ref(k,3) + atan((16*tan(0))/33))/100;0 0 1];
% params.linear_B = [-(4*sin(mov_ref(k,3) + atan((16*tan(0))/33))*(tan(0)^2 + 1))/(825*((256*tan(0)^2)/1089 + 1));(4*cos(mov_ref(k,3) + atan((16*tan(0))/33))*(tan(0)^2 + 1))/(825*((256*tan(0)^2)/1089 + 1));(tan(0)^2 + 1)/(33*((256*tan(0)^2)/1089 + 1)^(1/2)) - (256*tan(0)^2*(tan(0)^2 + 1))/(35937*((256*tan(0)^2)/1089 + 1)^(3/2))];
% end
% 
% if (k > 1)
% params.linear_A = [1 0 -sin(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))/100;0 1 cos(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))/100;0 0 1];
% params.linear_B = [-(4*sin(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))*(tan(u(k-1,1))^2 + 1))/(825*((256*tan(u(k-1,1))^2)/1089 + 1));(4*cos(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))*(tan(u(k-1,1))^2 + 1))/(825*((256*tan(u(k-1,1))^2)/1089 + 1));(tan(u(k-1,1))^2 + 1)/(33*((256*tan(u(k-1,1))^2)/1089 + 1)^(1/2)) - (256*tan(u(k-1,1))^2*(tan(u(k-1,1))^2 + 1))/(35937*((256*tan(u(k-1,1))^2)/1089 + 1)^(3/2))];
% end

%% "A" and "B" matrices A = (4 by 4), B = (4 by 2)
% if (k == 1)
% params.linear_A = [1 0 0 -sin(mov_ref(k,3) + atan((16*tan(0))/33))/100;0 1 0 cos(mov_ref(k,3) + atan((16*tan(0))/33))/100;0 0 (-10/9) 0;0 0 0 1];
% params.linear_B = [0 -(4*sin(mov_ref(k,3) + atan((16*tan(0))/33))*(tan(0)^2 + 1))/(825*((256*tan(0)^2)/1089 + 1));0 (4*cos(mov_ref(k,3) + atan((16*tan(0))/33))*(tan(0)^2 + 1))/(825*((256*tan(0)^2)/1089 + 1));(10/9) 0;0 (tan(0)^2 + 1)/(33*((256*tan(0)^2)/1089 + 1)^(1/2)) - (256*tan(0)^2*(tan(0)^2 + 1))/(35937*((256*tan(0)^2)/1089 + 1)^(3/2))];
% end
% 
% if (k > 1)
% params.linear_A = [1 0 0 -sin(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))/100;0 1 0 cos(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))/100;0 0 (-10/9) 0;0 0 0 1];
% params.linear_B = [0 -(4*sin(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))*(tan(u(k-1,1))^2 + 1))/(825*((256*tan(u(k-1,1))^2)/1089 + 1));0 (4*cos(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))*(tan(u(k-1,1))^2 + 1))/(825*((256*tan(u(k-1,1))^2)/1089 + 1));(10/9) 0;0 (tan(u(k-1,1))^2 + 1)/(33*((256*tan(u(k-1,1))^2)/1089 + 1)^(1/2)) - (256*tan(u(k-1,1))^2*(tan(u(k-1,1))^2 + 1))/(35937*((256*tan(u(k-1,1))^2)/1089 + 1)^(3/2))];
% end