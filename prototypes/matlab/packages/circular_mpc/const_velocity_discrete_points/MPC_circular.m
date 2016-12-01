yalmip('clear')
clear all;
close all;
clc

%% Parameters definition
% Model Parameters
params.Ts = 0.01;   % Sampling time (both for "MPC" and "Simulated Vehicle")
params.nstates = 3;
params.ninputs = 1;
params.l_f = 0.17;
params.l_r = 0.16;
params.l_q = params.l_r / (params.l_r + params.l_f);

% Control Parameters
params.N = 5;   % The horizon
params.Q = [0.50 0 0; 0 0.50 0; 0 0 0.01];
params.R = 0.001;

%% Initial conditions for the state of the vehicle
       
params.x0 = -0.2635;  
params.y0 = -1.5844;  
params.v0 = 1;
params.psi0 = 6.0107;

%% Number of iterations
params.N_max = 3000;

%% The circular trajectory. 
% Every point on the circle is a reference
% to be followed by the vehicle. x_o and y_o are the coordinates of 
% the circle's center. r is its radius.
trajectory_params.x_o = 0;
trajectory_params.y_o = 0;
trajectory_params.r = 1.5;

%% Simulation environment
% Initialization

z(1,:)  = [params.x0, params.y0, params.psi0];
u(1) = 0;

% Generate the trajectory to be followed
circ = trajectory_circular(trajectory_params);

% Obtain the symbolic form of the matrices A, B
syms xkn ykn psikn xk yk psik Ts lr lf delta
[symbolic_linear_A, symbolic_linear_B] = Symbolic_Kinematic_Model(params);

k = 0;
while (k < params.N_max)
  k = k+1
  
  % The distances of all the points in the trajectory to the vehicle
  dists = sqrt((circ(:,1)-z(k,1)).^2 + (circ(:,2)-z(k,2)).^2);
  
  % The index of the point that is the closest to the vehicle
  [~, idx] = min(dists);
  
  % The first reference. Now we need to find the rest
  mov_ref(k,:) = circ(idx, :);
  % mov_ref(k,:) = circ(mod(idx+10, 360) + 1, :);
    
  
  refs = zeros(params.N+1, 3);
  
  for i = 1:params.N+1
    t = mov_ref(k,3) + params.v0 / trajectory_params.r * params.Ts * i;
    x = trajectory_params.x_o + trajectory_params.r * cos(t - pi/2);
    y = trajectory_params.y_o + trajectory_params.r * sin(t - pi/2);
    
    refs(i,:) = [x, y, t];
  end
 
%   mov_ref(k,:)
%   refs
%   

if (k == 1)
  params.linear_A = subs(symbolic_linear_A, [Ts lr lf psik delta], [params.Ts params.l_r params.l_f z(k,3) 0]);
  params.linear_B = subs(symbolic_linear_B, [Ts lr lf psik delta], [params.Ts params.l_r params.l_f z(k,3) 0]);
  
  params.linear_A = double(params.linear_A);
  params.linear_B = double(params.linear_B);
end

if (k > 1)
  % Completing a circle? Increase the orientation by 2*pi so that
  % it is always increasing along the trajectory.
  if (mov_ref(k,3) < mov_ref(k-1,3))
    
    refs(:,3) = refs(:,3) + 2*pi;
    mov_ref(k,3) = mov_ref(k,3) + 2*pi;
    circ(idx,3) = circ(idx,3) + 2*pi;
  end
  
  params.linear_A = subs(symbolic_linear_A, [Ts lr lf psik delta], [params.Ts params.l_r params.l_f z(k,3) u(k-1)]);
  params.linear_B = subs(symbolic_linear_B, [Ts lr lf psik delta], [params.Ts params.l_r params.l_f z(k,3) u(k-1)]);

  params.linear_A = double(params.linear_A);
  params.linear_B = double(params.linear_B);
end

%%   Ensure stability
%   [params.Qf,~,~,err] = dare(params.linear_A, params.linear_B, params.Q, params.R);
%   if (err == -1 || err == -2)
%     params.Qf = params.Q;
%   end
%     
  yalmip('clear')

  % The predicted state and control variables
  z_mpc = sdpvar(params.N+1, params.nstates);
  u_mpc = sdpvar(params.N, params.ninputs);

  % Initial conditions. Reset constraints and cost
  constraints = [];
  J = 0;

  for i = 1:params.N
    
    J = J + ...
      (z_mpc(i,:)-refs(i,:)) * params.Q * (z_mpc(i,:)-refs(i,:))' +  ...
      u_mpc(i,:) * params.R * u_mpc(i,:)';


    % Model contraints
    constraints = [constraints, ...
      z_mpc(i+1,:)' == params.linear_A * z_mpc(i,:)' + params.linear_B * u_mpc(i,:)'];

    % Input constraints
    constraints = [constraints, ...
      -pi/3 <= u_mpc(i) <= pi/3];
  end
  
  %  terminal cost
%   J = J + (z_mpc(params.N+1, :) - refs(params.N+1,:)) * params.Qf * (z_mpc(params.N+1, :)-refs(params.N+1,:))';

  % terminal constraints?
%     constraints = [constraints, ...
%        z_mpc(params.N+1, 3) - refs(params.N+1, 3) == 0];
        
  assign(z_mpc(1,:), z(k,:));

  % Options
  ops = sdpsettings('solver', 'quadprog');

  % Optimize
  optimize([constraints, z_mpc(1,:) == z(k,:)], J, ops);

  %% snatch first predicted input 
  u(k) = value(u_mpc(1));

  % simulate the vehicle
  z(k+1,:) = car_sim_circular(z(k,:), u(k), params);
%   z(k+1,3) = z(k+1,3) + 2*pi*fix(round(180/(2*pi)*mov_ref(k,3))/180) - 2*pi*fix(round(180/(2*pi)*z(k,3))/180);
  
 180/pi * (z(k+1,3)-refs(1,3))
  
  
%   figure
  plot(circ(:,1), circ(:,2))
  hold on
  plot(z(2:end,1), z(2:end,2));
  plot(refs(:,1), refs(:,2), '*')
  plot(z(k,1), z(k,2), '*', 'Color', 'r')
  ref_l = refline(tan(mov_ref(k,3)), -tan(mov_ref(k,3))*mov_ref(k,1)+mov_ref(k,2));
  z_l = refline(tan(z(k,3)), -tan(z(k,3))*z(k,1)+z(k,2));
  set(z_l,'Color','r')
  axis equal
  axis([-2 2 -2 2])
  drawnow
  title('trajectory')
  hold off


end


%% Plot state deviations and inputs

figure
subplot(2,2,1)
plot(circ(:,1), circ(:,2))
hold on
plot(mov_ref(:,1), mov_ref(:,2))
title('given reference')
axis equal
hold off

subplot(2,2,2)
plot((z(1:end-1, 1) - mov_ref(:,1)).^2 + (z(1:end-1, 2) - mov_ref(:,2)).^2)
title('distance deviation')


subplot(2,2,3)
plot((mov_ref(:,3) - z(1:end-1,3))*180/pi)
title('orientation deviation')

figure
plot(u * 180 / pi)
title('input steering angle')
