yalmip('clear')
clear all;
close all;
clc

%% Parameters definition
% Model Parameters
params.Ts = 0.01;   % Sampling time (both for "MPC" and "Simulated Vehicle")
params.nstates = 4;
params.ninputs = 2;
params.l_f = 0.17;
params.l_r = 0.16;
params.l_q = params.l_r / (params.l_r + params.l_f);
params.tau = 1.35;

% Control Parameters
params.N = 20;   % The horizon
params.Q = [10 0 0 0; 0 10 0 0; 0 0 10 0; 0 0 0 10];
params.R = [0.01 0; 0 0.1];

%% Initial conditions for the state of the vehicle
params.x0 = 1.55;  
params.y0 = 0;  
params.v0 = 0;
params.psi0 = pi/2;

%% Number of iterations
params.N_max = 200;

%% The circular trajectory. 
% Every point on the circle is a reference
% to be followed by the vehicle. x_o and y_o are the coordinates of 
% the circle's center. r is its radius and v_ref is the velocity that
% the vehicle should have at each reference point
trajectory_params.x_o = 0;
trajectory_params.y_o = 0;
trajectory_params.r = 1.5;
trajectory_params.v_ref = 1;

%% Simulation environment
% Initialization

z(1,:)  = [params.x0, params.y0, params.v0 params.psi0];
u(1,:) = [0 0];

% Generate the trajectory to be followed
circ = trajectory_circular(trajectory_params);

% Obtain the symbolic form of the matrices A, B
syms xkn ykn vkn psikn xk yk vk psik Ts lr lf delta Vref tau
[symbolic_linear_A, symbolic_linear_B] = Symbolic_Kinematic_Model();

k = 0;
while (k < params.N_max)
  k = k+1;
  
  % The distances of all the points in the trajectory to the vehicle
  dists = sqrt((circ(:,1)-z(k,1)).^2 + (circ(:,2)-z(k,2)).^2);
  
  % The index of the point that is the closest to the vehicle
  [~, idx] = min(dists);
  
% mov_ref(k,:) = circ(mod(idx+10, 360) + 1, :);
  mov_ref(k,:) = circ(idx, :);
  

if (k == 1)
  params.linear_A = subs(symbolic_linear_A, [Ts lr lf tau vk psik delta], [params.Ts params.l_r params.l_f params.tau mov_ref(k,3) mov_ref(k,4) 0]);
  params.linear_B = subs(symbolic_linear_B, [Ts lr lf tau vk psik delta], [params.Ts params.l_r params.l_f params.tau mov_ref(k,3) mov_ref(k,4) 0]);
  
  params.linear_A = double(params.linear_A);
  params.linear_B = double(params.linear_B);
end

if (k > 1)
  % Completing a circle? Increase the orientation by 2*pi so that
  % it is always increasing along the trajectory.
  if (mov_ref(k,4) < mov_ref(k-1,4))
    mov_ref(k,4) = mov_ref(k,4) + 2*pi;
    circ(idx,4) = circ(idx,4) + 2*pi;
  end
  
  params.linear_A = subs(symbolic_linear_A, [Ts lr lf tau vk psik delta], [params.Ts params.l_r params.l_f params.tau mov_ref(k,3) mov_ref(k,4) u(k-1,2)]);
  params.linear_B = subs(symbolic_linear_B, [Ts lr lf tau vk psik delta], [params.Ts params.l_r params.l_f params.tau mov_ref(k,3) mov_ref(k,4) u(k-1,2)]);

  params.linear_A = double(params.linear_A);
  params.linear_B = double(params.linear_B);
end

%%   Ensure stability
%   [params.Qf,~,~] = dare(params.linear_A, params.linear_B, params.Q, params.R);

  
  yalmip('clear')

  % The predicted state and control variables
  z_mpc = sdpvar(params.N+1, params.nstates);
  u_mpc = sdpvar(params.N, params.ninputs);

  % Initial conditions. Reset constraints and cost
  constraints = [];
  J = 0;

  for i = 1:params.N
    J = J + ...
      (z_mpc(i,:)-mov_ref(k,:)) * params.Q * (z_mpc(i,:)-mov_ref(k,:))' +  ...
      u_mpc(i,:) * params.R * u_mpc(i,:)';

    % Model contraints
    constraints = [constraints, ...
      z_mpc(i+1,:)' == params.linear_A * z_mpc(i,:)' + params.linear_B * u_mpc(i,:)'];

    % Input constraints
    constraints = [constraints, ...
      0 <= u_mpc(i,1) <= 10*trajectory_params.v_ref, ...
      -pi/3 <= u_mpc(i,2) <= pi/3];
  end
  
  %  terminal cost
%   J = J + (z_mpc(params.N+1, :) - mov_ref(k,:)) * params.Qf * (z_mpc(params.N+1, :)-mov_ref(k,:))';

  assign(z_mpc(1,:), z(k,:));

  % Options
  ops = sdpsettings('solver', 'quadprog');

  % Optimize
  optimize([constraints, z_mpc(1,:) == z(k,:)], J, ops);

  %% snatch first predicted input 
  u(k,:) = value(u_mpc(1,:));

  % simulate the vehicle
  z(k+1,:) = car_sim_circular(z(k,:), u(k,:), params);
  
end


%% Plot state deviations and inputs

figure
plot(circ(:,1), circ(:,2))
hold on
plot(z(:,1), z(:,2))
title('trajectory')
axis equal
hold off

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
plot(mov_ref(:,3) - z(1:end-1,3))
title('velocity deviation')
ylim([params.v0, trajectory_params.v_ref])

subplot(2,2,4)
plot((mov_ref(:,4) - z(1:end-1,4))*180/pi)
title('orientation deviation')

figure
subplot(1,2,1)
plot(u(:,1))
title('input velocity')
subplot(1,2,2)
plot(u(:,2) * 180 / pi)
title('input steering angle')
