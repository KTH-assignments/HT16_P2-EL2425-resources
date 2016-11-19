% Created by Pedro Lima (pfrdal@kth.se) and Valerio Turri (turri@kth.se)
% EL2700 MPC Course Prof. Mikael Johansson
% Nonlinear MPC for vehicle control and obstacle avoidance
%%
yalmip('clear')

clear all; 
close all;
clc

%% Parameters definition
% model parameters

params.Ts                   = 0.1;                      % sampling time (both of MPC and simulated vehicle)
params.nstates              = 3;                        % number of states
params.ninputs              = 1;                        % number of inputs

% control parameters
params.N                    = 14;                        % The horizon
params.Q                    = eye(3)*0.001; %[0.001 0 0 0; 0 0.1 0 0; 0 0 0.001 0; 0 0 0 0.1;];
% params.Q                    = [0.001 0 0 0; 0 0.1 0 0; 0 0 0.001 0; 0 0 0 0.1;];

% params.Qf                   = params.Q;
% params.Qf                   = [0.001 0 0 0; 0 1 0 0; 0 0 0.001 0; 0 0 0 1;];
params.R                    = 0.1; %[2 0; 0 10];
% params.R                    = [2 0; 0 10];

% initial conditions
params.x0                   = 2.5;                        % initial x coordinate
params.y0                   = -0.25;                        % initial y coordinate
params.v0                   = 2;                       % initial speed
params.psi0                 = pi/3;                        % initial heading angle

params.N_max                = 1000;                        % maximum number of simulation steps




%% Simulation environment
% initialization

z       = zeros(params.N_max+1, params.nstates);             % z(k,j) denotes state j at step k-1
e       = zeros(params.N_max+1, params.nstates);             % z(k,j) denotes state j at step k-1
u       = zeros(params.N_max, params.ninputs);             % z(k,j) denotes input j at step k-1
z(1,:)  = [params.x0, params.y0, params.psi0]; % definition of the intial state


k = 0;

while (k < params.N_max)
  k = k+1;

  yalmip('clear')
  
  
  % The linearized model's matrices
  params.linear_A = [
    1 0 -params.Ts * params.v0 *sin(z(k,3)); 
    0 1 params.Ts * params.v0 * cos(z(k,3)); 
    0 0 1];
  
  params.linear_B = [
    -params.Ts * 0.17 * params.v0 / 0.33 * sin(z(k,3)); 
    params.Ts * params.v0 * cos(z(k,3)); 
    params.Ts*params.v0/0.33];
  
  % Ensure stability
  [params.Qf,~,~] = dare(params.linear_A, params.linear_B, params.Q, params.R);

  
  
  % The predicted state and control variables
  z_mpc = sdpvar(params.N+1, params.nstates);
  u_mpc = sdpvar(params.N, params.ninputs);

  % Initial conditions
  constraints = [];
  J = 0;



  for i = 1:params.N

     J = J + ...
      z_mpc(i,:) * params.Q * z_mpc(i,:)' + ...
      u_mpc(i,:) * params.R * u_mpc(i,:)';

      % Model constraints
      constraints = [constraints, ...
        z_mpc(i+1,:)' == params.linear_A * z_mpc(i,:)' + params.linear_B * u_mpc(i,:)'];

      % Input constraints
      constraints = [constraints, ...
        -pi/3 <= u_mpc(i,2) <= pi/3];

  end

  % Terminal cost
  J = J + (z_mpc(params.N+1, :)) * params.Qf * (z_mpc(params.N+1, :))';

  % Options
  ops = sdpsettings('solver', 'quadprog');
  ops = sdpsettings(ops, 'verbose', 2);


  % Optimize
  parameters_in = z_mpc(1,:);
  solutions_out = {u_mpc, z_mpc};
  controller = optimizer(constraints, J, ops, parameters_in, solutions_out);


  
  
  % Find deviations
  
  circ = trajectory_planner();
  
  dists = sqrt((circ(1,:)-z(k,1)).^2 + (circ(2,:)-z(k,2)).^2)
  
  [min, idx] = min(dists);
  
  e(k,1) = min * sin(z(k,3));
  e(k,2) = -min * cos(z(k,3));
  e(k,3) = 0;
  e(k,4) = circ(idx,3) - z(k,3);
  
  

%   % snatch first predicted input
%   [U,~,~,~,controller] = controller{e};
%   predicted_inputs = value(U{1});
%   predicted_states = value(U{2});
%   u(k,:) = value(predicted_inputs(1,:));

u(k,:) = 10;

  end 

  % simulate the vehicle
  z(k+1,:) = car_sim(z(k,:), u(k,:), params)';