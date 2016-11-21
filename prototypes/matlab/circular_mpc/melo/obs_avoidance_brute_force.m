% Created by Pedro Lima (pfrdal@kth.se) and Valerio Turri (turri@kth.se)
% EL2700 MPC Course Prof. Mikael Johansson
%%
yalmip('clear')

clear all;
close all;
clc

%% Parameters definition
% model parameters
params.Ts                   = 0.01;                     % sampling time (both of MPC and simulated vehicle)
params.nstates              = 3;                        % number of states
params.ninputs              = 1;                        % number of inputs
params.l_f                  = 0.17;
params.l_r                  = 0.16;
params.l_q                  = params.l_r / (params.l_r + params.l_f);

% control parameters
params.N                    = 20;                        % The horizon
params.Q                    = [1 0 0; 0 1 0; 0 0 8];
% params.Q                    = 100 * eye(3);
params.R                    = 1;

% initial conditions
params.x0                   = 0;                        % initial x coordinate
params.y0                   = -1.5;                        % initial y coordinate
params.v0                   = 1;                       % initial speed
params.psi0                 = 2*pi;                        % initial heading angle

params.N_max                = 1000;                        % maximum number of simulation steps

%% Simulation environment
% initialization

z(1,:)  = [params.x0, params.y0, params.psi0]; % definition of the intial state
u(1)  = 0;
k = 0;
circ = trajectory();
figure
plot(circ(:,1), circ(:,2))
hold on
axis equal

while (k < params.N_max)
  k = k+1;
  dists = sqrt((circ(:,1)-z(k,1)).^2 + (circ(:,2)-z(k,2)).^2);
  [~, idx] = min(dists);
  
% mov_ref(k,:) = circ(mod(idx+10, 360) + 1, :);
  mov_ref(k,:) = circ(idx, :);
  
 if (k > 1)
    if (mov_ref(k,3) < mov_ref(k-1,3))
        mov_ref(k,3) = mov_ref(k,3) + 2*pi;
        circ(idx,3) = circ(idx,3) + 2*pi;
    end
 end

%% MELO PART
if (k == 1)
params.linear_A = [1 0 -sin(mov_ref(k,3) + atan((16*tan(0))/33))/100;0 1 cos(mov_ref(k,3) + atan((16*tan(0))/33))/100;0 0 1];
params.linear_B = [-(4*sin(mov_ref(k,3) + atan((16*tan(0))/33))*(tan(0)^2 + 1))/(825*((256*tan(0)^2)/1089 + 1));(4*cos(mov_ref(k,3) + atan((16*tan(0))/33))*(tan(0)^2 + 1))/(825*((256*tan(0)^2)/1089 + 1));(tan(0)^2 + 1)/(33*((256*tan(0)^2)/1089 + 1)^(1/2)) - (256*tan(0)^2*(tan(0)^2 + 1))/(35937*((256*tan(0)^2)/1089 + 1)^(3/2))];
end

if (k > 1)
params.linear_A = [1 0 -sin(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))/100;0 1 cos(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))/100;0 0 1];
params.linear_B = [-(4*sin(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))*(tan(u(k-1,1))^2 + 1))/(825*((256*tan(u(k-1,1))^2)/1089 + 1));(4*cos(mov_ref(k,3) + atan((16*tan(u(k-1,1)))/33))*(tan(u(k-1,1))^2 + 1))/(825*((256*tan(u(k-1,1))^2)/1089 + 1));(tan(u(k-1,1))^2 + 1)/(33*((256*tan(u(k-1,1))^2)/1089 + 1)^(1/2)) - (256*tan(u(k-1,1))^2*(tan(u(k-1,1))^2 + 1))/(35937*((256*tan(u(k-1,1))^2)/1089 + 1)^(3/2))];
end

%% ALEX PART
%  params.linear_A = [
%     1 0 -params.Ts * params.v0 * sin(mov_ref(k,3) + beta);
%     0 1 params.Ts * params.v0 * cos(mov_ref(k,3) + beta);
%     0 0 1];
%   
%   params.linear_B = [
%     -params.Ts * params.v0 * sin(mov_ref(k,3) + beta) * p;
%     params.Ts * params.v0 * cos(mov_ref(k,3) + beta) * p;
%     params.Ts * params.v0 / params.l_r * cos(beta) * p];

%%   Ensure stability
  [params.Qf,~,~, err] = dare(params.linear_A, params.linear_B, params.Q, params.R);
  if(err == -1 || err == -2)
    params.Qf = params.Q;
  end


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
      -pi/3 <= u_mpc(i,1) <= pi/3];
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
  z(k+1,:) = car_sim(z(k,:), u(k,:), params);

  
  
end

plot(z(:,1), z(:,2))
hold off

figure
hold on
plot(circ(:,1), circ(:,2))
plot(mov_ref(:,1), mov_ref(:,2))
axis equal
hold off

figure
hold on
plot(circ(:,1), circ(:,3))
plot(z(:,1),z(:,3))
axis equal