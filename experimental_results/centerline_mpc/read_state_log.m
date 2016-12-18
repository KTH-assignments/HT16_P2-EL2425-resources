clear all
close all

S = csvread('centerline_mpc_states_log.csv');
U = csvread('centerline_mpc_inputs_log.csv');

figure
plot(S(:,1))
grid

figure
plot(S(:,2)*180/pi)
grid

figure
plot(U*180/pi)
grid




