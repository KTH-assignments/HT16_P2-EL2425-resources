clear all
close all

S = csvread('centerline_mpc_states_log.csv');
U = csvread('centerline_mpc_inputs_log.csv');

figure
plot(S(1:490,1))
grid

figure
plot(S(1:490,2)*180/pi)
grid

figure
plot(U(1:70)*180/pi)
grid




