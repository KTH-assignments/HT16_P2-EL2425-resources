clear all
close all

S = csvread('centerline_pid_states_log.csv');
U = csvread('centerline_pid_inputs_log.csv');

figure
plot(S(1:450,1)*180/pi)
grid

figure
plot(U(1:450)*180/pi)
grid




