clear all
close all

S = csvread('centerline_mpc_states_log.csv');
U = csvread('centerline_mpc_inputs_log.csv');

figure
plot(S(1:end,1))
axis([200 510 -2 2])
grid

figure
plot(S(:,2)*180/pi)
axis([200 505 -180 50])
grid

figure
plot(U*180/pi)
grid
axis([34 80 -20 50])





