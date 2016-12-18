clear all
close all
st = 1;
en = 150;
M = csvread('circular_mpc_inputs_log.csv');

axis equal

figure
plot(M(1:150)*180/pi)
grid

