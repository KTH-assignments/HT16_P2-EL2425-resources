clear all
close all
st = 150;
en = 380;
M = csvread('circular_mpc_inputs_log.csv');

figure
plot(M(st:en)*180/pi)
grid

