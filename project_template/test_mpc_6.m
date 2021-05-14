%% Init
clear all
close all
addpath(genpath(cd));
rng(1234)
load('system/parameters_scenarios.mat')

param = compute_controller_base_parameters();
% Uncontrolled system
T0_1 = param.T_sp + [-2.25; 1.75; 0.75];
T0_2 = param.T_sp + [1.5; 2.75; -0.25];
Q_1 = diag([4973679, 5427908, 4949349]);
R = eye(3);

% Task 23:
figure;
clear controller_mpc_6;
simulate_building(T0_1, @controller_mpc_6, Q_1, R, scen3, 1);