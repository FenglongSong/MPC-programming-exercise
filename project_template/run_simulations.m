%% Init
clear all
close all
addpath(genpath(cd));
rng(1234)
load('system/parameters_scenarios.mat')

%% Task 5:
param = compute_controller_base_parameters();
T_sp = param.T_sp;
T0_example = T_sp;
figure; 
% set(gcf, 'WindowStyle' ,'docked');
[T,~,~,t] = simulate_building(T0_example);


%% Example
% figure; set(gcf, 'WindowStyle' ,'docked');
% clear persisten variables of function controller_example
% clear controller_example
% execute simulation
% [T,~,~,t] = simulate_building(T0_example,@controller_example);


%% Unconstrained optimal control
disp('Unconstraint optimal control');

% Uncontrolled system
%figure(1); set(gcf, 'WindowStyle' ,'docked');
T0_1 = T_sp + [-2.25; 1.75; 0.75];
T0_2 = T_sp + [1.5; 2.75; -0.25];
% Tuning of LQR on first initial condition

clear controller_lqr
% Task 6: Tune LQR parameters
[Q,R] = heuristic_LQR_tuning(25, T0_1, T_sp, scen1);

% Task 7: close-loop simulate with initial condition 1
% [T, ~, ~, t] = simulate_building(T0_1, @controller_lqr, Q, R, scen1, 0);
% figure;


% Task 8: close-loop simulate with initial condition 2
% [T, ~, ~, t] = simulate_building(T0_2, @controller_lqr, Q, R, scen1, 0);

% pause;


%% From LQR to MPC
disp('First MPC'); 
% Task 11
[T, ~, ~, t] = simulate_building(T0_2, @controller_mpc_1, Q, R, scen1, 0);
% pause;


%% MPC with guarantees
disp('MPC with guarantees');

% pause;


%% Soft-constrained MPC
disp('Soft-constrained MPC');

% pause;


%% Offset-free MPC
disp('Offset-free MPC');

% pause;


%% Comparison using forces
disp('MPC Implementation with FORCES Pro');
