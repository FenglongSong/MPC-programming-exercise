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
[T,p,~,~,T_v,p_v] = simulate_building(T0_example);


%% Unconstrained optimal control
disp('Unconstraint optimal control');

% Uncontrolled system
T0_1 = T_sp + [-2.25; 1.75; 0.75];
T0_2 = T_sp + [1.5; 2.75; -0.25];

% Tuning of LQR on first initial condition
clear controller_lqr
% Task 6: Tune LQR parameters
% [Q_1, R] = heuristic_LQR_tuning(2500, T0_1, T_sp, scen1);
Q_1 = [4109479,0,0; 0,2064982,0; 0,0,2076632];
R = eye(3);

% [Q_2, R] = heuristic_LQR_tuning(2500, T0_2, T_sp, scen1);
Q2 = [3618351,0,0; 0,912436,0; 0,0,452536];

% Task 7: close-loop simulate with initial condition 1
[T,p,~,~,T_v,p_v] = simulate_building(T0_1, @controller_lqr, Q_1, R, scen1, 1);


% Task 8: close-loop simulate with initial condition 2
% [T, ~, ~, t] = simulate_building(T0_2, @controller_lqr, Q_2, R, scen1, 1);

% pause;


%% From LQR to MPC
disp('First MPC'); 
% Task 9: Invariant set
[Ax, bx] = compute_X_LQR(Q_1, R);

% Task 11
figure;
[T, ~, ~, t] = simulate_building(T0_1, @controller_mpc_1, Q_1, R, scen1, 1);
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
