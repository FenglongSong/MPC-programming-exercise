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
simulate_building(T0_example);


%% Unconstrained optimal control
disp('Unconstraint optimal control');

% Uncontrolled system
T0_1 = T_sp + [-2.25; 1.75; 0.75];
T0_2 = T_sp + [1.5; 2.75; -0.25];

% Tuning of LQR on first initial condition

% Task 6: Tune LQR parameters
clear controller_lqr;
% [Q_1, R] = heuristic_LQR_tuning(2500, T0_1, T_sp, scen1);
% Q_1 = [4109479,0,0; 0,2064982,0; 0,0,2076632];
Q_1 = diag([4973679, 5427908, 4949349]);
R = eye(3);

clear controller_lqr;
% [Q_2, R] = heuristic_LQR_tuning(2500, T0_2, T_sp, scen1);
% Q_2 = [3618351,0,0; 0,912436,0; 0,0,452536];
Q_2 = diag([5836090, 1342679, 3555303]);

% Task 7: close-loop simulate with initial condition 1
figure;
clear controller_lqr;
simulate_building(T0_1, @controller_lqr, Q_1, R, scen1, 1);

% Task 8: close-loop simulate with initial condition 2
figure;
clear controller_lqr;
simulate_building(T0_2, @controller_lqr, Q_2, R, scen1, 1);

% pause;


%% From LQR to MPC
disp('First MPC');
% Task 9: Invariant set
% clear controller_lqr; compute_X_LQR(Q_1, R);
% clear controller_lqr; compute_X_LQR(Q_2, R);


% Task 10: infinite horizon cost under LQR


% Task 11
figure;
clear controller_mpc_1;
[~,~,J_mpc1_T1] = simulate_building(T0_1, @controller_mpc_1, Q_1, R, scen1, 1);
figure;
clear controller_mpc_1;
[~,~,J_mpc1_T2] = simulate_building(T0_2, @controller_mpc_1, Q_2, R, scen1, 1);
% pause;


%% MPC with guarantees
disp('MPC with guarantees');

% Task 13: terminal set X_f = {0}, no terminal cost
figure;
clear controller_mpc_2;
[~,~,J_mpc2_T1] = simulate_building(T0_1, @controller_mpc_2, Q_1, R, scen1, 1);
figure;
clear controller_mpc_2;
[~,~,J_mpc2_T2] = simulate_building(T0_2, @controller_mpc_2, Q_2, R, scen1, 1);

% Task 14: terminal set X_f = X_LQR, with terminal cost
figure;
clear controller_mpc_3;
[~,~,J_mpc3_T1] = simulate_building(T0_1, @controller_mpc_3, Q_1, R, scen1, 1);
figure;
clear controller_mpc_3;
[~,~,J_mpc3_T2] = simulate_building(T0_2, @controller_mpc_3, Q_2, R, scen1, 1);


% Task 17: 
figure;
clear controller_mpc_3;
simulate_building(T0_1, @controller_mpc_3, Q_1, R, scen2, 1);

% pause;


%% Soft-constrained MPC
disp('Soft-constrained MPC');

% Task 18:
figure;
clear controller_mpc_4;
simulate_building(T0_2, @controller_mpc_4, Q_2, R, scen2, 1);

% Task 19:
figure;
clear controller_mpc_3;
simulate_building(T0_1, @controller_mpc_3, Q_1, R, scen1, 1);
figure;
clear controller_mpc_4;
simulate_building(T0_1, @controller_mpc_4, Q_1, R, scen1, 1);

% Task 20:
d = zeros(3, scen2.Nbar + 30);
d(1,36:43) = -1.5e4;
d(1,44:50) = -1e4;
d(2,37:43) = 5.4e3;
d(3,45:49) = 1.8e3;

figure;
clear controller_mpc_5;
simulate_building(T0_1, @controller_mpc_5, Q_1, R, scen2, 1, 30, d);

% pause;


%% Offset-free MPC
disp('Offset-free MPC');

% Task 21:
A_aug = [param.A, param.Bd; zeros(3,3), eye(3)];
B_aug = [param.B; zeros(3,3)];
C_aug = [eye(3), zeros(3,3)];
D_aug = zeros(3,3);

% Task 23:
figure;
clear controller_mpc_6;
simulate_building(T0_1, @controller_mpc_6, Q_1, R, scen3, 1);

% pause;


%% Comparison using forces
disp('MPC Implementation with FORCES Pro');
