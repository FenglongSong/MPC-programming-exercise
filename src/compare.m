figure;
hold on;

subplot(3,1,1);
plot(d(1,:),'color',[0 0.4470 0.7410],'LineWidth',1);
xlabel('t/min');
ylabel('d_{VC}');

subplot(3,1,2);
plot(d(2,:),'color',[0.8500 0.3250 0.0980],'LineWidth',1);
xlabel('t/min');
ylabel('d_{F1}');

subplot(3,1,3);
plot(d(3,:),'color',[0.9290 0.6940 0.1250],'LineWidth',1);
xlabel('t/min');
ylabel('d_{F2}');



%% Init
clear all
close all
addpath(genpath(cd));
rng(1234)
load('system/parameters_scenarios.mat')

%% Task 5:
param = compute_controller_base_parameters();
% figure; 
% [T,p,J_opt,t_wall,T_v,p_v] = simulate_building(param.T_sp);



% Uncontrolled system
T0_1 = param.T_sp + [-2.25; 1.75; 0.75];
T0_2 = param.T_sp + [1.5; 2.75; -0.25];

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


d = zeros(3, scen2.Nbar + 30);
d(1,36:43) = -1.5e4;
d(1,44:50) = -1e4;
d(2,37:43) = 5.4e3;
d(3,45:49) = 1.8e3;


% Task 11
figure;
clear controller_mpc_1;
[TA,pA,JA,t_wallA,T_vA,p_vA] = simulate_building(T0_1, @controller_mpc_1, Q_1, R, scen1, 1,30,d);
% figure;
clear controller_mpc_3_zyl;
% [TB,pB,JB,t_wallB,T_vB,p_vB] = simulate_building(T0_1, @controller_mpc_5_zyl, Q_1, R, scen2, 1, 30,d);
