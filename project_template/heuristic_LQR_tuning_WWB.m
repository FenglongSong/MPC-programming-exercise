% BRRIEF:
%   Template for tuning of Q and R matrices for LQR controller as described 
%   in task 6.
% INPUT:
%   n_samples:  Number of samples considered for the tuning.
%   T0:         Initial condition
%   T_sp:       Set point
%   scen:       Disturbance scenario
% OUTPUT:
%   Q, R: Describes stage cost of LQR controller (x^T Q x + u^T R u)

function [Q, R] = heuristic_LQR_tuning_WWB(n_samples, T0, T_sp, scen)

figure(2); set(gcf, 'WindowStyle' ,'docked'); grid on; hold on
xlabel('Energy consumption [kWh]'); 
ylabel('Relative norm of steady state deviation');
%...

R = eye(3);
Q_best = eye(3);
dT_best = Inf;
% param = compute_controller_base_parameters;
for index = 1:n_samples
    if mod(index, 200)==0
        fprintf('LQR Tuning Processed %d percent\n', 100*index/n_samples);
        
    end
    Q_idx = [randi([1 10e6]), 0, 0;
            0, randi([1 10e6]), 0;
            0, 0, randi([1 10e6])];
    clear controller_lqr;
    [T, p, ~, T_v, p_v] = simulate_building(T0, @controller_lqr, Q_idx, R, scen, 0);
    
    dT_relative = norm(T_sp - T(:, 15))/norm(T_sp - T0);
    power_sum = sum(abs(p), 'all')/1000/60;
    
    if T_v==true
        fprintf('Occur: T_v==true, index: %d\n', index)
        scatter(power_sum, dT_relative, [], 'red');
    elseif p_v==true
        fprintf('Occur: p_v==true, index: %d\n', index)
        scatter(power_sum, dT_relative, [], 'blue');
    else
        scatter(power_sum, dT_relative, [], 'green');
        fprintf('power_sum = %d\n', power_sum);
        if power_sum<16
            if dT_relative < dT_best
                dT_best = dT_relative;
                Q_best = Q_idx;
                fprintf('Best power_sum = %.3f, Best dT_relative = %.3f\n', power_sum, dT_relative);
                scatter(power_sum, dT_relative, [], 'black', 'filled');
            end
        end
    end
end
%...

Q = Q_best;
end
