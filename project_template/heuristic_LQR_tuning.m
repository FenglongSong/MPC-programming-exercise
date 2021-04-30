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

function [Q, R] = heuristic_LQR_tuning(n_samples, T0, T_sp, scen)

R = eye(3);

figure(2); 
% set(gcf, 'WindowStyle' ,'docked'); 
grid on; hold on
xlabel('Energy consumption [kWh]');
ylabel('Relative norm of steady state deviation');

Q_idx = eye(3);
Q_best = eye(3);
dT_best = 1e6;

for index = 1:n_samples
    Q_idx(1,1) = randi([1,1e7]);
    Q_idx(2,2) = randi([1,1e7]);
    Q_idx(3,3) = randi([1,1e7]);
    
    clear controller_lqr
    [T, p, ~, ~, T_v, p_v] = simulate_building(T0, @controller_lqr, Q_idx, R, scen, 0);
    
    dT_relative = norm(T_sp-T(:,15), 2) / norm(T_sp-T0, 2);
    power_sum = sum(abs(p), 'all')/1000/60;
    
    if T_v == true
        scatter(power_sum, dT_relative, 'red');
    elseif p_v == true
        scatter(power_sum, dT_relative, 'blue');
    else
        scatter(power_sum, dT_relative, 'green');
    end
    
    if ~T_v && ~p_v
        if power_sum <= 16
            if dT_relative < dT_best
                dT_best = dT_relative;
                power_best = power_sum;
                Q_best = Q_idx;
            end
        end
    end
    
    
    
end

Q = Q_best;
scatter(power_best, dT_best, 'black', 'filled');

end
