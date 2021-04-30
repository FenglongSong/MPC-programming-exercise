% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_lqr(Q, R, T, ~, ~)
% controller variables
persistent param;

% initialize controller, if not done already
if isempty(param)
    param = init(Q, R);
%     param = init(eye(3), eye(3));
end

% Task 6:
% compute control action
% % [P_inf, ~, ~] = idare(param.A, param.B, Q, R);
% F_inf = - (param.B'*P_inf*param.B + R) \ (param.B'*P_inf*param.A);
% p = param.p_sp + F_inf * (T - param.T_sp);

[Klqr,~,~] = dlqr(param.A, param.B, Q, R);
p = param.p_sp - Klqr * (T - param.T_sp);
end

function param = init(Q, R)
% get basic controller parameters
param = compute_controller_base_parameters;
% add additional parameters if necessary, e.g.
% param.F = ...
end