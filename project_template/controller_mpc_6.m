% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
%   N: MPC horizon length, dimension (1,1)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_mpc_6(Q,R,T,N,~)
% controller variables
persistent param yalmip_optimizer h
global A_aug B_aug C_aug L

load('system/parameters_building');
if isempty(h)
    h = [T; [building.a_Env_VC * building.T_Env;0;0]];
end
Th = h(1:3);
dh = h(4:6);

param = compute_controller_base_parameters();


% set point update
A = [param.A - eye(3), param.B; eye(3), zeros(3)];
b = [-param.Bd * dh; param.b_ref];
sp = A \ b;
T_sp = sp(1:3);
p_sp = sp(4:6);



% initialize controller, if not done already
% if isempty(param)
    [param, yalmip_optimizer] = init(Q,R,T,N,h,T_sp,p_sp);
% end

% observer update

% evaluate control action by solving MPC problem
[u_mpc,errorcode] = yalmip_optimizer(T);
if(isnan(u_mpc))
    pause;
end
if (errorcode ~= 0)
    warning('MPC6 infeasible');
end
p = u_mpc + p_sp;

% observer update
h = A_aug * h + B_aug * p + L * (T - C_aug * h);

end


%% 
function [param, yalmip_optimizer] = init(Q,R,T,N,h,T_sp,p_sp)

global A_aug B_aug C_aug L

% get basic controller parameters
param = compute_controller_base_parameters;

% design disturbance observer
A_aug = [param.A, param.Bd; zeros(3), eye(3)];
B_aug = [param.B; zeros(3)];
C_aug = [eye(3), zeros(3)];
P = [0, 0, 0, 0.5, 0.5, 0.5];
L = -place(A_aug', C_aug', P)';

% init state and disturbance estimate variables
% ...
% implement your MPC using Yalmip here
nx = size(param.A,1);
nu = size(param.B,2);
U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
T0 = sdpvar(nx,1,'full');
d0 = h(4:6);

objective = 0;
constraints = [];
constraints = [constraints, X{1} == T0 - T_sp];
Xcons = param.Xcons + repmat(param.T_sp,1,2) - T_sp;
Ucons = param.Xcons + repmat(param.p_sp,1,2) - p_sp;

for k = 1:N-1
    constraints = [constraints, X{k+1} == param.A * X{k} + param.B * U{k}];
    constraints = [constraints, Xcons(:,1) <= X{k+1} <= Xcons(:,2)];
    constraints = [constraints, Ucons(:,1) <= U{k} <= Ucons(:,2)];
    objective = objective + U{k}'*R*U{k} + X{k}'*Q*X{k};
end


% get terminal cost
[~, P_inf, ~] = dlqr(param.A, param.B, Q, R);
objective = objective + X{N}'*P_inf*X{N};

% terminal set constraint
[A_x, b_x] = compute_X_LQR(Q, R);
constraints = [constraints, A_x * X{N} <= b_x];

ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_optimizer = optimizer(constraints,objective,ops,T0,U{1});
end