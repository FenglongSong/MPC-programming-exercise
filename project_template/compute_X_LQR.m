% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% INPUT:
%   Q, R: State and input weighting matrix, dimension (3,3)
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}

function [A_x, b_x] = compute_X_LQR(Q, R)
    % get basic controller parameters
    persistent param;
    if isempty(param)
        param = compute_controller_base_parameters;
    end
    % implement the X_LQR computation and assign the result
    % computes a control invariant set for LTI system x^+ = A*x+B*u
    system = LTISystem('A', param.A, 'B', param.B);
    system.x.min = param.Xcons(:,1);
    system.x.max = param.Xcons(:,2);
    system.u.min = param.Ucons(:,1);
    system.u.max = param.Ucons(:,2);
    InvSet = system.invariantSet();
    figure(3);
    InvSet.plot();
    
    A_x = InvSet.A;
    b_x = InvSet.b;
end