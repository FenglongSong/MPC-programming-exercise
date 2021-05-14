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
    [K, ~, ~] = dlqr(param.A, param.B, Q, R);
    K = -K;
    systemLQR = LTISystem('A', param.A + param.B * K);
    
    absxmin = param.Xcons(:,1);
    absxmax = param.Xcons(:,2);
    absumin = param.Ucons(:,1);
    absumax = param.Ucons(:,2);

    Xp = Polyhedron('A',[eye(3); -eye(3); K; -K], 'b', [absxmax; -absxmin; absumax; -absumin]);
    systemLQR.x.with('setConstraint');
    systemLQR.x.setConstraint = Xp;
    
    InvSet = systemLQR.invariantSet();
%     figure;
%     InvSet.plot(), alpha(0.25);
%     title('Resulting State Constraints under LQR Control'), xlabel('\Delta T_{VC}'), ylabel('\Delta T_{F1}'), zlabel('\Delta T_{F2}')
    
    A_x = InvSet.A;
    b_x = InvSet.b;
end