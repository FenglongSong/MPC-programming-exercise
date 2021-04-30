function param = compute_controller_base_parameters_WWB()
    % load truck parameters
    load('system/parameters_building');
    
    % Task 1: continuous time dynamics in state space form
    Ac = [-(building.a_F1_VC + building.a_F2_VC + building.a_Env_VC)/building.m_VC, building.a_F1_VC/building.m_VC, building.a_F2_VC/building.m_VC;
            building.a_F1_VC/building.m_F1, -(building.a_F1_VC + building.a_F2_F1)/building.m_F1, building.a_F2_F1/building.m_F1;
            building.a_F2_VC/building.m_F2, building.a_F2_F1/building.m_F2, -(building.a_F2_VC + building.a_F2_F1)/building.m_F2];
    Bc = [building.b_11/building.m_VC, building.b_12/building.m_VC, building.b_13/building.m_VC;
          building.b_21/building.m_F1, building.b_22/building.m_F1, building.b_23/building.m_F1;
          building.b_31/building.m_F2, building.b_32/building.m_F2, building.b_33/building.m_F2];
    Bdc = [1/building.m_VC, 0, 0;
           0, 1/building.m_F1, 0;
           0, 0, 1/building.m_F2];
    d = [building.d_VC; building.d_F1; building.d_F2];
    
    % Task 2: discretization
    Ts = 60;
    A = eye(3) + Ts*Ac;
    B = Ts*Bc;
    Bd = Ts*Bdc;
    
    % Task 3: set point computation
    b_ref = [25; -42; -18.5];
    C_ref = eye(3);
%     T_sp = b_ref;
%     p_sp = Bc \ (-Ac*T_sp - Bdc*d);
    sp = [eye(3)-A, -B; eye(3), zeros(3)] \ [Bd*d; b_ref];
    T_sp = sp(1:3);
    p_sp = sp(4:6);
    
    % Task 4: constraints for delta formulation
    Pcons = building.InputConstraints;
    Tcons = building.StateConstraints;
    Ucons = [Pcons(:, 1)-p_sp, Pcons(:, 2)-p_sp];
    Xcons = [Tcons(:, 1)-T_sp, Tcons(:, 2)-T_sp];
    
    % put everything together
    param.A = A;
    param.B = B;
    param.Bd = Bd;
    param.d = d;
    param.b_ref = b_ref;
    param.C_ref = C_ref;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
end
