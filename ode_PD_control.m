function dxdt = ode_PD_control(t,x,param)
%% Set up Prarameters
    [param,param4func] = set_params();
    q_d = param.q_d;
    dq_d = param.dq_d;
    Kp = param.Kp;
    Kd = param.Kd;
    
    q = x(1:4);
    dq = x(5:8);
    D = fcn_D(q,param4func);
    C = fcn_C(q,dq,param4func);
    G = fcn_G(q,param4func);
    B = fcn_B(q,param4func);

%% Calculation
    u = G + Kp*(q_d - q) + Kd*(dq_d - dq);    % control input
    ddq = inv(D) * (B*u - C*dq - G);
    
%% Return
    dxdt = [dq;ddq];

end