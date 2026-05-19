function dxdt = ode_PD_control(t,x,param)
%% Set up Parameters
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
    
    % % Forward Dynamics: Torque Input --> Joint Acceleration 
    % u = G + Kp*(q_d - q) + Kd*(dq_d - dq);  % control input
    % ddq = D  \ (B*u - C*dq - G);
    

    % % Inverse Dynamics: Desired Acceleration --> Torque  
    % a_q = Kp*(q_d - q) + Kd*(dq_d - dq); % Suppose ddq_d zero / Joint Space 
    % u = D*a_q + C*dq + G; 
    % ddq = D \ (B*u - C*dq - G);
    % % ddq = a_q; 

    % Inverse Dynamics + Task Space 
    x_task = fcn_x(q, param4func);
    J = fcn_J(q, param4func);
    dx = J * dq; 

    x_d = param.p_d;
    dx_d = param.dp_d;
    ddx_d = param.ddp_d;
   
    Kpx = param.Kpx;
    Kdx = param.Kdx;

    a_x = ddx_d + Kdx*(dx_d - dx) + Kpx*(x_d - x_task);

    Jdotdq = zeros(2,1);

    lambda = param.lambda;
    a_q = J.' * ((J*J.' + lambda^2*eye(2)) \ (a_x - Jdotdq));

    u = D*a_q + C*dq + G;

    ddq = D \ (B*u - C*dq - G);



%% Return
    dxdt = [dq;ddq];

end