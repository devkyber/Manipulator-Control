function [param,param4func] = set_params()
    %% 
    param.M1 = 1;
    param.M2 = 1;
    param.M3 = 1;
    param.M4 = 1;
    param.J1 = 1;
    param.J2 = 1;
    param.J3 = 1;
    param.J4 = 1;
    param.L1 = 1;
    param.L2 = 1;
    param.L3 = 1;
    param.L4 = 1;
    param.g = 9.81;
    
    %% Desired values
    % param.q_d = [pi/4; 0; 0; 0];    %Desired Position of Joints (Feel free to modify)

    param.p_d = [1; 3];
    param.theta4 = 2*pi/3;
    param.q4 = pi/6;
    param.branch = 1;

    param.q_d = ik_analytic(param.p_d, param.theta4, param.q4, param.branch, param);
  
    param.dq_d = [0; 0; 0; 0];         %Desired Velocity of Joints
    param.Kp = 1000;                    %Proportional Gain
    param.Kd = 300;                    %Derivative Gain

    %% Format output
    param4func = [param.M1, param.M2, param.M3, param.M4, ...
                  param.J1, param.J2, param.J3, param.J4, ...
                  param.L1, param.L2, param.L3, param.L4, ...
                  param.g ];

end