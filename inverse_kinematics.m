function q_sol = inverse_kinematics(p_d, q4abs, q4, branch, param)
% Analytic IK for 4-link planar arm with q4 chosen as a free variable.
%
% Inputs:
%   p_d: [x; y] desired endpoint position
%   q4abs: desired absolute final link angle
%   q4: chosen q4 value
%   branch: +1 or -1 for elbow branch
%   param: parameter struct with L1,L2,L3,L4
%
% Output:
%   q_sol: [q1; q2; q3; q4]

    x_d = p_d(1);
    y_d = p_d(2);

    L1 = param.L1;
    L2 = param.L2;
    L3 = param.L3;
    L4 = param.L4;

    theta3 = q4abs - q4;
    theta4 = q4abs;

    % Wrist point: endpoint of link 2.
    p_w = [x_d; y_d] ...
        - L3*[cos(theta3); sin(theta3)] ...
        - L4*[cos(theta4); sin(theta4)];

    x_w = p_w(1);
    y_w = p_w(2);

    % 2-link analytic IK for links L1 and L2
    D = (x_w^2 + y_w^2 - L1^2 - L2^2)/(2*L1*L2);

    tol = 1e-10;
    if D > 1 + tol || D < -1 - tol
        error('target unreachable for this q4 value.');
    end

    D = max(min(D,1),-1);

    cosq2 = D;
    sinq2 = branch * sqrt(1 - D^2);

    q2 = atan2(sinq2, cosq2);
    q1 = atan2(y_w, x_w) - atan2(L2*sinq2, L1 + L2*cosq2);

    q3 = theta3 - q1 - q2;

    q_sol = [q1; q2; q3; q4];
end