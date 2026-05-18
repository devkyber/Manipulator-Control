function [value,isterminal,direction] = event_passed(t,x,param)
%% Unpack states
% q
q1 = x(1);
q2 = x(2);
q3 = x(3);
q4 = x(4);

%% Check event
    value = [param.q_d(1) - q1, param.q_d(2) - q2, param.q_d(3) - q3, param.q_d(4) - q4];
    isterminal = [0, 0, 0, 0];
    direction = [0, 0, 0, 0];

end
    