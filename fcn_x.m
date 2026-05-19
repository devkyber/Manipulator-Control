function [x_task] = fcn_x(q,params)

x_task = zeros(2,1);

  x_task(1,1)=params(12)*cos(q(1) + q(2) + q(3) + q(4)) + params(10)*cos(q(1) + q(2)) + params(9)*...
         cos(q(1)) + params(11)*cos(q(1) + q(2) + q(3));
  x_task(2,1)=params(12)*sin(q(1) + q(2) + q(3) + q(4)) + params(10)*sin(q(1) + q(2)) + params(9)*...
         sin(q(1)) + params(11)*sin(q(1) + q(2) + q(3));

 