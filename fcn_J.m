function [J] = fcn_J(q,params)

J = zeros(2,4);

  J(1,1)=- params(12)*sin(q(1) + q(2) + q(3) + q(4)) - params(10)*sin(q(1) + q(2)) - params(9)*...
         sin(q(1)) - params(11)*sin(q(1) + q(2) + q(3));
  J(1,2)=- params(12)*sin(q(1) + q(2) + q(3) + q(4)) - params(10)*sin(q(1) + q(2)) - params(11)*...
         sin(q(1) + q(2) + q(3));
  J(1,3)=- params(12)*sin(q(1) + q(2) + q(3) + q(4)) - params(11)*sin(q(1) + q(2) + q(3));
  J(1,4)=-params(12)*sin(q(1) + q(2) + q(3) + q(4));
  J(2,1)=params(12)*cos(q(1) + q(2) + q(3) + q(4)) + params(10)*cos(q(1) + q(2)) + params(9)*...
         cos(q(1)) + params(11)*cos(q(1) + q(2) + q(3));
  J(2,2)=params(12)*cos(q(1) + q(2) + q(3) + q(4)) + params(10)*cos(q(1) + q(2)) + params(11)*...
         cos(q(1) + q(2) + q(3));
  J(2,3)=params(12)*cos(q(1) + q(2) + q(3) + q(4)) + params(11)*cos(q(1) + q(2) + q(3));
  J(2,4)=params(12)*cos(q(1) + q(2) + q(3) + q(4));

 