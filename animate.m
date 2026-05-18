function [] = animate(t,x,param)
    len = max(size(t));
    rot = @(th)[cos(th) -sin(th); sin(th) cos(th)];
    
    for i = 1:len
        p1 = rot(x(i,1))*[param.L1;0];
        p2 = p1 + rot(x(i,1) + x(i,2)) * [param.L2;0];
        p3 = p2 + rot(x(i,1) + x(i,2) + x(i,3))*[param.L3;0];
        p4 = p3 + rot(x(i,1) + x(i,2) + x(i,3) + x(i,4))*[param.L4;0];
        
        hold on
        plot([0 p1(1)],[0,p1(2)], 'r');    
        plot([p1(1) p2(1)],[p1(2) p2(2)], 'g');
        plot([p2(1) p3(1)],[p2(2) p3(2)], 'b')
        plot([p3(1) p4(1)],[p3(2) p4(2)], 'w')      
        
        text(-3,4.5,['t = ',num2str(t(i)), 'sec']);
        text(-3,4,['\theta_1 = ',num2str( mod(180/pi*x(i,1),360) ), char(176)]);
        text(-3,3.5,['\theta_2 = ',num2str( mod(180/pi*x(i,2),360) ), char(176)]);
        text(-3,3,['\theta_3 = ',num2str( mod(180/pi*x(i,3),360) ), char(176)]);
        text(-3,2.5,['\theta_4 = ',num2str( mod(180/pi*x(i,4),360) ), char(176)]);
        xlabel('x position [-]');
        ylabel('y position [-]');
        grid on;
        
        % constrain the view
        axis([-3 3 -1 5])
        F(i) = getframe;
        clf
    end
end