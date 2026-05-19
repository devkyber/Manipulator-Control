%% PD Control of a Manipulator
%% Set up parameters
[param,param4func] = set_params();

%% Set initial conditions
% state vectors with four joint angles and four joint angular velocities 
x0 = [0; pi; 0; pi; 0; 0; 0; 0];  % x = [q1 q2 q3 q4 dq1 dq2 dq3 dq4] 
t0 = 0:0.01:10;

%% ODE options
event1 = @(t,X)event_passed(t,X,param);
options1 = odeset('RelTol', 1e-5, 'AbsTol', 1e-6, 'Events', event1);

%% Solve ODE
[t,x,te,xe] = ode45(@(t,x)ode_PD_control(t,x,param), t0, x0, options1);

%% Animation
animate(t,x,param);

