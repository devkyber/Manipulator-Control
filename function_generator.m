%% generate 2 link inverted pendulum
%% Author: Hae-Won Park
%% Modified by: Won Dong Shin

%% Add path
addpath('other_functions');

%% Initial setup
syms q1 q2 q3 q4 real % generalized Coordinate
syms dq1 dq2 dq3 dq4 real        
syms M1 M2 M3 M4 real % mass of each link
syms J1 J2 J3 J4 real % inertia of each link
syms L1 L2 L3 L4 real % length of each link
syms g real  % gravity acceleration        

q =[q1, q2, q3, q4];
dq = [dq1, dq2, dq3, dq4];
[c_list_q_stance, m_list_q_stance] = gen_c_m_lists(q, 'q') ;
[c_list_dq_stance, m_list_dq_stance] = gen_c_m_lists(dq, 'dq') ;

params = {'M1', M1 ; ...
          'M2', M2 ; ...
          'M3', M3 ; ...
          'M4', M4 ; ...
          'J1', J1 ; ...
          'J2', J2 ; ...
          'J3', J3 ; ...
          'J4', J4 ; ...
          'L1', L1 ; ...
          'L2', L2 ; ...
          'L3', L3 ; ...
          'L4', L4 ; ...
          'g', g;
          } ;

data = {} ;
for j=1:length(params)
    data(j,1) = params(j,1) ;
    data(j,2) = {['params(' num2str(j) ')']} ;
end

m_list_params = data;

%% forward kinematics
rot = @(th)[cos(th), -sin(th) ; sin(th), cos(th)]; % counterclockwise 2X2 rotation matrix (use 'th' for the angle ex) cos(th) )
q2abs = q1 + q2; %absolute angle for link 2
q3abs = q1 + q2 + q3;
q4abs = q1 + q2 + q3 + q4;

dq2abs = dq1 + dq2 ; %absolute angular velocity for link 2
dq3abs = dq1 + dq2 + dq3;
dq4abs = dq1 + dq2 + dq3 + dq4;

% Calculate end position of each link
p1 = rot(q1)*[L1 ; 0]; % [L1*cos(q1) ; L1*sin(q1)];
p2 = p1 + rot(q2abs)*[L2 ; 0]; %p1 + [L2*cos(q2abs) ; L2*sin(q2abs)]; 
p3 = p2 + rot(q3abs)*[L3 ; 0]; %p2 + [L3*cos(q3abs) ; L3*sin(q3abs)];
p4 = p3 + rot(q4abs)*[L4 ; 0]; %p3 + [L4*cos(q4abs) ; L4*sin(q4abs)];

% Calculate the COM position for each link, assuming the COM is located at the midpoint of each link
p1_COM = rot(q1)*[L1/2 ; 0];
p2_COM = p1 + rot(q2abs)*[L2/2 ; 0];
p3_COM = p2 + rot(q3abs)*[L3/2 ; 0];
p4_COM = p3 + rot(q4abs)*[L4/2 ; 0];

% Total mass
M = M1 + M2 + M3 + M4;

% Resultant CoM of the system
p_COM = M1/M * (p1_COM) + M2/M * (p2_COM) + M3/M * (p3_COM) + M4/M * (p4_COM);

%% Velocity kinematics ; velocity of center of mass using jacobian matrix 
v0 = jacobian([0;0],q)*dq'; 
v1_COM = jacobian(p1_COM,q)*dq';
v2_COM = jacobian(p2_COM,q)*dq';
v3_COM = jacobian(p3_COM,q)*dq';
v4_COM = jacobian(p4_COM,q)*dq';


%% Kinetic Energy (Consider both translational and rotational velocities)
KE_1 = 1/2*M1*(v1_COM.' * v1_COM) + 1/2*J1*dq1^2; % .' : transpose
KE_2 = 1/2*M2*(v2_COM.' * v2_COM) + 1/2*J2*dq2abs^2;
KE_3 = 1/2*M3*(v3_COM.' * v3_COM) + 1/2*J3*dq3abs^2;
KE_4 = 1/2*M4*(v4_COM.' * v4_COM) + 1/2*J4*dq4abs^2;

KE = KE_1 + KE_2 + KE_3 + KE_4;

%% Potential Energy
PE = M1*g*p1_COM(2) + M2*g*p2_COM(2) + M3*g*p3_COM(2) + M4*g*p4_COM(2); % using height of COM

%% Controlled joints
Upsilon = [q1, q2, q3, q4] ;
        
%% Euler-Lagrange Equation
[D, C, G, B] = std_dynamics(KE,PE,q,dq,Upsilon);
m_output_dir = pwd;

write_fcn_m(fullfile(m_output_dir,'fcn_D.m'), {'q','params'}, ...
    [m_list_q_stance; m_list_params], {D,'D'});

write_fcn_m(fullfile(m_output_dir,'fcn_C.m'), {'q','dq','params'}, ...
    [m_list_q_stance; m_list_dq_stance; m_list_params], {C,'C'});

write_fcn_m(fullfile(m_output_dir,'fcn_G.m'), {'q','params'}, ...
    [m_list_q_stance; m_list_params], {G,'G'});

write_fcn_m(fullfile(m_output_dir,'fcn_B.m'), {'q','params'}, ...
    [m_list_q_stance; m_list_params], {B,'B'});


%% Task Space
x_task = p4;
J = jacobian(x_task, q);

write_fcn_m(fullfile(m_output_dir,'fcn_x.m'), {'q','params'}, ...
    [m_list_q_stance; m_list_params], {x_task,'x_task'});

write_fcn_m(fullfile(m_output_dir,'fcn_J.m'), {'q','params'}, ...
    [m_list_q_stance; m_list_params], {J,'J'});

