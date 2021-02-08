%solving IK

close all;
clear all;
clc;

import casadi.*
opti = casadi.Opti();


%% Create the Gripper (Initially at Home Position)
L_act = 0;     
L0 = 10;     

L1 = 20;   L2 = 40.74;   L3 = 53.38;   
L4 = 72.8;   L11 = 76.714;  L12 = 57.76;    
L13 = 10;  L14 = 10;


L5 = L4;    L6 = L3;    L7 = L2;
L10 = L13;  L9 = L12;   L8 = L11;
L15 = L14;

CurrentLinks = [L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 L11 L12 L13 L14 L15]';


PointM = opti.variable(1,2);
PointN = opti.variable(1,2);

%% contact points constraints (y=12)
opti.subject_to(0<PointM(1));
opti.subject_to(0<PointN(1));


% cube object y=+/-12
rock_y=30;
    
offset = 14;

opti.subject_to(PointM(2)==rock_y+offset);
opti.subject_to(PointN(2)==-rock_y+offset);
    % init 
opti.set_initial(PointM,[80,rock_y+offset]);
opti.set_initial(PointN,[70,-rock_y+offset]);

[Theta, JointCoord] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM, PointN);
%% looc constraint
D_x = JointCoord(1,4);      D_y = JointCoord(2,4);
%strict D_y constraint
D_x2 = JointCoord(1,15);    D_y2 = JointCoord(2,15);

opti.subject_to(D_x == D_x2);
opti.subject_to(D_y == D_y2);
%opti.minimize(abs(PointM(1)-PointN(1)))

opti.subject_to(-pi/4 < Theta(1) < pi/4); opti.subject_to(pi/2 < Theta(3)); opti.subject_to(pi/8 < Theta(8) < pi/2);
   

opti.solver('ipopt');
sol = opti.solve();

sol.value(PointM)
sol.value(PointN)

PointM_sol = sol.value(PointM);
PointN_sol = sol.value(PointN);
[Theta_sol, JointCoord_sol] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM_sol, PointN_sol);
 F_actuator_total = 1; % (Newtons)
 Actuator_joint_num = 2; % We are assuming the pin joints are at the same point for now
[F_M_S, F_N_S] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta_sol)

DrawingGripper(JointCoord_sol,[10,0])

