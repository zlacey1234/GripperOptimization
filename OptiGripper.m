close all;
clear all;
clc;

import casadi.*


opti = casadi.Opti();


%% Create the Gripper (Initially at Home Position)
L_act = 0;       L1 = opti.variable();
L2 = opti.variable();

%L16 = opti.variable();
%theta16 = 0;

L0 = 0;      
L3 = opti.variable();     L4 = opti.variable();     L11 = opti.variable();   
L12 = opti.variable();    L13 = opti.variable();    L14 = opti.variable();

L5 = L4;    L6 = L3;    L7 = L2;
L10 = L13;  L9 = L12;   L8 = L11;
L15 = L14;

CurrentLinks = [L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 L11 L12 L13 L14 L15]';

%% Contact point constraint
% Assuming a shape with sides on the positive x quadrants

PointM = opti.variable(1,2);
PointN = opti.variable(1,2);

%% contact points constraints (y=12)
% cube object y=+/-12
rock_y=12;
opti.subject_to(0<PointM(1));
opti.subject_to(0<PointN(1));
%opti.subject_to(PointM(1) == PointN(1));
opti.subject_to(PointM(2)==rock_y);
opti.subject_to(PointN(2)==-rock_y);
% init 
opti.set_initial(PointM,[12,rock_y]);
opti.set_initial(PointN,[12,-rock_y]);

%% Contact points (y=2x)
%opti.subject_to(PointM(2)==2*PointM(1));
%opti.subject_to(PointN(2)==-2*PointN(1));
%opti.set_initial(PointM,[2,4]);
%opti.set_initial(PointN,[2,-4]);

 

%% inverse kinematics 
[Theta, JointCoord] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM, PointN);

%% Theta constraints
opti.subject_to(-pi/4 < Theta(1) < pi/4);
opti.subject_to(pi/2 < Theta(3));
opti.subject_to(pi/8 < Theta(8) < pi/2);


%% Point D constraint
D_x = JointCoord(1,4);
D_y = JointCoord(2,4);
%opti.subject_to(D_y == 0);

%opti.subject_to(L1 < D_x);
opti.subject_to(-L2 < D_y < L2);

 %% Reaction force constraint
 F_actuator_total = 20; % (Newtons)
 Actuator_joint_num = 2; % We are assuming the pin joints are at the same point for now
        
 [F_M, F_N] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta);
 
if(true)
    %constraint gripping force by gear ratio
    min_ratio = 5; max_ratio = 10;
    opti.subject_to(min_ratio < (F_M(2)-F_N(2))/F_actuator_total < max_ratio);
else
    %constraint gripping force by newton
    min_grasping = 20; %N newton
    opti.subject_to(min_grasping<F_M(2)<20*min_grasping);
    opti.subject_to(min_grasping<-F_N(2)<20*min_grasping);
    
end

% constraint in X
%opti.subject_to(0<=F_M(1));
%opti.subject_to(0<=F_N(1));


 %% link length constraints (all positive and not too long) 
 minL = 3; maxL = 21;
opti.subject_to(minL < L1 < maxL); opti.subject_to(minL < L2 < maxL); opti.subject_to(minL < L3 < maxL);
opti.subject_to(minL < L4 < maxL); opti.subject_to(minL < L11 < maxL); opti.subject_to(minL < L12 < maxL);
opti.subject_to(minL < L13 < maxL); opti.subject_to(minL < L14 < maxL); 

%% Other length constraints
%opti.subject_to((L4 + L13)<maxL);


%% initial guess for links
opti.set_initial(L1,8);
opti.set_initial(L2,8);
opti.set_initial(L3,8);
opti.set_initial(L4,8);
opti.set_initial(L11,20);
opti.set_initial(L12,13);
opti.set_initial(L13,10);
opti.set_initial(L14,5);

%% 

%% Objective function
%opti.minimize(PointM(1)+PointN(1));
%opti.minimize(1/(F_M(2)-F_N(2)))
%opti.minimize((F_M(2)-F_N(2))/F_actuator_total)
opti.minimize(L12+L15);
%opti.minimize(D_y^2)
 
%run solver 
opti.solver('ipopt');
sol = opti.solve();


CurrentLinks_sol = [sol.value(L1) sol.value(L2) sol.value(L3) sol.value(L4) sol.value(L5) sol.value(L6)...
    sol.value(L7) sol.value(L8) sol.value(L9) sol.value(L10) sol.value(L11) sol.value(L12) sol.value(L13)...
    sol.value(L14) sol.value(L15)]';
PointM_sol = sol.value(PointM);
PointN_sol = sol.value(PointN);
[Theta_sol, JointCoord_sol] = InverseKinematicsGripper2D(L_act, sol.value(L0), CurrentLinks_sol, PointM_sol, PointN_sol);


DrawingGripper(JointCoord_sol,[15,0])

sol.value(F_M)
sol.value(F_N)
D_x = JointCoord_sol(1,4)
D_y = JointCoord_sol(2,4)
 
        
     