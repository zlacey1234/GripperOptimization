close all;
clear all;
clc;

import casadi.*
opti = casadi.Opti();


%% Create the Gripper (Initially at Home Position)
L_act = 0;     
L0 = 0;     

L1 = opti.variable();   L2 = opti.variable();   L3 = opti.variable();   
L4 = opti.variable();   L11 = opti.variable();  L12 = opti.variable();    
L13 = opti.variable();  L14 = opti.variable();


L5 = L4;    L6 = L3;    L7 = L2;
L10 = L13;  L9 = L12;   L8 = L11;
L15 = L14;

CurrentLinks = [L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 L11 L12 L13 L14 L15]';

%% Contact point constraint
% Assuming a shape with sides on the positive x quadrants

PointM_C = opti.variable(1,2); PointN_C = opti.variable(1,2);
offset_BC = 10;
%% contact points constraints (y=12)
opti.subject_to(0<PointM_C(1)); opti.subject_to(0<PointN_C(1));

Cy=opti.variable(); opti.subject_to(0<Cy); opti.set_initial(Cy,90);
Cx=(PointM_C(1)-PointN_C(1)); 

opti.subject_to(PointM_C(2)==Cy+offset_BC); opti.subject_to(PointN_C(2)==-Cy+offset_BC);

opti.subject_to(PointN_C(2)<PointM_C(2)); opti.subject_to(PointN_C(1)<=PointM_C(1));
    % init 
opti.set_initial(PointM_C,[30,70]); opti.set_initial(PointN_C,[25,-70]);

 

%% inverse kinematics for passive adaptationyh
[Theta, JointCoord] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM_C, PointN_C);

%% inverse kinematics for moving actuator
% Set actuator position to be at A, use the same point D 
%[Theta_act, JointCoord_act] = GripperKinematic(L1, L0, CurrentLinks, JointCoord(1:2,4));

%% Theta constraints
opti.subject_to(-pi/4 < Theta(1) < pi/4);
opti.subject_to(pi/2 < Theta(3));
opti.subject_to(pi/8 < Theta(8) < pi/2);


%% Point D constraint
D_x = JointCoord(1,4);      D_y = JointCoord(2,4);
%strict D_y constraint
D_x2 = JointCoord(1,15);    D_y2 = JointCoord(2,15);

%% looc constraint
opti.subject_to(D_x == D_x2);
opti.subject_to(D_y == D_y2);


%% Point D boundary condition
%opti.subject_to(L1 < D_x);
%opti.subject_to(-L2 < D_y < L2);



 %% Reaction force constraint
 F_actuator_total = 1; % (Newtons)
 Actuator_joint_num = 2; % We are assuming the pin joints are at the same point for now
        
 [F_M, F_N] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta);
 

 %% calculate weight
 min_ratio = 10; min_sf =2;
 w_C = weightSF((F_M(2)-F_N(2)),min_ratio,min_sf,5,1/20);
 opti.subject_to(min_sf*min_ratio < (F_M(2)-F_N(2)));
 
 %% calculate density
 PointM_C = JointCoord(1:2,13); PointN_C = JointCoord(1:2,14);
 rho_C = hold_pdf([(PointM_C(1)-PointN_C(1)),PointM_C(2)-PointN_C(2)]);
 %rho_B = hold_pdf([20,PointM_B(2)-PointN_B(2)]);
% rho_B = hold_pdf(PointM_B-PointN_B);
% rho_B = hold_pdf([Bx,2*By]);


%% link length constraints (all positive and not too long) 
minL = 0.1; maxL = 200;
opti.subject_to(minL < L1 < maxL); opti.subject_to(minL < L2 < maxL); opti.subject_to(minL < L3 < maxL);
opti.subject_to(minL < L4 < maxL); opti.subject_to(minL < L11 < maxL); opti.subject_to(minL < L12 < maxL);
opti.subject_to(minL < L13 < maxL); opti.subject_to(minL < L14 < maxL); 

%% Other length constraints
%opti.subject_to((L4 + L13)<maxL);


%% initial guess for links
%amplification
amp = 7;
opti.set_initial(L1,amp*6);
opti.set_initial(L2,amp*6);
opti.set_initial(L3,amp*6);
opti.set_initial(L4,amp*6);
opti.set_initial(L11,amp*9);
opti.set_initial(L12,amp*8);
opti.set_initial(L13,amp*7);
opti.set_initial(L14,amp*2);

%% Metric calculations
%theta14_diff = (Theta(11)-Theta_act(11));
%opti.subject_to(0 < theta14_diff);

%% Objective function
%opti.minimize(w_a/w_a^2);
opti.minimize(w_C*rho_C);
 
%run solver 
opti.solver('ipopt');
sol = opti.solve();


CurrentLinks_sol = [sol.value(L1) sol.value(L2) sol.value(L3) sol.value(L4) sol.value(L5) sol.value(L6)...
    sol.value(L7) sol.value(L8) sol.value(L9) sol.value(L10) sol.value(L11) sol.value(L12) sol.value(L13)...
    sol.value(L14) sol.value(L15)]';


PointM_sol = sol.value(PointM_C);
PointN_sol = sol.value(PointN_C);
[Theta_sol, JointCoord_sol] = InverseKinematicsGripper2D(L_act, sol.value(L0), CurrentLinks_sol, PointM_sol, PointN_sol);


DrawingGripper(JointCoord_sol,[10,0])

fprintf('Reaction forces M: %d \n',sol.value(F_M));
fprintf('Reaction forces N: %d \n',sol.value(F_N));

%% object plot test
% x=5:15;
% y=1*(x-5);
% hold on 
% plot(x,y)
% plot(x,-y)
% hold off
