close all;
clear all;
clc;

import casadi.*


opti = casadi.Opti();


%% Create the Gripper (Initially at Home Position)
L_act = 0;       L1 = opti.variable();
L2 = opti.variable();

% Point D
x_Di = L1(1) + L2(1);
x_D = x_Di;
y_D = 0;

L16 = opti.variable();
theta16 = 0;

L0 = opti.variable();      
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

%contact point opti.variable constraints
% cube object y=+/-10
rock_y=12;
opti.subject_to(PointM(2)==rock_y);
opti.subject_to(PointN(2)==-rock_y);

% init 
opti.set_initial(PointM,[15,rock_y]);
opti.set_initial(PointN,[15,-rock_y]);


%initial condition
initialDistXFromMiddleToe = 0;
xVelocityOfApproach = -1; % (mm/s) negative since it's moving to the left

%% Forward Kinematics

% [Theta, Joints] = GripperKinematic(L_act,L0,CurrentLink, [x_D y_D]);
 
% M_x = Joints(1,13);   M_y = Joints(2,13);
% N_x = Joints(1,14);   N_y = Joints(2,14);
 

%% inverse kinematics 
[Theta, JointCoord] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM, PointN);

 %% Reaction force constraint
 F_actuator_total = 20; % (Newtons)
 Actuator_joint_num = 2; % We are assuming the pin joints are at the same point for now
        
 [F_M, F_N] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta);
 
if(true)
    %constraint gripping force by gear ratio
    min_ratio = 5;
    opti.subject_to((F_M(2)-F_N(2))/F_actuator_total > min_ratio);
else
    %constraint gripping force by newton
    min_grasping = 20; %N newton
    opti.subject_to(min_grasping<F_M(2)<20*min_grasping);
    opti.subject_to(min_grasping<-F_N(2)<20*min_grasping);
    
end

% constraint in X
%opti.subject_to(0<=F_M(1));
%opti.subject_to(0<=F_N(1));
 
 %% objective function
 %opti.minimize();
  
 
 %% link length constraints (all positive and not too long) 
 minL = 6; maxL = 25;
opti.subject_to(minL < L1 < maxL); opti.subject_to(minL < L2 < maxL); opti.subject_to(minL < L3 < maxL);
opti.subject_to(minL < L4 < maxL); opti.subject_to(minL < L11 < maxL); opti.subject_to(minL < L12 < maxL);
opti.subject_to(minL < L13 < maxL); opti.subject_to(minL < L14 < maxL); 

%%
 
%% constraints

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
 opti.minimize(PointM(1)^2+PointN(1)^2);
 
%run solver 
opti.solver('ipopt');
sol = opti.solve();

 
 
        
     