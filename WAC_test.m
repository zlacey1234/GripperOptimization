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

%% width interporation
width_sample = 10; offset = 15;
width_min = opti.variable(); width_max = opti.variable();
width = linspace(width_min,width_max,width_sample);
opti.subject_to(width_min < width_max);
opti.set_initial(width_min,20); opti.set_initial(width_max,100);

PointM_x_init = linspace(203,241,width_sample);
PointN_x_init = linspace(238,209,width_sample);

%% CDF sampling 
pdf_sample = 100;
height_max = 50;
%%
%Theta = opti.variable(12,width_sample);
%JointCoord = opti.variable(2*15,width_sample);
w = opti.variable(1,width_sample);
rho_sum = opti.variable(1,width_sample);
%%

 F_actuator_total = 1; % (Newtons)
 Actuator_joint_num = 2; % We are assuming the pin joints are at the same point for now
        

 %Define contact points
    PointM = [opti.variable(1,width_sample); width'/2 + offset];
    PointN = [opti.variable(1,width_sample);-width'/2 + offset];
    %PointM & N init
    opti.set_initial(PointM(1,:), PointM_x_init);
    opti.set_initial(PointN(1,:), PointN_x_init);
 

%% calculate rho and weight
for k = 1 : width_sample
    
    %solve inverse kinematics
    [Theta_temp, JointCoord_temp] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM(:,k), PointN(:,k));
    %theta angle constraints (avoid singularity)
    opti.subject_to(-pi/4 < Theta_temp(1) < pi/4); opti.subject_to(pi/2 < Theta_temp(3)); opti.subject_to(pi/8 < Theta_temp(8) < pi/2);
    %loop constraint (D_upper == D_lower)
    opti.subject_to(JointCoord_temp(1:2,15)-0.1 <= JointCoord_temp(1:2,4) <= JointCoord_temp(1:2,15)+0.1);
    %reaction force
    [F_M, F_N] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta_temp);
    %weight calculation
    min_ratio = 5; min_sf =2; peak = 5; alpha = 1/10;
    w(k) = weightSF((F_M(2)-F_N(2)),min_ratio,min_sf,peak,alpha);
    
    %calculate pdf sum thru sampling
    pdf_x = linspace(abs(PointM(1,k)-PointN(1,k))+0.001,height_max,pdf_sample);
    pdf_y = width(k);
    rho_sum_temp = opti.variable(1,pdf_sample);
    for k2=1:pdf_sample
        rho_sum_temp(k2) = hold_pdf([pdf_x(k2),pdf_y]);
    end
    rho_sum(k) = sum(rho_sum_temp);
end


%% calculate CDF
CDF=sum(w.*rho_sum);
opti.minimize(1-CDF)

%% initial guess for linksh
%amplification
amp = 0.9;
opti.set_initial(L1,amp*65.5);
opti.set_initial(L2,amp*80);
opti.set_initial(L3,amp*80);
opti.set_initial(L4,amp*80);
opti.set_initial(L11,amp*200);
opti.set_initial(L12,amp*130);
opti.set_initial(L13,amp*100);
opti.set_initial(L14,amp*50);

%% link length constraints (all positive and not too long) 
minL = 0; maxL = 200;
opti.subject_to(minL < L1 < maxL); opti.subject_to(minL < L2 < maxL); opti.subject_to(minL < L3 < maxL);
opti.subject_to(minL < L4 < maxL); opti.subject_to(minL < L11 < maxL); opti.subject_to(minL < L12 < maxL);
opti.subject_to(minL < L13 < maxL); opti.subject_to(minL < L14 < maxL); 


opti.solver('ipopt');
sol = opti.solve();

CurrentLinks_sol = [sol.value(L1) sol.value(L2) sol.value(L3) sol.value(L4) sol.value(L5) sol.value(L6)...
    sol.value(L7) sol.value(L8) sol.value(L9) sol.value(L10) sol.value(L11) sol.value(L12) sol.value(L13)...
    sol.value(L14) sol.value(L15)]';

for k=1:width_sample
    figure
    PointM_sol = sol.value(PointM(:,k));
    PointN_sol = sol.value(PointN(:,k));
    [Theta_sol, JointCoord_sol] = InverseKinematicsGripper2D(L_act, sol.value(L0), CurrentLinks_sol, PointM_sol, PointN_sol);
    
    DrawingGripper(JointCoord_sol,[10,0])
end

sol.value(width_min)

sol.value(width_max)
