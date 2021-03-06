close all;
clear all;
clc;
iiiii
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

% offset
offset_AD = 0;
offset_BC = 10;

%% Edge A
PointM_A = opti.variable(1,2); PointN_A = opti.variable(1,2);
opti.subject_to(0<PointM_A(1)); opti.subject_to(0<PointN_A(1)); %x positive
% Edge A constraint and init
Ay=opti.variable(); opti.subject_to(0<Ay); opti.set_initial(Ay,5);
%Contact point at edge A
opti.subject_to(PointM_A(2)==Ay+offset_AD); opti.subject_to(PointN_A(2)==-Ay+offset_AD);
%This constraint helped solving
opti.subject_to(PointN_A(2)<PointM_A(2))
% init for point M and N
opti.set_initial(PointM_A,[10,10]); opti.set_initial(PointN_A,[10,-10]);

%% Edge D 
PointM_D = opti.variable(1,2); PointN_D = opti.variable(1,2);
opti.subject_to(0<PointM_D(1)); opti.subject_to(0<PointN_D(1));
% Edge D constraint and init
Dy=opti.variable(); opti.subject_to(0<Dy); opti.set_initial(Dy,90);
%contact point at edge D
opti.subject_to(PointM_D(2)==Dy+offset_AD); opti.subject_to(PointN_D(2)==-Dy+offset_AD);
%This constraint helped solving
opti.subject_to(PointN_D(2)<PointM_D(2))
% init for point M and N
opti.set_initial(PointM_D,[30,70]); opti.set_initial(PointN_D,[30,-70]);

%% Edge B 
PointM_B = opti.variable(1,2); PointN_B = opti.variable(1,2);
opti.subject_to(0<PointM_B(1)); opti.subject_to(0<PointN_B(1));
% Edge B constraint and init
By=opti.variable(); opti.subject_to(0<By); opti.set_initial(By,18);
Bx=(PointM_B(1)-PointN_B(1)); 
%contact point at edge B
opti.subject_to(PointM_B(2)==By+offset_BC); opti.subject_to(PointN_B(2)==-By+offset_BC);
%This constraint may help solving
opti.subject_to(PointN_B(2)<PointM_B(2)); opti.subject_to(PointN_B(1)<=PointM_B(1));
% init for point M and N
%opti.set_initial(PointM_B,[30,20]); opti.set_initial(PointN_B,[28,-20]);
opti.set_initial(PointM_B,[70,40]); opti.set_initial(PointN_B,[65,-40]);

%% Edge C
PointM_C = opti.variable(1,2); PointN_C = opti.variable(1,2);
opti.subject_to(0<PointM_C(1)); opti.subject_to(0<PointN_C(1));
% Edge C constraint and init
Cy=opti.variable(); opti.subject_to(0<Cy); opti.set_initial(Cy,90);
Cx=(PointM_C(1)-PointN_C(1)); 
%contact point at edge B
opti.subject_to(PointM_C(2)==Cy+offset_BC); opti.subject_to(PointN_C(2)==-Cy+offset_BC);
%This constraint may help solving
opti.subject_to(PointN_C(2)<PointM_C(2)); opti.subject_to(PointN_C(1)<=PointM_C(1));
%init for PointM and N
opti.set_initial(PointM_C,[30,70]); opti.set_initial(PointN_C,[25,-70]);


%% Edge relationship
%opti.subject_to(Ay<Dy);

 

%% inverse kinematics for passive adaptationyh
[Theta_A, JointCoord_A] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM_A, PointN_A);
[Theta_B, JointCoord_B] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM_B, PointN_B);
[Theta_C, JointCoord_C] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM_C, PointN_C);
[Theta_D, JointCoord_D] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM_D, PointN_D);

%% inverse kinematics for moving actuator
% Set actuator position to be at A, use the same point D 
%[Theta_act, JointCoord_act] = GripperKinematic(L1, L0, CurrentLinks, JointCoord(1:2,4));

%% Theta constraints
opti.subject_to(-pi/4 < Theta_A(1) < pi/4); opti.subject_to(pi/2 < Theta_A(3)); opti.subject_to(pi/8 < Theta_A(8) < pi/2);
opti.subject_to(-pi/4 < Theta_B(1) < pi/4); opti.subject_to(pi/2 < Theta_B(3)); opti.subject_to(pi/8 < Theta_B(8) < pi/2);
opti.subject_to(-pi/4 < Theta_C(1) < pi/4); opti.subject_to(pi/2 < Theta_C(3)); opti.subject_to(pi/8 < Theta_C(8) < pi/2);
opti.subject_to(-pi/4 < Theta_D(1) < pi/4); opti.subject_to(pi/2 < Theta_D(3)); opti.subject_to(pi/8 < Theta_D(8) < pi/2);




%% loop constraint for D
opti.subject_to(JointCoord_A(1:2,4) == JointCoord_A(1:2,15));
opti.subject_to(JointCoord_B(1:2,4) == JointCoord_B(1:2,15));
opti.subject_to(JointCoord_C(1:2,4) == JointCoord_C(1:2,15));
opti.subject_to(JointCoord_D(1:2,4) == JointCoord_D(1:2,15));


%% Point D boundary condition
%opti.subject_to(L1 < D_x);
%opti.subject_to(-L2 < D_y < L2);



 %% Reaction force constraint
 F_actuator_total = 1; % (Newtons)
 Actuator_joint_num = 2; % We are assuming the pin joints are at the same point for now
        
[F_M_A, F_N_A] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta_A);
[F_M_B, F_N_B] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta_B);
[F_M_C, F_N_C] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta_C);
[F_M_D, F_N_D] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta_D);

 
 %% calculate weight
 min_ratio = 10; min_sf =2; peak = 5; alpha = 1/20;
 % at edge A
 w_A = weightSF((F_M_A(2)-F_N_A(2)),min_ratio,min_sf,peak,alpha);
 opti.subject_to(min_sf*min_ratio < (F_M_A(2)-F_N_A(2)));
 
  % at edge B
 w_B = weightSF((F_M_B(2)-F_N_B(2)),min_ratio,min_sf,peak,alpha);
 opti.subject_to(min_sf*min_ratio < (F_M_B(2)-F_N_B(2)));
 
  % at edge C
 w_C = weightSF((F_M_C(2)-F_N_C(2)),min_ratio,min_sf,peak,alpha);
 opti.subject_to(min_sf*min_ratio < (F_M_C(2)-F_N_C(2)));
 
 %at edge D
 w_D = weightSF((F_M_D(2)-F_N_D(2)),min_ratio,min_sf,peak,alpha);
 opti.subject_to(min_sf*min_ratio < (F_M_D(2)-F_N_D(2)));
 
 
 %% calculate density
 % Review PointM_A is overrided
 max_height =88;
%  % at A
%  PointM_A = JointCoord_A(1:2,13); PointN_A = JointCoord_A(1:2,14);
%  rho_A = hold_pdf([max_height,PointM_A(2)-PointN_A(2)]);
%   % at B
%  PointM_B = JointCoord_B(1:2,13); PointN_B = JointCoord_B(1:2,14);
%  rho_B = hold_pdf([(PointM_B(1)-PointN_B(1)),PointM_B(2)-PointN_B(2)]);
%   % at C
%  PointM_C = JointCoord_C(1:2,13); PointN_C = JointCoord_C(1:2,14);
%  rho_C = hold_pdf([(PointM_C(1)-PointN_C(1)),PointM_C(2)-PointN_C(2)]);
%  % at D
%  PointM_D = JointCoord_D(1:2,13); PointN_D = JointCoord_D(1:2,14);
%  rho_D = hold_pdf([max_height,PointM_D(2)-PointN_D(2)]);

%%
%  % at A
 PointM_A_calc = JointCoord_A(1:2,13); PointN_A_calc = JointCoord_A(1:2,14);
 rho_A = hold_pdf([max_height,PointM_A_calc(2)-PointN_A_calc(2)]);
  % at B
 PointM_B_calc = JointCoord_B(1:2,13); PointN_B_calc = JointCoord_B(1:2,14);
 rho_B = hold_pdf([(PointM_B_calc(1)-PointN_B_calc(1)),PointM_B_calc(2)-PointN_B_calc(2)]);
  % at C
 PointM_C_calc = JointCoord_C(1:2,13); PointN_C_calc = JointCoord_C(1:2,14);
 rho_C = hold_pdf([(PointM_C_calc(1)-PointN_C_calc(1)),PointM_C_calc(2)-PointN_C_calc(2)]);
 % at D
 PointM_D_calc = JointCoord_D(1:2,13); PointN_D_calc = JointCoord_D(1:2,14);
 rho_D = hold_pdf([max_height,PointM_D_calc(2)-PointN_D_calc(2)]);

%%
% % at A
%  rho_A = hold_pdf([max_height,2*Ay]);
%   % at B
%  rho_B = hold_pdf([Bx,2*By]);
%   % at C
%  rho_C = hold_pdf([Cx,2*Cy]);
%  % at D
%  rho_D = hold_pdf([max_height,Dy]);
 

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
opti.minimize(w_A*rho_A + w_B*rho_B + w_C*rho_C + w_D*rho_D);
 
%run solver 
opti.solver('ipopt');
sol = opti.solve();


CurrentLinks_sol = [sol.value(L1) sol.value(L2) sol.value(L3) sol.value(L4) sol.value(L5) sol.value(L6)...
    sol.value(L7) sol.value(L8) sol.value(L9) sol.value(L10) sol.value(L11) sol.value(L12) sol.value(L13)...
    sol.value(L14) sol.value(L15)]';


PointM_sol = sol.value(PointM_A);
PointN_sol = sol.value(PointN_A);
[Theta_sol, JointCoord_sol] = InverseKinematicsGripper2D(L_act, sol.value(L0), CurrentLinks_sol, PointM_sol, PointN_sol);

DrawingGripper(JointCoord_sol,[10,0])

figure
PointM_sol = sol.value(PointM_B);
PointN_sol = sol.value(PointN_B);
[Theta_sol, JointCoord_sol] = InverseKinematicsGripper2D(L_act, sol.value(L0), CurrentLinks_sol, PointM_sol, PointN_sol);

DrawingGripper(JointCoord_sol,[10,0])

figure
PointM_sol = sol.value(PointM_C);
PointN_sol = sol.value(PointN_C);
[Theta_sol, JointCoord_sol] = InverseKinematicsGripper2D(L_act, sol.value(L0), CurrentLinks_sol, PointM_sol, PointN_sol);

DrawingGripper(JointCoord_sol,[10,0])

figure
PointM_sol = sol.value(PointM_D);
PointN_sol = sol.value(PointN_D);
[Theta_sol, JointCoord_sol] = InverseKinematicsGripper2D(L_act, sol.value(L0), CurrentLinks_sol, PointM_sol, PointN_sol);

DrawingGripper(JointCoord_sol,[10,0])


%fprintf('Reaction forces M: %d \n',sol.value(F_M));
%fprintf('Reaction forces N: %d \n',sol.value(F_N));

%% object plot test
% x=5:15;
% y=1*(x-5);
% hold on 
% plot(x,y)
% plot(x,-y)
% hold off
