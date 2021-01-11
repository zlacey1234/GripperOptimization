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

% offset
offset_AD = 0;
offset_BC = 10;

dis = 1;
%% Edge A
PointM_A = opti.variable(1,2); PointN_A = opti.variable(1,2);
opti.subject_to(0<PointM_A(1)); opti.subject_to(0<PointN_A(1)); %x positive
% Edge A constraint and init
Ay=opti.variable(); opti.subject_to(0<Ay); opti.set_initial(Ay,105*dis);
%Contact point at edge A
opti.subject_to(PointM_A(2)==Ay+offset_AD); opti.subject_to(PointN_A(2)==-Ay+offset_AD);
%This constraint helped solving
opti.subject_to(PointN_A(2)<PointM_A(2))
% init for point M and N
opti.set_initial(PointM_A,[75,105]*dis); opti.set_initial(PointN_A,[75,-105]*dis);

%% Edge D 
PointM_D = opti.variable(1,2); PointN_D = opti.variable(1,2);
opti.subject_to(0<PointM_D(1)); opti.subject_to(0<PointN_D(1));
% Edge D constraint and init
Dy=opti.variable(); opti.subject_to(0<Dy); opti.set_initial(Dy,150);
%contact point at edge D
opti.subject_to(PointM_D(2)==Dy+offset_AD); opti.subject_to(PointN_D(2)==-Dy+offset_AD);
%This constraint helped solving
opti.subject_to(PointN_D(2)<PointM_D(2))
% init for point M and N
opti.set_initial(PointM_D,[80,150]); opti.set_initial(PointN_D,[80,-150]);

%% Edge B 
PointM_B = opti.variable(1,2); PointN_B = opti.variable(1,2);
opti.subject_to(0<PointM_B(1)); opti.subject_to(0<PointN_B(1));
% Edge B constraint and init
By=opti.variable(); opti.subject_to(0<By); opti.set_initial(By,138*dis);
Bx=(PointM_B(1)-PointN_B(1)); 
%contact point at edge B
opti.subject_to(PointM_B(2)==By+offset_BC); opti.subject_to(PointN_B(2)==-By+offset_BC);
%This constraint may help solving
opti.subject_to(PointN_B(2)<PointM_B(2)); opti.subject_to(PointN_B(1)<=PointM_B(1));
% init for point M and N
%opti.set_initial(PointM_B,[30,20]); opti.set_initial(PointN_B,[28,-20]);
opti.set_initial(PointM_B,[82,148]*dis); opti.set_initial(PointN_B,[81,-128]*dis);

%% Edge C
PointM_C = opti.variable(1,2); PointN_C = opti.variable(1,2);
opti.subject_to(0<PointM_C(1)); opti.subject_to(0<PointN_C(1));
% Edge C constraint and init
Cy=opti.variable(); opti.subject_to(0<Cy); opti.set_initial(Cy,145);
Cx=(PointM_C(1)-PointN_C(1)); 
%contact point at edge B
opti.subject_to(PointM_C(2)==Cy+offset_BC); opti.subject_to(PointN_C(2)==-Cy+offset_BC);
%This constraint may help solving
opti.subject_to(PointN_C(2)<PointM_C(2)); opti.subject_to(PointN_C(1)<=PointM_C(1));
%init for PointM and N
opti.set_initial(PointM_C,[81,155]); opti.set_initial(PointN_C,[81,-135]);


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
opti.subject_to(JointCoord_A(1:2,15)-0.1 <= JointCoord_A(1:2,4) <= JointCoord_A(1:2,15)+0.1);
opti.subject_to(JointCoord_B(1:2,15)-0.1 <= JointCoord_B(1:2,4) <= JointCoord_B(1:2,15)+0.1);
opti.subject_to(JointCoord_C(1:2,15)-0.1 <= JointCoord_C(1:2,4) <= JointCoord_C(1:2,15)+0.1);
opti.subject_to(JointCoord_D(1:2,15)-0.1 <= JointCoord_D(1:2,4) <= JointCoord_D(1:2,15)+0.1);

%opti.subject_to(JointCoord_A(1:2,4) == JointCoord_A(1:2,15));
%opti.subject_to(JointCoord_B(1:2,4) == JointCoord_B(1:2,15));
%opti.subject_to(JointCoord_C(1:2,4) == JointCoord_C(1:2,15));
%opti.subject_to(JointCoord_D(1:2,4) == JointCoord_D(1:2,15));


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
% opti.subject_to(min_sf*min_ratio < (F_M_A(2)-F_N_A(2)));
 
  % at edge B
 w_B = weightSF((F_M_B(2)-F_N_B(2)),min_ratio,min_sf,peak,alpha);
 %opti.subject_to(min_sf*min_ratio < (F_M_B(2)-F_N_B(2)));
 
  % at edge C
 w_C = weightSF((F_M_C(2)-F_N_C(2)),min_ratio,min_sf,peak,alpha);
 %opti.subject_to(min_sf*min_ratio < (F_M_C(2)-F_N_C(2)));
 
 %at edge D
 w_D = weightSF((F_M_D(2)-F_N_D(2)),min_ratio,min_sf,peak,alpha);
 %opti.subject_to(min_sf*min_ratio < (F_M_D(2)-F_N_D(2)));
 
 
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


%% Monte Carlo Sampling

A_d = JointCoord_A(1:2,13); B_d = JointCoord_B(1:2,13); C_d = JointCoord_C(1:2,13); D_d = JointCoord_D(1:2,13);
a1 = (2*A_d(2)-2*B_d(2))/(A_d(1)-B_d(1));
b1 = 2*A_d(2) - a1*A_d(1);
a2 = (2*D_d(2)-2*C_d(2))/(D_d(1)-C_d(1));
b2 = 2*D_d(2) - a2*D_d(1);

N_line = 5;
N_x = 5;

a_v = linspace(a1,a2,N_line);
b_v = linspace(b1,b2,N_line);
X_h = linspace(B_d(1),A_d(1),N_x);
count = 1;
rho_all = opti.variable(1,(N_line)*(N_x));
%w_all = opti.variable(1,(N_line)*(N_x));
%debug_D = opti.variable(2,(N_line)*(N_x));
for k1=1:N_line
    for k2=1:N_x
        Dy = a_v(k1)*X_h(k2) + b_v(k1);
        Dx = X_h(k2);
        %debug_D(1:2,count) = [Dx Dy];
        
        %[Theta_S, JointCoord_S] = GripperKinematic(L1, L0, CurrentLinks, [Dx,Dy]);
        %PointM_S_calc = JointCoord_S(1:2,13); PointN_S_calc = JointCoord_S(1:2,14);
        rho_all(count) = hold_pdf([Dx,Dy]);
        %rho_all(count) = hold_pdf([(PointM_S_calc(1)-PointN_S_calc(1)),PointM_S_calc(2)-PointN_S_calc(2)]);
        %[F_M_S, F_N_S] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta_S);
        %w_all(count) = weightSF((F_M_S(2)-F_N_S(2)),min_ratio,min_sf,peak,alpha);
        %opti.subject_to(min_sf*min_ratio < (F_M(2)-F_N(2)));
        
        count = count + 1;
    end
end

 

%% link length constraints (all positive and not too long) 
minL = 0; maxL = 200;
opti.subject_to(minL <= L1 < maxL); opti.subject_to(minL < L2 < maxL); opti.subject_to(minL < L3 < maxL);
opti.subject_to(minL < L4 < maxL); opti.subject_to(minL < L11 < maxL); opti.subject_to(minL < L12 < maxL);
opti.subject_to(minL < L13 < maxL); opti.subject_to(minL < L14 < maxL); 

%% Other length constraints
%opti.subject_to((L4 + L13)<maxL);


%% initial guess for links
%amplification
amp = 1;
opti.set_initial(L1,amp*0.2407);
opti.set_initial(L2,amp*131);
opti.set_initial(L3,amp*21);
opti.set_initial(L4,amp*128);
opti.set_initial(L11,amp*137);
opti.set_initial(L12,amp*22);
opti.set_initial(L13,amp*13);
opti.set_initial(L14,amp*59);

%% Metric calculations
%theta14_diff = (Theta(11)-Theta_act(11));
%opti.subject_to(0 < theta14_diff);

%% Objective function
%opti.minimize(w_a/w_a^2);
%opti.minimize(w_A*rho_A + w_B*rho_B + w_C*rho_C + w_D*rho_D);
%CDF=sum(w_all.*rho_all);
CDF=sum(rho_all);
% CDF=0;
% for k=1:length(w_all)
%     CDF = CDF + (w_all(k))*rho_all(k);
% end

%opti.minimize(CDF/CDF^2);
%opti.minimize(1/CDF);
opti.minimize(1-CDF)
 
%run solver 
%opti.set_initial(opti.debug.value_variables());

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
