function [CurrentLinks_sol,width_sol,CDF_sol,PointM_sol,PointN_sol,eval_sol] = WAC_func(L0,offset,min_ratio,width_sample,pdf_sample,guess_CurrentLink,guess_PointM_x,guess_PointN_x,limit_MN,act_range)

import casadi.*
opti = casadi.Opti();


%% Create the Gripper (Initially at Home Position)
L_act = 0;
L1 = opti.variable();   L2 = opti.variable();   L3 = opti.variable();
L4 = opti.variable();   L11 = opti.variable();  L12 = opti.variable();
L13 = opti.variable();  L14 = opti.variable();
%L0 = opti.variable();

L5 = L4;    L6 = L3;    L7 = L2;
L10 = L13;  L9 = L12;   L8 = L11;
L15 = L14;

CurrentLinks = [L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 L11 L12 L13 L14 L15]';

%% width interporation
width_min = opti.variable(); width_max = opti.variable();
width = linspace(width_min,width_max,width_sample);
opti.subject_to(0 < width_min < width_max);

opti.set_initial(width_min,11); opti.set_initial(width_max,60);

PointM_x_init = linspace(guess_PointM_x(1),guess_PointM_x(2),width_sample);
PointN_x_init = linspace(guess_PointN_x(1),guess_PointN_x(2),width_sample);

%% CDF sampling
pdf_sample_x = pdf_sample(1);
pdf_sample_y = pdf_sample(2);
height_max = 100;

%% Actuator setting
min_sf = 1.5; peak = 5; alpha = 1/20;

F_actuator_total = 1; % (Newtons)
Actuator_joint_num = 2; % We are assuming the pin joints are at the same point for now


%% Define contact points
PointM = [opti.variable(1,width_sample); width'/2 + offset];
PointN = [opti.variable(1,width_sample);-width'/2 + offset];
%PointM & N init
opti.set_initial(PointM(1,:), PointM_x_init);
opti.set_initial(PointN(1,:), PointN_x_init);

opti.subject_to(0<PointM(1,:) < limit_MN);
opti.subject_to(0<PointN(1,:) < limit_MN);


%% For loop variables
w = opti.variable(1,width_sample);
rho_sum = opti.variable(1,width_sample);
PointD = opti.variable(2,width_sample);
%th13 = opti.variable(1,width_sample);

%% calculate rho and weight
for k = 1 : width_sample
    
    %solve inverse kinematics
    [Theta_temp, JointCoord_temp] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM(:,k), PointN(:,k));
    %theta angle constraints (avoid singularity)
    opti.subject_to(-pi/4 < Theta_temp(1) < pi/4); opti.subject_to(pi/2 < Theta_temp(3));
%    opti.subject_to(pi/8 < Theta_temp(8) < 3*pi/4);
    %opti.subject_to(-pi/2 < Theta_temp(6) < pi/2);
  %  opti.subject_to(-pi/8 < Theta_temp(10) < pi/8);
    %loop constraint (D_upper == D_lower)
    opti.subject_to(JointCoord_temp(1:2,15)-0.1 <= JointCoord_temp(1:2,4) <= JointCoord_temp(1:2,15)+0.1);
    PointD(:,k) = JointCoord_temp(:,4);
    %th13(k) = Theta_temp(10); %theta 11
    
    
    %reaction force
    [F_M, F_N] = StaticEquilibrium_Gripper2(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta_temp, JointCoord_temp);
    %weight calculation
    
    w(k) = weightSF((F_M(2)-F_N(2)),min_ratio,min_sf,peak,alpha);
    opti.subject_to(1.5*min_ratio<(F_M(2)-F_N(2)));
    
    %calculate pdf sum thru sampling
    pdf_x = linspace(abs(PointM(1,k)-PointN(1,k))+0.001,height_max,pdf_sample_x);
    if k ~= width_sample
        pdf_y = linspace(width(k),width(k+1),pdf_sample_y);
        
        rho_sum_temp = opti.variable(1,pdf_sample_x*pdf_sample_y);
        count = 1;
        pdf_x_diff = pdf_x(2)-pdf_x(1);
        pdf_y_diff = pdf_y(2)-pdf_y(1);
        
        for k1=1:length(pdf_y)
            for k2=1:length(pdf_x)
                rho_sum_temp(count) = pdf_x_diff * pdf_y_diff * hold_pdf([pdf_x(k2),pdf_y(k1)]);
                count = count + 1;
            end
        end
        rho_sum(k) = sum(rho_sum_temp);
        
    else
        %rho_sum(k) = sum(rho_sum_temp(pdf_sample_x*pdf_sample_y-pdf_sample_y+1:end));
        %rho_sum(k-1) = rho_sum(k-1) - sum(rho_sum_temp(pdf_sample_x*pdf_sample_y-pdf_sample_y+1:end));
    end
    
end

%Point D range constraint
%opti.subject_to(PointD(1,end)-PointD(1,1)<act_range);


%% calculate CDF
CDF=sum(w.*rho_sum);
opti.minimize(1-CDF);

%% initial guess for linksh
%amplification
%opti.set_initial(L0,20);%65.5
opti.set_initial(L1,guess_CurrentLink(1));%65.5
opti.set_initial(L2,guess_CurrentLink(2));
opti.set_initial(L3,guess_CurrentLink(3));
opti.set_initial(L4,guess_CurrentLink(4));
opti.set_initial(L11,guess_CurrentLink(11));
opti.set_initial(L12,guess_CurrentLink(12));
opti.set_initial(L13,guess_CurrentLink(13));
opti.set_initial(L14,guess_CurrentLink(14));



%% link length constraints (all positive and not too long)
minL = 10; maxL = 100;
%opti.subject_to(minL < L0 < maxL);
opti.subject_to(20 < L1 < maxL); opti.subject_to(minL < L2 < maxL); opti.subject_to(minL < L3 < maxL);
opti.subject_to(minL < L4 < maxL); opti.subject_to(minL < L11 < maxL); opti.subject_to(minL < L12 < maxL);
opti.subject_to(minL < L13 < maxL); opti.subject_to(20 < L14 < maxL);

%opti.set_initial(sol_save.value_variables());


eval_sol = true;
try
    p_opts = struct('expand',true);
    s_opts = struct('max_iter',3000);
    opti.solver('ipopt',p_opts,s_opts);
    sol = opti.solve();
catch
    eval_sol = false;
end

if (eval_sol)
    CurrentLinks_sol = [sol.value(L1) sol.value(L2) sol.value(L3) sol.value(L4) sol.value(L5) sol.value(L6)...
        sol.value(L7) sol.value(L8) sol.value(L9) sol.value(L10) sol.value(L11) sol.value(L12) sol.value(L13)...
        sol.value(L14) sol.value(L15)]';
    
    width_sol = sol.value(width);
    CDF_sol = sol.value(CDF);
    
    PointM_sol = sol.value(PointM(:,:));
    PointN_sol = sol.value(PointN(:,:));
    
else
    % debug info
    CurrentLinks_sol = [opti.debug.value(L1) opti.debug.value(L2) opti.debug.value(L3) opti.debug.value(L4) opti.debug.value(L5) opti.debug.value(L6)...
        opti.debug.value(L7) opti.debug.value(L8) opti.debug.value(L9) opti.debug.value(L10) opti.debug.value(L11) opti.debug.value(L12) opti.debug.value(L13)...
        opti.debug.value(L14) opti.debug.value(L15)]';
    
    width_sol = opti.debug.value(width);
    CDF_sol = opti.debug.value(CDF);
    
    PointM_sol = opti.debug.value(PointM(:,:));
    PointN_sol = opti.debug.value(PointN(:,:));
    
end



end