close all;
clear all;
clc;
import casadi.*
opti = casadi.Opti();

offset = 0;

PointM = [opti.variable(1,2); [5+offset, 30+offset]];
PointN = [opti.variable(1,2); [-5+offset, -30+offset]];

%opti.set_initial(PointM(1,:), [120,75]);
%opti.set_initial(PointN(1,:), [120,75]);

guess_CurrentLinks = [];

Lmin = 10; Lmax = 100;
gmin = 40; gmax = 150;

for k=1:100
    rng(1,'twister')
    L_act = 0; L0 = 40;
    L1 = randi([30 Lmax]);   L2 = randi([Lmin Lmax]);   L3 = randi([Lmin Lmax]);
    L4 = randi([Lmin Lmax]);   L11 = randi([Lmin Lmax]);  L12 = randi([Lmin Lmax]);
    L13 = randi([Lmin Lmax]);  L14 = randi([Lmin Lmax]);
    
    
    L5 = L4;    L6 = L3;    L7 = L2;
    L10 = L13;  L9 = L12;   L8 = L11;
    L15 = L14;
    
    CurrentLinks = [L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 L11 L12 L13 L14 L15]';
    
    %% always use the same random guess
    rng(1,'twister')
    for r=1:20
        PointM_x_guess = [randi([gmin gmax]),randi([gmin gmax])];
        opti.set_initial(PointM(1,:), PointM_x_guess);
        opti.set_initial(PointN(1,:), PointM_x_guess);
        
        %% Create the Gripper (Initially at Home Position)    
        
        for k1=1:2
            [Theta_temp, JointCoord_temp] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM(:,k1), PointN(:,k1));
            %theta angle constraints (avoid singularity)
            opti.subject_to(-pi/4 < Theta_temp(1) < pi/4); opti.subject_to(pi/2 < Theta_temp(3));
            opti.subject_to(pi/8 < Theta_temp(8) < 3*pi/4);
            %opti.subject_to(-pi/2 < Theta_temp(6) < pi/2);
            %opti.subject_to(-pi/2 < Theta_temp(10) < pi/2);
            %loop constraint (D_upper == D_lower)
            opti.subject_to(JointCoord_temp(1:2,15)-0.1 <= JointCoord_temp(1:2,4) <= JointCoord_temp(1:2,15)+0.1);
            
        end
        
        try
            p_opts = struct('expand',true);
            s_opts = struct('max_iter',300);
            opti.solver('ipopt',p_opts,s_opts);

            sol = opti.solve();
        catch
            continue;
            
        end
        
        guess_CurrentLinks = [guess_CurrentLinks; CurrentLinks];
        break;
        
    end
    
    
end