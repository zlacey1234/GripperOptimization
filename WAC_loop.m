close all;
clear all;
clc;

N_guess = 2;
N_amp =10;
L0 = 10; offset=15; min_ratio=5; width_sample=5; pdf_sample=[20,10];

guess_CurrentLink = [65.5,80,80,80,0,0,0,0,0,0,140,120,100,50]';
guess_CurrentLink = [guess_CurrentLink, [42,42,42,42,0,0,0,0,0,0,68,56,49,14]'];
guess_PointM_x = [123 133.5; 123 133.5];
guess_PointN_x = [132 113; 132 113];
limit_MN = 70;

amp = linspace(0.05,2,N_amp);
CLs_sol=[];wd_sol=[];C_sol=[];k_sol=[];

for k1 = 1:N_guess
    for k2=1:N_amp
        [CurrentLinks_sol,width_sol,CDF_sol,PointM_sol,PointN_sol,eval_sol] = ...
            WAC_func(L0,offset,min_ratio,width_sample,pdf_sample,amp(k2)*guess_CurrentLink(:,k1),amp(k2)*guess_PointM_x(:,k1),amp(k2)*guess_PointN_x(:,k1),limit_MN);
        
        if(eval_sol)
            CLs_sol = [CLs_sol;CurrentLinks_sol'];
            wd_sol = [wd_sol;width_sol'];
            C_sol = [C_sol;CDF_sol];
            %M_sol = [M_sol;PointM_sol];
            k_sol = [k_sol;[k1 k2]];
            
            L_act = 0;
            for kf=1:width_sample
                figure
                [Theta_sol, JointCoord_sol] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks_sol, PointM_sol(:,kf), PointN_sol(:,kf));
                DrawingGripper(JointCoord_sol,[10,0])
                axis square;
                axis equal;
            end
            disp('One sol')
        end
    end
end

