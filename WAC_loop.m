close all;
clear all;
clc;

N_guess = 3;
N_amp =10;
L0 = 10; offset=12; min_ratio=0.213; width_sample=20; pdf_sample=[20,10];

guess_CurrentLink = [65.5,80,80,80,0,0,0,0,0,0,140,120,100,50]';
guess_CurrentLink = [guess_CurrentLink, [42,42,42,42,0,0,0,0,0,0,68,56,49,14]'];
guess_CurrentLink = [guess_CurrentLink, [20,40.75,25.5,48.34,0,0,0,0,0,0,50.86,10,20,20]'];

guess_PointM_x = [133.5 133.5; 133.5 133.5; 70 57.13];
guess_PointN_x = [113 113; 113 113;42 61];
limit_MN = 50;
act_range = 30;
amp = linspace(0.05,2,N_amp);
CLs_sol=[];wd_sol=[];C_sol=[];k_sol=[];M_sol=[];N_sol=[];

for k1 = 2:N_guess
    for k2=1:N_amp
        [CurrentLinks_sol,width_sol,CDF_sol,PointM_sol,PointN_sol,eval_sol] = ...
            WAC_func(L0,offset,min_ratio,width_sample,pdf_sample,amp(k2)*guess_CurrentLink(:,k1),amp(k2)*guess_PointM_x(k1,:),amp(k2)*guess_PointN_x(k1,:),limit_MN,act_range);
        
        if(eval_sol)
            CLs_sol = [CLs_sol;CurrentLinks_sol'];
            wd_sol = [wd_sol;width_sol'];
            C_sol = [C_sol;CDF_sol];
            %M_sol = [M_sol;PointM_sol];
            k_sol = [k_sol;[k1 k2]];
            M_sol = [M_sol; PointM_sol];
            N_sol = [N_sol;PointN_sol];
            L_act = 0;
            s_point = PointM_sol-PointN_sol;
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

