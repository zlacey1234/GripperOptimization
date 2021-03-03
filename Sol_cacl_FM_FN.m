
for kt = 1:20
    [Theta_temp, JointCoord_temp] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks_sol, PointM_sol(:,kt), PointN_sol(:,kt));   
    [F_M_sol(:,kt), F_N_sol(:,kt)] = StaticEquilibrium_Gripper2(64, 2, CurrentLinks_sol, Theta_temp, JointCoord_temp);
end