
% Testing Inverse Kinematics Function


% P = [JointCheck(1,4) + L16*cos(theta16), JointCheck(2,4) + L16*sin(theta16)];

DrawingGripper(JointCoord_sol,[15,0])


norm(JointCoord_sol(:,4) - JointCoord_sol(:,5))

L1 = CurrentLinks_sol(1);     L2 = CurrentLinks_sol(2);     L3 = CurrentLinks_sol(3);     
L4 = CurrentLinks_sol(4);     L5 = CurrentLinks_sol(5);     L6 = CurrentLinks_sol(6);     
L7 = CurrentLinks_sol(7);     L8 = CurrentLinks_sol(8);     L9 = CurrentLinks_sol(9);    
L10 = CurrentLinks_sol(10);   L11 = CurrentLinks_sol(11);  L12 = CurrentLinks_sol(12);
L13 = CurrentLinks_sol(13);   L14 = CurrentLinks_sol(14);  L15 = CurrentLinks_sol(15);

theta3 = Theta_sol(1);  theta4 = Theta_sol(2);  theta5 = Theta_sol(3);
theta6 = Theta_sol(4);  theta8 = Theta_sol(5);  theta9 = Theta_sol(6);   
theta10 = Theta_sol(7); theta11 = Theta_sol(8); theta12 = Theta_sol(9);
theta13 = Theta_sol(10); theta14 = Theta_sol(11); theta15 = Theta_sol(12);


% Point M
P_M = [(L_act + L11*cos(theta11) + (L12+ L14)*cos(theta12));...
       (L0/2 + L11*sin(theta11) + (L12+ L14)*sin(theta12))];

% Point N   
P_N = [(L_act + L8*cos(theta8) + (L9+ L15)*cos(theta9));...
       (-L0/2 + L8*sin(theta8) + (L9+ L15)*sin(theta9))];
% Point D   
P_D1 = [(L1 + L3*cos(theta3) + L4*cos(theta4));...
       (L2 + L3*sin(theta3) + L4*sin(theta4))];
   
   
% Point D   
P_D2 = [(L1 + L6*cos(theta6) + L5*cos(theta5));...
       (-L2 + L6*sin(theta6) + L5*sin(theta5))];