function [F_M, F_N, Check] = StaticEquilibrium_Gripper2(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta, JointCoord)
%% Function: StaticEquilibrium_Gripper2
% Summary: The StaticEquilibrium function uses the Link Angles, Link Lengths
%          and the Force of the linear actuator to solve the static
%          equilibrium problem for the 2D gripper design (Determining the
%          reaction forces on the Toes (Point M and Point N).
%          
%          This Gripper2 represent the Gripper Design which generates
%          gripping force via (pulling Point D using a connected link from 
%          Point D to the Linear Actuator (Point Q) 
%
%          Primary Differences: 
%             - Joints I and J are now fixed along the y-axis.
%             - L_act is now used to specify x position of Joint Q
%             - Additional L16 which is the new link
%               connected to the linear actuator and Point D.
%
%             - Determine the Joint Coordinate of Point Q and find the
%               theta16. 
% INPUTS:
%    F_actuator_total: this is the total force of the linear actuator
%                      provided in Newtons. 
%
%    Actautor_joint_num: this is the number of joints connected to the
%                        linear actuator. The total for of the linear 
%                        actuator is distributed across the joints. 
%
%    CurrentLinks: this is an matrix that holds the Link Length Parameters. 
%                  This matrix is a 15x1 matrix.
%         CurrentLinks = [L1;     : Length of Link between Points (O and A)
%                         L2;     : Length of Link between Points (A and B)
%                         L3;     : Length of Link between Points (B and C)
%                         L4;     : Length of Link between Points (C and D)
%                         L5;     : Length of Link between Points (D and E)
%                         L6;     : Length of Link between Points (E and F)
%                         L7;     : Length of Link between Points (F and A)
%                         L8;     : Length of Link between Points (J and K)
%                         L9;     : Length of Link between Points (K and L)
%                        L10;     : Length of Link between Points (L and E)
%                        L11;     : Length of Link between Points (I and H)
%                        L12;     : Length of Link between Points (H and G)
%                        L13;     : Length of Link between Points (G and C)
%                        L14;     : Length of Link between Points (G and M)
%                        L15]     : Length of Link between Points (L and N)
%
%    Theta: this is a vector that holds the calulated theta angles
%          Theta = [theta3,        : Angle of Link 3 (BC)
%                   theta4,        : Angle of Link 4 (CD)
%                   theta5,        : Angle of Link 5 (DE)                   
%                   theta6,        : Angle of Link 6 (EF)
%                   theta8,        : Angle of Link 8 (JK)
%                   theta9,        : Angle of Link 9 (KL)                   
%                  theta10,        : Angle of Link 10 (LE)
%                  theta11,        : Angle of Link 11 (IH)
%                  theta12,        : Angle of Link 12 (HG)
%                  theta13,        : Angle of Link 13 (GC)
%                  theta14,        : Angle of Link 14 (GM)
%                  theta15,        : Angle of Link 15 (LN)
%
%                  theta16,        : Angle of Link 16 (QD)
%
%    JointCoord: this is a 2x14 matrix that contains the coordinates of the 
%                joints of the gripper. 
% 
%                 |Ax  Bx  Cx  Dx  Ex  Fx  Gx  Hx  Ix  Jx  Kx  Lx  Mx  Nx| 
%    JointCoord = |                                                      |
%                 |Ay  By  Cy  Dy  Ey  Fy  Gy  Hy  Iy  Jy  Ky  Ly  My  Ny| 
%
%
% OUTPUTS:
%    F_M: This is the reaction force calculated on Toe M (Point M). It is a
%         2x1 matrix. 
%          F_M = [F_Mx;      : x-component of the reaction force on Toe M
%                 F_My]      : y-component of the reaction force on Toe M
% 
%    F_N: This is the reaction force calculated on Toe N (Point N). It is a
%         2x1 matrix. 
%          F_N = [F_Nx;      : x-component of the reaction force on Toe N
%                 F_Ny]      : y-component of the reaction force on Toe N
% 
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com

%% Unpack Parameters
L1 = CurrentLinks(1);     L2 = CurrentLinks(2);     L3 = CurrentLinks(3);     L4 = CurrentLinks(4);
L5 = CurrentLinks(5);     L6 = CurrentLinks(6);     L7 = CurrentLinks(7);     L8 = CurrentLinks(8);
L9 = CurrentLinks(9);    L10 = CurrentLinks(10);   L11 = CurrentLinks(11);   L12 = CurrentLinks(12);
L13 = CurrentLinks(13);  L14 = CurrentLinks(14);   L15 = CurrentLinks(15);

theta3 = Theta(1);  
theta4 = Theta(2);  theta5 = Theta(3);
theta6 = Theta(4); 
theta8 = Theta(5);  theta9 = Theta(6);   
theta10 = Theta(7); theta11 = Theta(8); theta12 = Theta(9);
theta13 = Theta(10); theta14 = Theta(11); theta15 = Theta(12);

theta16 = 0;

%% Unpacking the Joint Coordinates
A_x = JointCoord(1,1);    B_x = JointCoord(1,2);    C_x = JointCoord(1,3);
A_y = JointCoord(2,1);    B_y = JointCoord(2,2);    C_y = JointCoord(2,3);

D_x = JointCoord(1,4);    E_x = JointCoord(1,5);    F_x = JointCoord(1,6);
D_y = JointCoord(2,4);    E_y = JointCoord(2,5);    F_y = JointCoord(2,6);

G_x = JointCoord(1,7);    H_x = JointCoord(1,8);    I_x = JointCoord(1,9);
G_y = JointCoord(2,7);    H_y = JointCoord(2,8);    I_y = JointCoord(2,9);

J_x = JointCoord(1,10);    K_x = JointCoord(1,11);    L_x = JointCoord(1,12);
J_y = JointCoord(2,10);    K_y = JointCoord(2,11);    L_y = JointCoord(2,12);

M_x = JointCoord(1,13);    N_x = JointCoord(1,14);    
M_y = JointCoord(2,13);    N_y = JointCoord(2,14); 

%% Solve for the Tension in Link 16 (QD)  
F_act = F_actuator_total; % Newtons

F_QD = F_act/cos(theta16);

%% Solve for the Tesion in Link 4 (CD) and Link 5 (DE)

% Equilibriem Equations at Point D
% Sum Forces Y: 
% F_CD*sin(theta4 + pi) + F_DE*sin(theta5 - pi) - F_QD*sin(theta16) = 0
% Sum Forces X: 
% F_CD*cos(theta4 + pi) + F_DE*cos(theta5 - pi) - F_QD*cos(theta16) = 0

F_DE = F_QD/(cos(theta5 - pi) - (sin(theta5 - pi)*cos(theta4 + pi))/(sin(theta4 + pi)));
F_CD = -F_DE*sin(theta5 - pi)/(sin(theta4 + pi));


Check1 = [F_CD*sin(theta4 + pi) + F_DE*sin(theta5 - pi) - F_QD*sin(theta16);
          F_CD*cos(theta4 + pi) + F_DE*cos(theta5 - pi) - F_QD*cos(theta16)];

a1 = (-sin(theta11-theta12)*(L14+ L12));
b1 = (cos(theta3)*(M_y - C_y) - sin(theta3)*(M_x - C_x));
c1 = (-F_CD*sin(pi+ theta4-theta12)*L14);
d1 = cos(theta11);
e1 = cos(theta3);   
f1 = (-F_CD*cos(theta4));

F_BC = (c1 - (f1/d1)*a1)/(b1 - (e1/d1)*a1);
F_IH = (f1 - F_BC*e1)/d1;

F_My = -F_CD*sin(theta4) - F_IH*sin(theta11) - F_BC*sin(theta3);
    
F_M = [0; F_My];

Check2 = [F_IH*(-sin(theta11-theta12)*(L14+ L12)) + F_BC*cos(theta3)*(M_y - C_y) - F_BC*sin(theta3)*(M_x - C_x) + 0  - (-F_CD*sin(pi+ theta4-theta12)*L14); 
          F_IH*cos(theta11) + F_BC*cos(theta3) + 0 + (F_CD*cos(theta4));
          F_IH*sin(theta11) + F_BC*sin(theta3) + F_My +  F_CD*sin(theta4)];

a2 = sin(theta9 - theta8)*(L9 + L15);
b2 = -(sin(theta6)*(N_x - E_x) - cos(theta6)*(N_y - E_y));
c2 = F_DE*L15*sin(theta5 - theta9);
d2 = cos(theta8);    
e2 = cos(theta6);    
f2 = -F_DE*cos(theta5);

F_EF = (c2 - (f2/d2)*a2)/(b2 - (e2/d2)*a2);      
F_JK = (f2 - F_EF*e2)/d2;

F_Ny = -F_JK*sin(theta8) -F_EF*sin(theta6) - F_DE*sin(theta5);

F_N = [0; F_Ny];
Check = [Check1];  


