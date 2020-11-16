function [Theta, JointCoord] = InverseKinematicsGripper2D(L_act, L0, CurrentLinks, PointM, PointN)
%% Function: InverseKinematicsGripper2D
% Summary: The InverseKinematicsGripper2D function takes the inverse
%          kinematics of the closed chain gripper structure (given the link
%          lengths, kinematic constraints and location of the end effectors
%          [the tip of the toes Points M and N], calculate the Theta angles
%          of each of the links). Note: Theta of each of the links are
%          measured from the positive x-axis. 
% INPUTS:
%    L_act: Determines the x position of Joints (I and J)
%    L0: Determines the y position of Joints (I and J)
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
%    PointM: this represents the given x and y coordinates of Point M. One
%            of the Grasping Toe Tip of the Gripper in the 2D Case
%          PointM = [x_M,   : x coordinate of Point M
%                    y_M]   : y coordinate of Point M
%
%    PointD: this represents the given x and y coordinates of Point N. The
%            other Grasping Toe Tip of the Gripper in the 2D Case
%          PointN = [x_N,   : x coordinate of Point N
%                    y_N]   : y coordinate of Point N
%
%
% OUTPUTS:
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
%    JointCoord: this is a 2x14 matrix that contains the coordinates of the 
%                joints of the gripper. 
% 
%                 |Ax  Bx  Cx  Dx  Ex  Fx  Gx  Hx  Ix  Jx  Kx  Lx  Mx  Nx| 
%    JointCoord = |                                                      |
%                 |Ay  By  Cy  Dy  Ey  Fy  Gy  Hy  Iy  Jy  Ky  Ly  My  Ny| 
% 
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com

%% Unpack Parameters
L1 = CurrentLinks(1);     L2 = CurrentLinks(2);     L3 = CurrentLinks(3);     
L4 = CurrentLinks(4);     L5 = CurrentLinks(5);     L6 = CurrentLinks(6);     
L7 = CurrentLinks(7);     L8 = CurrentLinks(8);     L9 = CurrentLinks(9);    
L10 = CurrentLinks(10);   L11 = CurrentLinks(11);  L12 = CurrentLinks(12);
L13 = CurrentLinks(13);   L14 = CurrentLinks(14);  L15 = CurrentLinks(15);

x_M = PointM(1);   y_M = PointM(2);
x_N = PointN(1);   y_N = PointN(2);

%% Calculating theta11, theta12, given x_M and y_M
r_IM = sqrt((x_M - L_act)^2 + (y_M - L0/2)^2);
theta11 = acos((r_IM^2 + L11^2 - (L12 + L14)^2)/(2*r_IM*L11)) + atan2(y_M - L0/2, x_M - L_act);
theta12 = theta11  +  acos((L11^2 + (L12 + L14)^2 - r_IM^2)/(2*L11*(L12+L14)))  -  pi;

%% Calculating theta8, theta9, given x_N and y_N
r_JN = sqrt((x_N - L_act)^2 + (y_N + L0/2)^2);
theta8 = atan2(y_N + L0/2, x_N - L_act)  -  acos((r_JN^2 + L8^2 - (L9+L15)^2)/(2*r_JN*L8));
theta9 = pi  -  acos((L8^2 + (L9+L15)^2 - r_JN^2)/(2*L8*(L9+L15)))  +  theta8;

%% Calculating x_G, y_G, given theta11, and theta12 (Due to closed chain constraint)
x_G = (L_act + L11*cos(theta11) + (L12)*cos(theta12));
y_G = (L0/2 + L11*sin(theta11) + (L12)*sin(theta12));

%% Calculating theta3, theta4, given x_G and y_G
r_BG = sqrt((x_G - L1)^2 + (y_G - L2)^2);
theta3 = atan2(y_G - L2,x_G - L1) - acos((r_BG^2 + L3^2 - L13^2)/(2*r_BG*L3)); 
theta4= theta3 - acos((L3^2 + L13^2 - r_BG^2)/(2*L3*L13));

%% Calculating x_L, y_L, given theta8, and theta9 (Due to closed chain constraint)
x_L = (L_act + L8*cos(theta8) + (L9)*cos(theta9));
y_L = (-L0/2 + L8*sin(theta8) + (L9)*sin(theta9));

%% Calculating theta5, theta6, given x_L and y_L
r_FL = sqrt((x_L - L1)^2 + (y_L + L7)^2);
theta6 = atan2(y_L + L7,x_L - L1) + acos((r_FL^2 + L6^2 - L10^2)/(2*r_FL*L6));
theta5 = theta6 + acos((L6^2 + L10^2 - r_FL^2)/(2*L6*L10));

%% Assign Equivalent theta angles
theta10 = theta5;
theta13 = theta4;
theta14 = theta12;
theta15 = theta9;

%% Calculating the Joint Positions
% Point A
P_A = [L1;...
       0];

% Point B   
P_B = [L1;...   
       L2];

% Point C    
P_C = [(L1 + L3*cos(theta3));...    
      (L2 + L3*sin(theta3))];

% Point D   
P_D = [(L1 + L3*cos(theta3) + L4*cos(theta4));...
       (L2 + L3*sin(theta3) + L4*sin(theta4))];

% Point E   
P_E = [(L1 + L6*cos(theta6));...
       (-L2 + L6*sin(theta6))];

% Point F   
P_F = [L1;...
      -L7];

% Point G  
P_G = [(L_act + L11*cos(theta11) + (L12)*cos(theta12));...
       (L0/2 + L11*sin(theta11) + (L12)*sin(theta12))];

% Point H   
P_H = [(L_act + L11*cos(theta11));...
       (L0/2 + L11*sin(theta11))];

% Point I   
P_I = [L_act;...
       L0/2];

% Point J
P_J = [L_act;...
      -L0/2];

% Point K  
P_K = [(L_act + L8*cos(theta8));...
       (-L0/2 + L8*sin(theta8))];

% Point L   
P_L = [(L_act + L8*cos(theta8) + (L9)*cos(theta9));...
       (-L0/2 + L8*sin(theta8) + (L9)*sin(theta9))];

% Point M
P_M = [x_M;...
       y_M];

% Point N   
P_N = [x_N;...
       y_N];
   
% Point D   
P_D2 = [(L1 + L6*cos(theta6) + L5*cos(theta5));...
       (-L2 + L6*sin(theta6) + L5*sin(theta5))];
   
%% Packing the Theta and Joint Position Parameters  
Theta = [theta3     theta4     theta5     theta6     theta8     theta9 ...
         theta10    theta11    theta12    theta13    theta14    theta15];

JointCoord = [P_A    P_B    P_C    P_D    P_E    P_F    P_G ...   
              P_H    P_I    P_J    P_K    P_L    P_M    P_N     P_D2];

end