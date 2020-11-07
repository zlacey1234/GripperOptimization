function [Theta, JointCoord] = GripperKinematic(L_act, L0, CurrentLinks, PointD)
%% Function: GripperKinematics
% Summary: The GripperKinematics function takes the inverse kinemetics of
%          the closed chain gripper stucture (given the link lengths,
%          kinematic constraints, and the loaction of Point D, calculate
%          the Theta angles of each of the links). Following this, the
%          function then uses the forward kinematics to solve for the
%          position coordinates of each individual joints given the
%          calculated thetas.
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
%    PointD: this represents the given x and y coordinates of Point D.
%          PointD = [x_D,   : x coordinate of Point D
%                    y_D]   : y coordinate of Point D
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
L1 = CurrentLinks(1);     L2 = CurrentLinks(2);     L3 = CurrentLinks(3);     L4 = CurrentLinks(4);
L5 = CurrentLinks(5);     L6 = CurrentLinks(6);     L7 = CurrentLinks(7);     L8 = CurrentLinks(8);
L9 = CurrentLinks(9);    L10 = CurrentLinks(10);   L11 = CurrentLinks(11);   L12 = CurrentLinks(12);
L13 = CurrentLinks(13);  L14 = CurrentLinks(14);   L15 = CurrentLinks(15);

x_D = PointD(1);   y_D = PointD(2);

%% Calculating theta3, theta4, given x_D and y_D
r_BD = sqrt((x_D - L1)^2 + (y_D - L2)^2);
theta3 = acos((r_BD^2 + L3^2 - L4^2)/(2*r_BD*L3))  +  atan2(y_D - L2, x_D - L1);
theta4 = theta3 + acos((L3^2 + L4^2 - r_BD^2)/(2*L3*L4)) - pi;

%% Calculating theta6, theta5, given x_D and y_D
r_FD = sqrt((x_D - L1)^2 + (y_D + L7)^2);
theta6 = atan2(y_D + L7, x_D - L1) -  acos((r_FD^2 + L6^2 - L5^2)/(2*r_FD*L6));
theta5 = pi  -  acos((L6^2 + L5^2 - r_FD^2)/(2*L6*L5))  +  theta6;

%% Calculating x_G, y_G, given theta3, and theta4 (Due to closed chain constraint)
x_G = L1 + L3*cos(theta3) + L13*cos(theta4-pi);
y_G = L2 + L3*sin(theta3) + L13*sin(theta4-pi);

%% Calculating theta11, theta12, given x_G and y_G
r_IG = sqrt((x_G - L_act)^2 + (y_G - L0/2)^2);
theta11 = acos((r_IG^2 + L11^2 - L12^2)/(2*r_IG*L11))  +  atan2(y_G - L0/2, x_G - L_act);
theta12 = theta11  +  acos((L11^2 + L12^2 - r_IG^2)/(2*L11*L12))  -  pi;

%% Calculating x_J, y_J, given theta6, and theta5 (Due to closed chain constraint)
x_L = L1 + L6*cos(theta6) + L10*cos(theta5-pi);
y_L = -L7 + L6*sin(theta6) + L10*sin(theta5-pi);

%% Calculating theta8, theta9, given x_J and y_J
r_JK = sqrt((x_L - L_act)^2 + (y_L + L0/2)^2);
theta8 = atan2(y_L + L0/2, x_L - L_act)  -  acos((r_JK^2 + L8^2 - L9^2)/(2*r_JK*L8));
theta9 = pi  -  acos((L8^2 + L9^2 - r_JK^2)/(2*L8*L9))  +  theta8;

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
P_G = [x_G;...
       y_G];

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
P_L = [x_L;...
       y_L];

% Point M
P_M = [(L_act + L11*cos(theta11) + (L12+ L14)*cos(theta12));...
       (L0/2 + L11*sin(theta11) + (L12+ L14)*sin(theta12))];

% Point N   
P_N = [(L_act + L8*cos(theta8) + (L9+ L15)*cos(theta9));...
       (-L0/2 + L8*sin(theta8) + (L9+ L15)*sin(theta9))];
   
%% Packing the Theta and Joint Position Parameters  
Theta = [theta3     theta4     theta5     theta6     theta8     theta9 ...
         theta10    theta11    theta12    theta13    theta14    theta15];

JointCoord = [P_A    P_B    P_C    P_D    P_E    P_F    P_G ...   
              P_H    P_I    P_J    P_K    P_L    P_M    P_N];

end

