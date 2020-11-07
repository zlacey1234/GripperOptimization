function [F_M, F_N] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLinks, Theta)
%% Function: StaticEquilibrium
% Summary: The StaticEquilibrium function uses the Link Angles, Link Lengths
%          and the Force of the linear actuator to solve the static
%          equilibrium problem for the 2D gripper design (Determining the
%          reaction forces on the Toes (Point M and Point N).
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

theta3 = Theta(1);  theta4 = Theta(2);  theta5 = Theta(3);
theta6 = Theta(4);  theta8 = Theta(5);  theta9 = Theta(6);   
theta10 = Theta(7); theta11 = Theta(8); theta12 = Theta(9);
theta13 = Theta(10); theta14 = Theta(11); theta15 = Theta(12);

%% Calculating the Reaction Forces
% Given the force of the actuator on L11 and L8
F_actuator = F_actuator_total/Actuator_joint_num;

% Calculating the Reaction forces on the Toe Points
Fy_I = F_actuator*tan(theta11);
F_IH = norm([F_actuator Fy_I]);

% Moment about Toe (Point M) is zero
F_Gy = F_IH*(L12 + L14)*sin(theta11 - theta12)/(L14*cos(theta12));

% Sum of Forces in the y direction is zero
F_My = F_Gy -F_IH*sin(theta11);

% Sum of Forces in the x direction is zero
F_Mx = -F_IH*cos(theta11);


Fy_J = F_actuator*tan(theta8);
F_JK = norm([F_actuator Fy_J]);

% Moment about Toe (Point N) is zero
F_Ly = F_JK*(L9 + L15)*sin(theta9 - theta8)/(L15*cos(theta9));

% Sum of Forces in the y direction is zero
F_Ny = -F_JK*sin(theta8) - F_Ly;

% Sum of Forces in the x direction is zero
F_Nx = -F_JK*cos(theta8);

F_M = [F_Mx; F_My];
F_N = [F_Nx; F_Ny];
end