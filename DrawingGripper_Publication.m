function DrawingGripper_Publication(JointCoord, Label_Links)
%% Function: DrawingGripper_Publication
% Summary: The DrawingGripper_Publication function takes the Joint 
%          Coordinates to draw the 2D Gripper Model on a 2D MATLAB plot.
%
% INPUTS:
%    JointCoord: this is a 2x14 matrix that contains the coordinates of the 
%                joints of the gripper. 
% 
%                 |Ax  Bx  Cx  Dx  Ex  Fx  Gx  Hx  Ix  Jx  Kx  Lx  Mx  Nx| 
%    JointCoord = |                                                      |
%                 |Ay  By  Cy  Dy  Ey  Fy  Gy  Hy  Iy  Jy  Ky  Ly  My  Ny| 
%
%    P: this represents the given x and y coordinates of Point P. Tip of
%    the Middle Toe of the Gripper in the 2D Case. 
%          P = [x_P,   : x coordinate of Point P
%               y_P]   : y coordinate of Point P
%
%
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com
%

%%
if ~exist('Label_Links', 'var')
    Label_Links = 0
end

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

grid on
axis square
plot([0, A_x],[0, A_y],'-','LineWidth',4,'color',[0 0 0] + 0.5);
hold on
plot([B_x,F_x],[B_y,F_y], '-','LineWidth',4,'color',[0 0 0] + 0.7);
plot([B_x,C_x],[B_y,C_y], '-','LineWidth',4,'color',[0 0 0] + 0.3);
plot([F_x,E_x],[F_y,E_y], '-','LineWidth',4,'color',[0 0 0] + 0.3);
plot([G_x,D_x],[G_y,D_y], '-','LineWidth',4,'color',[0 0 0] + 0.7);
plot([L_x,D_x],[L_y,D_y], '-','LineWidth',4,'color',[0 0 0] + 0.7);
plot([H_x,I_x],[H_y,I_y], '-','LineWidth',4,'color',[0 0 0] + 0.4);
plot([H_x,M_x],[H_y,M_y], '-','LineWidth',4,'color',[0 0 0] + 0.5);
plot([K_x,J_x],[K_y,J_y], '-','LineWidth',4,'color',[0 0 0] + 0.4);
plot([K_x,N_x],[K_y,N_y], '-','LineWidth',4,'color',[0 0 0] + 0.5);

% if Label_Links
%     textoff = 1
%     text((0 + A_x)/2, (0 + A_y)/2 + textoff, '$\bf{L_1}$','FontSize',14,'Interpreter','Latex')
%     text((B_x + A_x)/2 - 2*textoff, (B_y + A_y)/2 , '$\bf{L_2}$','FontSize',14,'Interpreter','Latex')
%     text((B_x + C_x)/2, (B_y + C_y)/2 + textoff, '$\bf{L_{3}}$','FontSize',14,'Interpreter','Latex')
%     text((C_x + D_x)/2 - 2*textoff, (C_y + D_y)/2 , '$\bf{L_{4}}$','FontSize',14,'Interpreter','Latex')
%     
% end

plot(A_x, A_y, 'k.','MarkerSize', 30)
plot(B_x, B_y, 'k.','MarkerSize', 30)
plot(C_x, C_y, 'k.','MarkerSize', 30)
plot(D_x, D_y, 'k.','MarkerSize', 30)
plot(E_x, E_y, 'k.','MarkerSize', 30)
plot(F_x, F_y, 'k.','MarkerSize', 30)
plot(G_x, G_y, 'k.','MarkerSize', 30)
plot(H_x, H_y, 'k.','MarkerSize', 30)
plot(I_x, I_y, 'k.','MarkerSize', 30)
plot(J_x, J_y, 'k.','MarkerSize', 30)
plot(K_x, K_y, 'k.','MarkerSize', 30)
plot(L_x, L_y, 'k.','MarkerSize', 30)
plot(M_x, M_y, 'k.','MarkerSize', 30)
plot(N_x, N_y, 'k.','MarkerSize', 30)

xlabel('Height Direction, x, [mm]','FontSize',14,'Interpreter','Latex')
ylabel('Width Direction, y, [mm]','FontSize',14,'Interpreter','Latex')
xlim([-60 80])
ylim([-60 80])
hold off
%axis square
drawnow;
end