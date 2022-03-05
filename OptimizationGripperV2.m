% clear workspace 
clc
clear all
close all

%% Create the Gripper (Initially at Home Position)
L_act = 0;       L1 = 5:0.1:25;
L2 = 8;

% Point D
x_Di = L1(1) + L2(1);
x_D = x_Di;
y_D = 0;

L16 = 15;
theta16 = 0;

L0 = 4;      
L3 = 8;     L4 = 8;     L11 = 20;   
L12 = 13;    L13 = 10;    L14 = 5;

L5 = L4;    L6 = L3;    L7 = L2;
L10 = L13;  L9 = L12;   L8 = L11;
L15 = L14;

Links = {L1;  L2;  L3;  L4;  L5;  L6;  L7;  L8;  L9;  L10;  L11;  L12;  L13;  L14;  L15};

[CurrentLink, SizeVaryingMat] = IsLinkVarying(Links,1)
%% Create the Rock Object Shape
% Assuming a shape with sides on the positive x quadrants
Rockx = 3*[0,0,10,10];
Rocky = 3*[4,-4,-4,4];
% Rockx = 3*[0,0,2,7,7,2];
% Rocky = 3*[2,-2,-2,-8,8,2];
initialDistXFromMiddleToe = 0;
xVelocityOfApproach = -1; % (mm/s) negative since it's moving to the left

F_M = zeros(2,SizeVaryingMat);

for j = 1:SizeVaryingMat
     figure(j)
    
    % Point P
    Pinit = [x_Di + L16*cos(theta16), y_D + L16*sin(theta16)];
    
    %% Simulating Middle Toe Contact
    
    dt = 0.05; % (seconds)
    t = 0:dt:Pinit(1)/abs(xVelocityOfApproach);
    N1 = numel(t);
    dL16x = 0;
    
    M_x = zeros(N1,1);
    M_y = zeros(N1,1);
    N_x = zeros(N1,1);
    N_y = zeros(N1,1);
    
    for i = 1:N1
        %% Plotting the approach to the Rock Shape
        polyx = xVelocityOfApproach*t(i);
        x_D = x_Di;
 
%% Supressing the MATLAB plotting simulation 
%        pgon = polyshape(Rockx + Pinit(1) + initialDistXFromMiddleToe + polyx, Rocky)
%        plot(pgon,'FaceColor','yellow','FaceAlpha',0.85)
%        hold on

        P = Pinit;
        % If statement: P(1) > Rockx + P(1) + initialDistXFromMiddleToe + polyx
        % If middle toe is making contact with the object, then we update
        % the position of Point D (to simulate initial gripper closing via the
        % translation inward of Point D)
        if (Pinit(1) >  Pinit(1) + initialDistXFromMiddleToe + polyx)
            dL16x = polyx + initialDistXFromMiddleToe;
            x_D = x_Di + dL16x;
            % Point P
            P = [x_D + L16*cos(theta16), y_D + L16*sin(theta16)];
        end
        
        [CurrentLink, SizeVaryingMat] = IsLinkVarying(Links,j);
        
        [Theta, Joints] = GripperKinematic(L_act,L0,CurrentLink, [x_D y_D]);
        M_x(i) = Joints(1,13);   M_y(i) = Joints(2,13);
        N_x(i) = Joints(1,14);   N_y(i) = Joints(2,14);

        DrawingGripper(Joints, P);
        

        % if the gripper toe makes contact with the object
        if inpolygon(M_x(i),M_y(i),Rockx + Pinit(1) + initialDistXFromMiddleToe + polyx, Rocky) || inpolygon(N_x(i),N_y(i),Rockx + Pinit(1) + initialDistXFromMiddleToe + polyx, Rocky)
            break;
        end
        
    end
    
    if inpolygon(M_x(i),M_y(i),Rockx + Pinit(1) + initialDistXFromMiddleToe + polyx, Rocky) && inpolygon(N_x(i),N_y(i),Rockx + Pinit(1) + initialDistXFromMiddleToe + polyx, Rocky)
        disp('Static Equilibrium');
        % Given the force of the actuator on L11 and L8
        F_actuator_total = 20; % (Newtons)
        Actuator_joint_num = 2; % We are assuming the pin joints are at the same point for now
        
        [F_M(:,j), F_N(:,j)] = StaticEquilibrium(F_actuator_total, Actuator_joint_num, CurrentLink, Theta);
        
    end
end

%%
PlottingVaryingLinks(Links, F_M, F_N)