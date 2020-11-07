function PlottingVaryingLinks(Links, F_M, F_N)
%% Function: PlottingVaryingLinks
% Summary: The PlottingVaryingLinks function is similar to the
%          IsLinkVarying function. It essentially checks and determines 
%          which link/s have varying lengths. Based on this result, the
%          function can alter the plot title and axis labels with ease. 
% INPUTS:
%    Links: this is an cell array that holds the Link Length Parameters.
%           This cell array is a 15x1 cell array. Each element of the cell
%           array either contains a singular Link Length for the Links with
%           constant length (constant variables) or an 1xn matrix which
%           holds the spanning lengths for the varying Link (varying
%           variable). 
%         Links = {L1;             : Length of Link between Points (O and A)
%                  L2;             : Length of Link between Points (A and B)
%                  L3;             : Length of Link between Points (B and C)
%                  L4;             : Length of Link between Points (C and D)
%                  L5;             : Length of Link between Points (D and E)
%                  L6;             : Length of Link between Points (E and F)
%                  L7;             : Length of Link between Points (F and A)
%                  L8;             : Length of Link between Points (J and K)
%                  L9;             : Length of Link between Points (K and L)
%                 L10;             : Length of Link between Points (L and E)
%                 L11;             : Length of Link between Points (I and H)
%                 L12;             : Length of Link between Points (H and G)
%                 L13;             : Length of Link between Points (G and C)
%                 L14;             : Length of Link between Points (G and M)
%                 L15}             : Length of Link between Points (L and N)
%    
%    F_M: This is the reaction force calculated on Toe M (Point M). It is a
%         2xn matrix. This matrix represents the change in the reaction
%         force as the link length changes. 
%          F_M = [F_Mx;      : x-component of the reaction force on Toe M
%                 F_My]      : y-component of the reaction force on Toe M
% 
%    F_N: This is the reaction force calculated on Toe N (Point N). It is a
%         2xn matrix. This matrix represents the change in the reaction
%         force as the link length changes. 
%          F_N = [F_Nx;      : x-component of the reaction force on Toe N
%                 F_Ny]      : y-component of the reaction force on Toe N
% 
%% Unpack Parameters 
% Converting the elements in the cell array to matrices. 
 L1 = cell2mat(Links(1));     L2 = cell2mat(Links(2));     L3 = cell2mat(Links(3));     
 L4 = cell2mat(Links(4));     L5 = cell2mat(Links(5));     L6 = cell2mat(Links(6));     
 L7 = cell2mat(Links(7));     L8 = cell2mat(Links(8));     L9 = cell2mat(Links(9));    
L10 = cell2mat(Links(10));   L11 = cell2mat(Links(11));   L12 = cell2mat(Links(12));
L13 = cell2mat(Links(13));   L14 = cell2mat(Links(14));   L15 = cell2mat(Links(15));

% Counting the number of elements in each of the Link matrices
ElementsInCurrentLinks = [numel(L1) numel(L2) numel(L3)... 
                          numel(L4) numel(L5) numel(L6)... 
                          numel(L7) numel(L8) numel(L9)... 
                          numel(L10) numel(L11) numel(L12)... 
                          numel(L13) numel(L14) numel(L15)];
         
% Logic Array Statement. If the number of elements in the specified array
% has more than 1 element (if it is not a singular value) then return true
% (1) otherwise, return false (0)
IsCurrentLinksVarying = (ElementsInCurrentLinks > 1);

% Count how many of the fifteen Links are Varying in Length 
switch sum(IsCurrentLinksVarying)
    % Two Links are Varying. Should Be Symmetrical Links
    case 2
        % Check if Link 2 and Link 7 are varying
        if (IsCurrentLinksVarying(2) == 1 && IsCurrentLinksVarying(7) == 1)
            disp('Links 2 and 7')
            VaryingLink = L2;
            VaryingLinkName = 'Link_{2} and Link_{7}';
            
        elseif (IsCurrentLinksVarying(3) == 1 && IsCurrentLinksVarying(6) == 1)
            disp('Links 3 and 6')
            VaryingLink = L3;
            VaryingLinkName = 'Link_{3} and Link_{6}';
            
        elseif (IsCurrentLinksVarying(4) == 1 && IsCurrentLinksVarying(5) == 1)
            disp('Links 4 and 5')
            VaryingLink = L4;
            VaryingLinkName = 'Link_{4} and Link_{5}';
        
        elseif (IsCurrentLinksVarying(8) == 1 && IsCurrentLinksVarying(11) == 1)
            disp('Links 8 and 11')
            VaryingLink = L8;
            VaryingLinkName = 'Link_{8} and Link_{11}';
        
        elseif (IsCurrentLinksVarying(9) == 1 && IsCurrentLinksVarying(12) == 1)
            disp('Links 9 and 12')
            VaryingLink = L9;
            VaryingLinkName = 'Link_{9} and Link_{12}';
            
        elseif (IsCurrentLinksVarying(10) == 1 && IsCurrentLinksVarying(13) == 1)
            disp('Links 10 and 13')
            VaryingLink = L10;
            VaryingLinkName = 'Link_{10} and Link_{13}';
        
        elseif (IsCurrentLinksVarying(14) == 1 && IsCurrentLinksVarying(15) == 1)
            disp('Links 14 and 15')
            VaryingLink = L14;
            VaryingLinkName = 'Link_{14} and Link_{15}';
        
        else
            disp('Varying Links Not a Symmentrical Pair');
        end
        
    case 1
        disp('One varying link')
        
        if (IsCurrentLinksVarying(1) == 1)
            VaryingLink = L1;
            VaryingLinkName = 'Link_{1}';
        end
    otherwise
        disp('Invalid')
end
figure(1)
subplot(2,1,1);
plot(VaryingLink, F_M(1,:))
title(['X Reaction Force on Toe M vs Varying Length of ', VaryingLinkName])
xlabel(['Length of ', VaryingLinkName, ' (cm)']);
ylabel(['F_{Mx} (N)']) 
grid on

subplot(2,1,2);
plot(VaryingLink, F_M(2,:))
title(['Y Reaction Force on Toe M vs Varying Length of ', VaryingLinkName])
xlabel(['Length of ', VaryingLinkName, ' (cm)']);
ylabel(['F_{My} (N)']) 
grid on
end