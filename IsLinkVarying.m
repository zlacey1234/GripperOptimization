function [CurrentLinks, sizeVaryingMat] = IsLinkVarying(Links, element)
%% Function: IsLinkVarying
% Summary: The IsLinkVarying function is primarily used to determine
%          whether the specified Link has a varying length. This function
%          is also used to convert the Link cell array to a matrix with
%          singular Link Length values. If the Link isvarying (i.e., if 
%          the link spans a length with 0:0.1:1 [from 0 cm to 1 cm in 0.1 cm 
%          increments] then we need to return the singlular length of this 
%          link for the current iteration). 
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
%    element: this variable spacifies the current iteration in the length 
%             span to determine the element to use. Example, if L1 =
%             0:0.1:0.5 then L1 = [0  0.1  0.2  0.3  0.4  0.5]. if the 
%             variable element = 3, then the current L1 = 0.2 cm. 
%  
%
% OUTPUTS:
%    CurrentLinks: this is a matrix which holds the current singular link
%                  length values which are ultimately used in the
%                  GripperKinematics function as the input. This matrix is
%                  a 15x1 matrix.
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
%    sizeVaryingMat: This variable returns the numel (number of elements)
%                    in the spanning Link Length matrix (i.e., L1 =
%                    0:0.1:0.5 then L1 = [0  0.1  0.2  0.3  0.4  0.5]. This
%                    means the sizeVaryingMat = 6 since there are six
%                    elements (varying lengths of L1). 
% 
% AUTHOR : ZACHARY LACEY
% AFFILIATION : UNIVERSITY OF CALIFORNIA, LOS ANGELES
% EMAIL : zlacey@g.ucla.edu
%         zlacey1234@gmail.com

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
    % Two Links are Varying. For our case this should only occur if the two 
    % links are a Symmetrical Link pair. 
    case 2
        disp('Two varying links');
        
        % Check if Link 2 and Link 7 are varying
        if (IsCurrentLinksVarying(2) == 1 && IsCurrentLinksVarying(7) == 1)
            disp('Links 2 and 7')
            CurrentLinks = [L1;  L2(element);  L3;  L4;  L5;
                L6;  L7(element);  L8;  L9;  L10;
                L11;  L12;  L13;  L14;  L15];
            sizeVaryingMat = ElementsInCurrentLinks(2);
         
        % Check if Link 3 and Link 6 are varying    
        elseif (IsCurrentLinksVarying(3) == 1 && IsCurrentLinksVarying(6) == 1)
            disp('Links 3 and 6')
            CurrentLinks = [L1;  L2;  L3(element);  L4;  L5;
                L6(element);  L7;  L8;  L9;  L10;
                L11;  L12;  L13;  L14;  L15];
            sizeVaryingMat = ElementsInCurrentLinks(3);
         
        % Check if Link 4 and Link 5 are varying
        elseif (IsCurrentLinksVarying(4) == 1 && IsCurrentLinksVarying(5) == 1)
            disp('Links 4 and 5')
            CurrentLinks = [L1;  L2;  L3;  L4(element);  L5(element);
                L6;  L7;  L8;  L9;  L10;
                L11;  L12;  L13;  L14;  L15];
            sizeVaryingMat = ElementsInCurrentLinks(4);
        
        % Check if Link 8 and Link 11 are varying    
        elseif (IsCurrentLinksVarying(8) == 1 && IsCurrentLinksVarying(11) == 1)
            disp('Links 8 and 11')
            CurrentLinks = [L1;  L2;  L3;  L4;  L5;
                L6;  L7;  L8(element);  L9;  L10;
                L11(element);  L12;  L13;  L14;  L15];
            sizeVaryingMat = ElementsInCurrentLinks(8);
        
        % Check if Link 9 and Link 12 are varying
        elseif (IsCurrentLinksVarying(9) == 1 && IsCurrentLinksVarying(12) == 1)
            disp('Links 9 and 12')
            CurrentLinks = [L1;  L2;  L3;  L4;  L5;
                L6;  L7;  L8;  L9(element);  L10;
                L11;  L12(element);  L13;  L14;  L15];
            sizeVaryingMat = ElementsInCurrentLinks(9);
            
        % Check if Link 10 and Link 13 are varying    
        elseif (IsCurrentLinksVarying(10) == 1 && IsCurrentLinksVarying(13) == 1)
            disp('Links 10 and 13')
            CurrentLinks = [L1;  L2;  L3;  L4;  L5;
                L6;  L7;  L8;  L9;  L10(element);
                L11;  L12;  L13(element);  L14;  L15];
            sizeVaryingMat = ElementsInCurrentLinks(10);
        
        % Check if Link 14 and Link 15 are varying    
        elseif (IsCurrentLinksVarying(14) == 1 && IsCurrentLinksVarying(15) == 1)
            disp('Links 14 and 15')
            CurrentLinks = [L1;  L2;  L3;  L4;  L5;
                L6;  L7;  L8;  L9;  L10;
                L11;  L12;  L13;  L14(element);  L15(element)];
            sizeVaryingMat = ElementsInCurrentLinks(14);
        
        else
            disp('Varying Links Not a Symmentrical Pair');
        end
        
    case 1
        disp('One varying link')
        
        if (IsCurrentLinksVarying(1) == 1)
            CurrentLinks = [L1(element);  L2;  L3;  L4;  L5;
                L6;  L7;  L8;  L9;  L10;
                L11;  L12;  L13;  L14;  L15];
            sizeVaryingMat = ElementsInCurrentLinks(1);
        end
    
    % If none of the Links are varying. This function will still work even
    % if no Links are defined as having varying length (all links are
    % single length values). This mean this function does nothing and the
    % Link lengths just passes throught CurrentLinks = Links(in matrix
    % form)
    case 0 
        CurrentLinks = [L1;  L2;  L3;  L4;  L5;
                L6;  L7;  L8;  L9;  L10;
                L11;  L12;  L13;  L14;  L15];
        sizeVaryingMat = ElementsInCurrentLinks(1);
    otherwise
        disp('Invalid')
end

end
