function w = weightSF(ratio, min_ratio, min_sf,peak,alpha)
%% Function: weightSF
% Summary: this function calculate weighting parameter
%
% INPUTS:
%   ratio: is the force amplification ratio based on IK
%   min_ratio: is the minimum force amplification required for this gripper
%   min_sf: is minimum safety factor for this design
%   peak: is the peak point for x^2 part of weight
%   alpha: alpha <= 1 is aggressiveness factor that can control how fast the weight goes to 0 after the peak 
%
% OUTPUTS:
%    w: is a weight for objective function. W(SF(IK)). 
%       A function of safety factor, which is a function of inverse
%       kinematics
    sf = ratio./min_ratio;
    w = -alpha./((sf-min_sf)) - (alpha.*(sf-peak)).^2 + 1;
    

end