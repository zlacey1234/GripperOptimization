function density = hold_pdf(edge)
%% calculate densit at an point given height and width (x,y) in mm


%height = edge(1)*1000; width=edge(2)*1000; %in mm
height = edge(1); width=edge(2);


SIGMA_X = [17.269530503994100,20.532591126057010;20.532591126057010,46.892543003779290];
MU_X = [34.273333333333340,93.847843137254900];

s11 = sqrt(log(1+SIGMA_X(1,1)^2/MU_X(1)^2));
s12 = log(1+SIGMA_X(1,2)/(MU_X(1)*MU_X(2)));
s22 = sqrt(log(1+SIGMA_X(2,2)^2/MU_X(2)^2));

SIGMA = [s11 s12;s12 s22];

%m1 = log(MU_X(1)^2/sqrt(MU_X(1)^2+SIGMA_X(1,1)^2));
%m2 = log(MU_X(2)^2/sqrt(MU_X(2)^2+SIGMA_X(2,2)^2));
m1 = log(MU_X(1)) - (SIGMA(1,1)^2)/2;
m2 = log(MU_X(2)) - (SIGMA(2,2)^2)/2;

MU = [m1 m2];

%LOGNORMAL = mvnpdf((Y'),MU,SIGMA);

rho = (exp(SIGMA(1,2)^2)-1)/sqrt((exp(SIGMA(1,1)^2)-1)*(exp(SIGMA(2,2)^2)-1));
%rho = (exp(SIGMA(1,2))-1)/sqrt((exp(SIGMA(1,1)^2-1))*(exp(SIGMA(2,2)^2-1)));
%rho = SIGMA(1,2)^2/sqrt(SIGMA(1,1)*SIGMA(2,2));
%rho = 0;

q = 1/(1-rho^2)*( ((log(height)-MU(1))/SIGMA(1,1)).^2 - 2*rho*  ((log(height)-MU(1))/SIGMA(1,1)) .* ((log(width)-MU(2))/SIGMA(2,2)) +  ((log(width)-MU(2))/SIGMA(2,2)).^2   );

density = 1./(2*pi*height.*width*SIGMA(1,1)*SIGMA(2,2)*sqrt(1-rho^2)) .*  exp(-0.5*q);

end