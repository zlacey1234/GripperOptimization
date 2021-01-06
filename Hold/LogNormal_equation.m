close all;
clear all;
clc;

%get width and height of the holds
file = readmatrix('HoldSize.xlsx');
W = file(:,1);
%Height vector is the half size of Width
H = file(1:length(W)/2,2);

%calculate log normal distribution pdf
pdWLN = fitdist(W,'Lognormal');
pdHLN = fitdist(H,'Lognormal');



x_H = min(H):1:max(H);
y_H = pdf(pdHLN,x_H);

sigma_H = pdHLN.sigma;
mu_H = pdHLN.mu;
H_lognormal = 1./(x_H*sigma_H*sqrt(2*pi)).*exp(-(log(x_H)-mu_H).^2/(2*sigma_H^2));

figure
histogram(H,'Normalization','pdf');
hold on
plot(x_H,H_lognormal);
hold on
plot(x_H,y_H,'.')
legend('data','pdf calculated ','pdf w/ a function');
hold off



x_W = min(W):1:max(W);
y_W = pdf(pdWLN,x_W);

sigma_W = pdWLN.sigma;
mu_W = pdWLN.mu;
W_lognormal = 1./(x_W*sigma_W*sqrt(2*pi)).*exp(-(log(x_W)-mu_W).^2/(2*sigma_W^2));


figure
histogram(W,'Normalization','pdf');
hold on
plot(x_W,W_lognormal);
hold on
plot(x_W,y_W,'.')
legend('data','pdf calculated ','pdf w/ a function');
hold off

cdf_W = 0.5*erfc(-(log(x_W)-mu_W)./(sigma_W*sqrt(2)));

figure
plot(x_W,cdf_W);


%% test
H_temp =[H ; H];
A = [H_temp W];

SIGMA_X = sqrt(cov(A));
MU_X = mean(A);

s11 = sqrt(log(1+SIGMA_X(1,1)^2/MU_X(1)^2));
s12 = log(1+SIGMA_X(1,2)/(MU_X(1)*MU_X(2)));
s22 = sqrt(log(1+SIGMA_X(2,2)^2/MU_X(2)^2));

SIGMA = [s11 s12;
         s12 s22];

%m1 = log(MU_X(1)^2/sqrt(MU_X(1)^2+SIGMA_X(1,1)^2));
%m2 = log(MU_X(2)^2/sqrt(MU_X(2)^2+SIGMA_X(2,2)^2));
m1 = log(MU_X(1)) - (SIGMA(1,1)^2)/2;
m2 = log(MU_X(2)) - (SIGMA(2,2)^2)/2;

MU = [m1 m2];

%MU = mean(log(A));
%SIGMA = cov(log(A));

[X1,X2] = meshgrid(x_H,x_W);
Y = [X1(:) X2(:)]';
 
%LOGNORMAL = mvnpdf((Y'),MU,SIGMA);


rho = (exp(SIGMA(1,2)^2)-1)/sqrt((exp(SIGMA(1,1)^2)-1)*(exp(SIGMA(2,2)^2)-1));
%rho = (exp(SIGMA(1,2))-1)/sqrt((exp(SIGMA(1,1)^2-1))*(exp(SIGMA(2,2)^2-1)));
%rho = SIGMA(1,2)^2/sqrt(SIGMA(1,1)*SIGMA(2,2));
%rho = 0;

q = 1/(1-rho^2)*( ((log(Y(1,:))-MU(1))/SIGMA(1,1)).^2 - 2*rho*  ((log(Y(1,:))-MU(1))/SIGMA(1,1)) .* ((log(Y(2,:))-MU(2))/SIGMA(2,2)) +  ((log(Y(2,:))-MU(2))/SIGMA(2,2)).^2   );

LOGNORMAL = 1./(2*pi*Y(1,:).*Y(2,:)*SIGMA(1,1)*SIGMA(2,2)*sqrt(1-rho^2)) .*  exp(-0.5*q);


LOGNORMAL = reshape(LOGNORMAL,length(x_W),length(x_H));
surf(x_H,x_W,LOGNORMAL)

hold on
%histo = histogram2(A(:,1),A(:,2),'Normalization','pdf')