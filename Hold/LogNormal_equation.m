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



x_H = 0:1:90;
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



x_W = 10:1:320;
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