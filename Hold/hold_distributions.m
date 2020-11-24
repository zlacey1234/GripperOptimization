
close all;
x = 0:0.1:350;
%sigma  = std(pdW);
%mu = mean(pdW);

%pdy = 1/(sigma*sqrt(2))*exp(-.5*((x-mu)/sigma).^2);


yBR = pdf(pdBR,x);
yLG = pdf(pdLG,x);
yLL = pdf(pdLL,x);
yLN = pdf(pdLN,x);

figure
qqplot(W,pdBR);
figure
qqplot(W,pdLG);
figure
qqplot(W,pdLL);
figure
qqplot(W,pdLN);

%histogram(log(W),'Normalization','pdf');
%bline(x,pdy);
%line(x,y)