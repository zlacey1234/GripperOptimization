%Rw./Rw,Results_filtered(min_i:end,k)
%clear;
%load('gpr');

%min_i=5;
%X=[];R_file=[];
%for k=1:length(Results_filtered(min_i:end,1))
    
  %  X = [X; [Rw(k) Rw(k)/Rh(k)].*ones(20,2)];
 %   R_file = [R_file; Results_filtered(k,:)'];
%end
%xlswrite('gp_test.xlsx',file)
%gprMdl1 = fitrgp(gptest1,'F','KernelFunction','squaredexponential','OptimizeHyperparameters','auto');
gprMdl1 = fitrgp(gptest1,'F','KernelFunction','squaredexponential','Standardize',1)%,'BasisFunction','constant');
gprMdl1 = fitrgp(gptest1,'F','Standardize',1,'Holdout',0.25)
%kfoldLoss(gprMdl1)
ypred = kfoldPredict(gprMdl1);
figure();
plot(ypred(gprMdl1.Partition.test));


hold on
y = table2array(gptest1(:,2));
plot(y(gprMdl1.Partition.test),'r.');

figure
x_p = linspace(0,10);
X_p = [50*ones(size(x_p));x_p]';
[ypred1,~,yint1] = predict(gprMdl1,x_p');


figure
hold on
scatter(gptest1.R,gptest1.F,'r') % Observed data points
%fplot(@(x) x.*sin(x),[0,10],'--r')  % Function plot of x*sin(x)
plot(x_p,ypred1,'g')                  % GPR predictions
patch([x_p;flipud(x_p)],[yint1(:,1)';flipud(yint1(:,2))'],'k','FaceAlpha',0.1); % Prediction intervals
hold off
title('GPR Fit of Noise-Free Observations')
legend({'Noise-free observations','g(x) = x*sin(x)','GPR predictions','95% prediction intervals'},'Location','best')