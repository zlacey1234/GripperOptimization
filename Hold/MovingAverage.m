tbA = gptest1 %sortrows(gptest1);
A = tbA.R(:,2);
B = tbA.F;
C = min(1,tbA.C);

An=[];Bn=[];
for k=1:length(A)
    if(~isnan(B(k)))
        An = [An; A(k)];
        Bn = [Bn; B(k)];
    end
end

mr = 5;
Ma = movmean(An,[mr mr]);
Mb = movmean(Bn,[mr mr]);

figure
plot(Ma,Mb,'*')

%tb_gpr=table(Ma,Mb);
tb_gpr=table(An,Bn);
%gprMdl1 = fitrgp(tb_gpr,'Mb','KernelFunction','squaredexponential');
%gprMdl1 = fitrgp(tb_gpr,'Mb','KernelFunction','squaredexponential','Basis','linear');

%kfcn = @(XN,XM,theta) (exp(theta(2))^2)*exp(-(pdist2(XN,XM).^2)/(2*exp(theta(1))^2));
gprMdl1 = fitrgp(tb_gpr,'Bn','KernelFunction','squaredexponential','Basis','linear');


x_p = linspace(1,7,1000);
[ypred1,~,yint1] =  predict(gprMdl1,x_p');

figure
hold on
colors = [0 0 0;  % red for 0s
          1 0 0];
gscatter(A,B,C,colors,'.') % Observed data points

%fplot(@(x) x.*sin(x),[0,10],'--r')  % Function plot of x*sin(x)
plot(x_p([1,end]),ypred1([1,end]),'k')                  % GPR predictions
patch([x_p([1,end])';flipud(x_p([1,end])')],[yint([1,end],1);flipud(yint([1,end],2))],'k','FaceAlpha',0.1); % Prediction intervals
%patch([x_p;flipud(x_p)],[yint1(:,1)';flipud(yint1(:,2))'],'k','FaceAlpha',0.1); % Prediction intervals
hold off
%title('GPR Fit of Noise-Free Observations')
legend({'Holds in the range','Oversized hold', 'Mean','95% prediction intervals'})
xlabel('\eta')
ylabel('Max. Pulling Force [N]')
xlim([1 6.5])
ylim([0,65])
%writetable(tb_gpr,'tb_gpr.xlsx')

%set(gcf,'renderer','Painters')