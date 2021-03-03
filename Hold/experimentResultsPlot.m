clear all;
close all;
clc;

load('AllResults.mat')

min_i = 1;
Rw = Results_WH(min_i:end,1); %/ (PointM_sol(2,1)-PointN_sol(2,1));
%Rw = Results_WH(min_i:end,1) / max(Results_WH(min_i:end,1));
Rh = Results_WH(min_i:end,2); %/ max(Results_WH(min_i:end,2));

Results_filtered = NaN(length(Results_WH(min_i:end,1)),length(Results(1,:)));

mean_F = NaN(length(Results_WH(:,1)),1);
err = NaN(length(Results_WH(:,1)),1);
x = NaN(length(Results_WH(:,1)),1);

for k=1:length(Results_WH(:,1))
    temp = rmoutliers(Results(k,:));
    Results_filtered(k,1:length(temp)) = temp;
    mean_F(k)   = mean(Results_filtered(k,:));
    err(k)      = max(Results_filtered(k,:)) - min(Results_filtered(k,:));
    %x(k,1)        = Rw(k)/Rh(k);
end



figure
for k=1:20
plot(Rw./Rh,Results_filtered(min_i:end,k),'k.')
%plot(x,Results_filtered(:,k),'k.')
hold on
end
axis square
xlabel('Width-Height Ratio');
ylabel('Max. Pulling Force [N]')
grid on
xlim([0,7]);
ylim([0 70]);

%plot(Rw./Rh .* ones(98,20),Results(min_i:end,20),'.')

%figure
%errorbar(Rw./Rh,mean_F(min_i:end),err(min_i:end)/2,'ks')

%hold off


figure
Mw = ((PointM_sol(2,end)-PointN_sol(2,end))-(PointM_sol(2,1)-PointN_sol(2,1)))/2 + (PointM_sol(2,1)-PointN_sol(2,1));

for k=1:20
    Rht = Rh./max(Results_WH(min_i:end,2));
%plot(Rw./Mw./Rht,Results_filtered(min_i:end,k),'k.')
plot(Rw,Results_filtered(min_i:end,k),'k.')
%plot(x,Results_filtered(:,k),'k.')
hold on
end
axis square
xlabel('Width-Height Ratio');
ylabel('Max. Pulling Force [N]')
grid on
%xlim([0,7]);
%ylim([0 70]);

R=[];F=[];C=[];
for k=1:length(Results_filtered(min_i:end,1))
    
  R = [R; [Rw(k) Rw(k)/Rh(k)].*ones(20,2)];
   F = [F; Results_filtered(k,:)'];
  C = [C; floor(Rw(k)/126.5).*ones(20,1)];
end
gptest1=table(R,F,C);
%xlswrite('gptest1.xlsx',file)


set(gcf,'renderer','Painters')