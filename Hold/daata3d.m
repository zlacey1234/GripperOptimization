close all

load('data3d')
Results = Boulderingholdlists1S3(:,4:end);

for k=1:length(Results(:,1))
    Results_filtered(k,:) = rmoutliers(Results(k,:));
   
end



figure
for k=1:20
plot((Boulderingholdlists1S3(:,1)+Boulderingholdlists1S3(:,2))/2,Results_filtered(:,k),'k.')
%plot(x,Results_filtered(:,k),'k.')
hold on
end
axis square
xlabel('Width-Height Ratio');
ylabel('Max. Pulling Force [N]')
grid on
xlim([60 180]);
ylim([0 60]);
hold off

figure

for k=1:length(Results(:,1))
    temp = rmoutliers(Results(k,:));
    Results_filtered(k,1:length(temp)) = temp;
    mean_F(k)   = mean(Results_filtered(k,4:end));
    err(k)      = max(Results_filtered(k,4:end)) - min(Results_filtered(k,4:end));
    %x(k,1)        = Rw(k)/Rh(k);
end

%errorbar((Boulderingholdlists1S3(:,1)+Boulderingholdlists1S3(:,2))./(2*Boulderingholdlists1S3(:,3)),mean_F(1:end),err(1:end)/2,'ks')

%errorbar((Boulderingholdlists1S3(:,1)+Boulderingholdlists1S3(:,2))./(2),mean_F(1:end),err(1:end)/2,'ks')
hold on
errorbar((Boulderingholdlists1S3([2:4,6],1)+ Boulderingholdlists1S3([2:4,6],2))./(2),mean_F([2:4,6]),err([2:4,6])/2,'ks')
errorbar((Boulderingholdlists1S3([1,5],1)+Boulderingholdlists1S3([1,5],2))./(2),mean_F([1,5]),err([1,5])/2,'rs')
axis square
xlabel('Avgrage Width [mm]');
ylabel('Max. Pulling Force [N]')
legend({'Holds in the range','Oversized hold'})
grid on
xlim([60 180]);
ylim([0 60]);
hold off
figure
hold on
errorbar(max((Boulderingholdlists1S3([2:4,6],1))./(Boulderingholdlists1S3([2:4,6],3)),(Boulderingholdlists1S3([2:4,6],2))./(Boulderingholdlists1S3([2:4,6],3)))',mean_F([2:4,6]),err([2:4,6])/2,'ks')
errorbar(max((Boulderingholdlists1S3([1,5],1))./(Boulderingholdlists1S3([1,5],3)),(Boulderingholdlists1S3([1,5],2))./(Boulderingholdlists1S3([1,5],3))),mean_F([1,5]),err([1,5])/2,'rs')
axis square
xlabel('Maximum \eta');
ylabel('Max. Pulling Force [N]')
legend({'Holds in the range','Oversized hold'})
grid on
xlim([1 6.5]);
ylim([0 60]);

set(gcf,'renderer','Painters')
