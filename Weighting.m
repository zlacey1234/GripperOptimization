x = 1.51:0.01:10;
%y = -1./exp(x-2)+2 - exp((x-5)./10)

%y = -1./exp(x-2)+1 - (x-10)./10;

%y = -1./exp(x-2) + 1; - (x-10)./10;
%y=-.1./(x-2)+ 1 - ((x-5)./10).^2;

agg = 1/20;
y1=-agg./(x-1.5)+ 1;
y2=-((x-1).*agg).^2;
y=y1+y2;
plot(x,y, 'k')
hold on
plot([1.5 1.5], [-4 1],'r')
plot([3.5 3.5], [-4 1],'b')
%plot(x,y1)
%plot(x,y2)
hold off
xlabel('Safety Factor')
ylabel('Weight, \omega(SF)')
grid on
xlim([0,10]);
ylim([0,1]);
%axis square
legend('\omega(SF)','\phi','\gamma')
set(gcf,'renderer','Painters')


[a,b]=max(y)
x(b)

syms a sf b z
-a/exp(z-sf) - (a*(z-b))^2 +1
