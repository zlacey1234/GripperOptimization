x = 1.5:0.1:25;
%y = -1./exp(x-2)+2 - exp((x-5)./10)

%y = -1./exp(x-2)+1 - (x-10)./10;

%y = -1./exp(x-2) + 1; - (x-10)./10;
%y=-.1./(x-2)+ 1 - ((x-5)./10).^2;

agg = 1/20;
y1=-agg./(x-2)+ 1;
y2=-((x-5).*agg).^2;
y=y1+y2;
plot(x,y)
hold on
plot(x,y1)
plot(x,y2)
hold off

[a,b]=max(y)
x(44)

syms a sf b z
-a/(z-sf) - (a*(z-b))^2 +1
