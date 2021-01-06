A=[80,20];B=[10,25];C=[15,100];D=[80,130];

a1 = (A(2)-B(2))/(A(1)-B(1));
b1 = A(2) - a1*A(1);
a2 = (D(2)-C(2))/(D(1)-C(1));
b2 = D(2) - a2*D(1);

N_line = 5;
N_x = 5;

a_v = linspace(a1,a2,N_line);
b_v = linspace(b1,b2,N_line);
X = linspace(B(1),A(1),N_x);
count = 1;
Y = zeros(1,(N_line)*(N_x));
X_plot = [];
for k1=1:N_line
    for k2=1:N_x
        Y(count) = a_v(k1)*X(k2) + b_v(k1);
        count = count + 1;
    end
    X_plot = [X_plot X];
end

plot(X_plot,Y,'.')
hold on
plot(X,Y(1:5));
plot(X,Y(end-4:end));
hold off