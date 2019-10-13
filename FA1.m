% Firefly Algorithm
% written by Mojtaba Eslami

clear all

alpha = .2;
gamma = 1;
beta = 1;

MaxGeneration = 20;
m = 40; % number of fireflies
X = 4*rand(2,m);

x = [0:0.01:4];
y = [0:0.01:4];
for i=1:length(x)
    for j=1:length(y)
        f(i,j) = -sin(y(j))*sin(y(j)^2/pi)^20-sin(x(i))*sin(2*x(i)^2/pi)^20;
    end
end

figure(1);clf;hold on
contour(x,y,f,20)
plot(X(1,:),X(2,:),'k.')
xlabel('x_1')
ylabel('x_2')
colormap(gray)
axis([0 4 0 4])

I_best = -1;

for k=1:length(X)
    I(k) = 1/(-sin(X(1,k))*(sin(X(1,k).^2/pi)).^20-sin(X(2,k))*(sin(2*X(2,k).^2/pi)).^20+2);
end

t = 1;
while t<MaxGeneration
    for i=1:m
        for j=1:m
            if I(j)>I(i)
                X(:,i) = X(:,i)+beta*exp(-gamma*norm(X(:,i)-X(:,j))^2)*(X(:,j)-X(:,i))+alpha*(rand(2,1)-.5);
                I(i) = 1/(-sin(X(1,i))*(sin(X(1,i).^2/pi)).^20-sin(X(2,i))*(sin(2*X(2,i).^2/pi)).^20+2);
            end
        end
    end
    
    index = find(I==max(I),1);
    
    
    if max(I)>I_best
        X_best = X(:,index);
        I_best = max(I);
        f_best(t) = -2+1/I_best;
    else
        f_best(t) = min(f_best);
    end
    
    X(:,index) = X(:,index)+2*(rand(2,1)-.5); % random movement of the best firefly
    
    t = t+1;
end

figure(2);clf;hold on
contour(x,y,f,20)
plot(X(1,:),X(2,:),'k.')
xlabel('x_1')
ylabel('x_2')
colormap(gray)
axis([0 4 0 4])

figure(3)
plot(f_best,'k.')
xlabel('iteration number')
ylabel('min f(x_1,x_2)')

X_best

