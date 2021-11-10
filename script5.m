function script5();
clear all; close all;
to =0; % start time
tf =60; % end time
t = linspace(to,tf,600);
xo(1) = -20; %inicial condition
xo(2) = 20;

global fileID
fileID=fopen('datos5.txt','w');
options1=odeset('RelTol',0.01);
[t,x]=ode23(@(t,x) nonlinear(t,x),t,xo,options1);
fclose(fileID);
xlabel('Time(s)')
ylabel('Amplitude')
hold on
plot(t,x(:,1),t,x(:,2))
hold on;
a=-1 + (3)*rand(1,1);
an=1;
k=1;
u = (-2-an.*cos(x(:,1))).*(x(:,1)+an.*sin(x(:,1))+x(:,2))-x(:,1)-k.*(x(:,2)+2.*x(:,1)+an.*sin(x(:,1)));
plot(t,u(:,1))
axis([0 10 -50 50]);
legend('x1','x2','u');
grid

function[xdot]=nonlinear(t,x)
global fileID 
a=-1 + (4)*rand(1,1); % a con incertidumbre
k=1;
an =1; % a nominal
u = (-2-an*cos(x(1)))*(x(1)+an*sin(x(1))+x(2))-x(1)-k*(x(2)+2*x(1)+an*sin(x(1)));
xdot=[x(1)+a*sin(x(1))+x(2); u];
fprintf(fileID,' %10.6f %10.6f %10.6f %10.6f \n',t,u,x(1),x(2));
