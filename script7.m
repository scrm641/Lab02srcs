function script7()
  clear all
  close all
to =0; % start time
tf =30; % end time
t=linspace (to,tf,2000) ;
 xo(1)=-20; % initial condition
 xo(2)=30;

 %%global fileID
 %fileID=fopen('datos.txt','w');
 options1=odeset('RelTol',0.01);
 [t,x]= ode23(@(t,x)nonlinear(t,x),t,xo,options1);
% fclose(fileID);
%% Create us vector in order to Plot u(x1,x2);
 xd=sin(t);
 xd1=cos(t);
 xd2=-sin(t);
 landa=1;
 k = 1; 
 f = -(x(:,2).^2).*cos(3.*x(:,1));
 a = 1.5;
 yr2 = xd2 - landa.*(x(:,2)-xd1);
 s=x(:,2)-xd1+landa*(x(:,1)-xd);
 us=yr2 - k.*s + a.*f;
%%
 xlabel('Time (s)')
 ylabel('Amplitude')
 hold on
 plot(t,x(:,1))
 plot(t,x(:,2))
 plot(t,us)
 legend('x1','x2','u');
 axis([0 30 -5 5])
 grid
 end
 function[xdot]=nonlinear(t,x)
 xd=sin(t);
 xd1=cos(t);
 xd2=-sin(t);
 landa=1;
 k = 1;
 c = 1+1*rand(1,1);
 f = -x(2)^2*cos(3*x(1));
 a = 1.5;
 yr2 = xd2 - landa*(x(2)-xd1);
 s=x(2)-xd1+landa*(x(1)-xd);
 u=yr2 - k*s + a*f;
 % model dynamics
 xdot=[x(2);c*(x(2)^2)*cos(3*x(1))+u];
 end