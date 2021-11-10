function script6()
  clc 
  clear all
  close all
to =0; % start time
tf =30; % end time
t=linspace (to,tf,200) ;
 xo(1)=-20; % initial condition
 xo(2)=30;
 %%global fileID
 %fileID=fopen('datos.txt','w');
 options1=odeset('RelTol',0.01);
 [t,x]= ode23(@(t,x)nonlinear(t,x),t,xo,options1);
% fclose(fileID);
 eta=1;
 xd=sin(t);
 xd1=cos(t);
 xd2=-sin(t);
 landa=1;
 F=(0.5).*(x(:,2)).^2.*cos(3.*x(:,1));
 cn = 1.5; % nominal - valor medio
 s=x(:,2)-xd1+landa.*(x(:,1)-xd);
 us =-(cn.*x(:,2).^2.*cos(3.*x(:,1))) +xd2-landa.*(x(:,2)-xd1)-(F+eta).*sign(s);
  xlabel('Time (s)')
 ylabel('Amplitude')
 hold on
 plot(t,x(:,1))
 plot(t,x(:,2))
 %% Mas la variable de control
 plot(t,us)
 legend('x1','x2', 'u');
 axis([0 30 -5 5])
 grid
 end
 function[xdot]=nonlinear(t,x)
 c = 1+1*rand(1,1); % Robustez
 eta= 1;
 xd=sin(t);
 xd1=cos(t);
 xd2=-sin(t);
 landa=1;
 F=0.5*x(2)^2*cos(3*x(1));
 cn = 1.5; % nominal valor medio
 s=x(2)-xd1+landa*(x(1)-xd);
 u=-(cn*(x(2)^2)*cos(3*x(1)))+xd2-landa*(x(2)-xd1)-(F+eta)*sign(s);
 % model dynamics
 xdot=[x(2);c*(x(2)^2)*cos(3*x(1))+u];
 end