function script1()
  clear all; close all;
  to =0; % start time
  tf =60; % end time
  t = linspace(to,tf,200);
  xo(1) =-20; %inicial condition
  xo(2) =20;
  %%%global fileID
  %%%fileID = fopen('datos.txt', 'w');
  options1 = odeset('RelTol', 0.01);
  [t,x] = ode23(@(t, x)nonlinear(t, x), t, xo, options1) ;
  %%fclose(fileID);
  xlabel('Time ( s )')
  ylabel('Amplitude')
  hold on
  plot(t, x(:, 1))
  plot(t, x(:, 2))
  legend ('x1', 'x2');
  grid
function[xdot] = nonlinear(t, x)
  a = 1;
  k = 1;
  u=0; 
  % model dynamics
  xdot = [x(1)+a*sin(x(1))+x(2); u];
  %ls;
