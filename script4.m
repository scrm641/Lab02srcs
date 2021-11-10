function script4()
    clear;
    close all;
    xo(1) = -20;
    xo(2) = 20;
    to = 0;  
    tf = 10; 
    t = linspace(to,tf,200);
    % Solucion
    global fileID
    fileID=fopen('datos4.txt','w');
    options1=odeset('RelTol',0.01);
    [t,x]= ode23(@(t,x)nonlinear(t,x),t,xo,options1);
    fclose(fileID);
    function[xdot] = nonlinear(t,x)
            global fileID
            a = 1; % Parametros
            k = 1000;
            % Ley de control Backstepping
            u=(-2-a*cos(x(1)))*(x(1)+a*sin(x(1))+x(2))-x(1)-k*(x(2)+2*x(1)+a*sin(x(1)));
            % Modelo No lineal
            xdot=[x(1)+a*sin(x(1))+x(2);  u];
            % Llenando los archivos en un archivo de texto
            fprintf(fileID,'%10.6f %10.6f %10.6f %10.6f \n',t,u,x(1),x(2));
    
    
