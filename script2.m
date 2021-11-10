function script2()
    clear;
    close all;
    % Considerando a y k iguales a 1, y la ley de control backstepping:
    % u = (-2-a*cos(x(1)))*(x(1)+a*sin(x(1))+x(2))-x(1)-k*(x(2)+2*x(1)+a*sin(x(1)))
    % Condiciones iniciales
    xo(1) = -20;
    xo(2) = 20;
    % Resolviendo edo con ode23
    options1=odeset('RelTol',0.01); % Ajuste de toleracia relativa
    % Tiempo
    to = 0;  % start time
    tf = 10;  % end time 
    t = linspace(to,tf,200);
    % Solucion
    [t,x]= ode23(@(t,x)nonlinear(t,x),t,xo,options1);
    function[xdot] = nonlinear(t,x)
        a = 1; % Parametros
        % -1 + (2)∗sin(t); 
        k = 1;
        % Ley de control Backstepping
        u=(-2-a*cos(x(1)))*(x(1)+a*sin(x(1))+x(2))-x(1)-k*(x(2)+2*x(1)+a*sin(x(1)));
        % Modelo No lineal
        xdot=[x(1)+a*sin(x(1))+x(2);  u];
    end
    % Ploteando la respuesta del sistema controlado
    plot(t,x(:,1),t,x(:,2));
    xlabel('Tiempo');
    ylabel('Amplitud');
    legend('X_1','X_2');
    grid;
end