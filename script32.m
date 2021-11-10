function script32()
clc,clear
R=dlmread('datos3.txt'); # Choose the correct file to plot results (datos3.txt or datos4.txt generated for scripts which save information in text files)
figure(1)
hold on
plot(R(:,1),R(:,2))
plot(R(:,1),R(:,3))
plot(R(:,1),R(:,4))
axis([0 10])
grid
xlabel('Tiempo (s)')
ylabel('Amplitud')
legend('u','x1','x2')
title('u x1 x2')
   
