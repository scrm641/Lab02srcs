#@ SynRM - Sliding Mode Control 
# Q3 Advanced Control Systems
# Group A - 2021 
import math
import numpy as np
import random
from scipy import integrate
import matplotlib.pyplot as plt
# Parameters - Review Bibliography 
Ld = 0.230 # d-axis Inductance
Lq = 0.110 # q-axis Inductance
TL = 14 # Load Torque
#
B = 0.0045 # Viscous damping coefficient 
dB = 0.01 # % Variation of V (%)
Rs = 2.95
dRs = 0.05
Jm = 0.04838 
dJ = 0.05
#
speed = 6.28*10 # Speed desired
speed2 = 6.28*7.5
speed3 = 6.28*12
landa = 5
eta = 1
npp = 2;
cf1 = Rs*(Lq+Ld)/(Lq*Ld)
cF1 = dRs*cf1
a0f = 3*npp*(Ld-Lq)/(2*Jm);
a0F = 3*npp*(Ld-Lq)/(2*Jm) * (1/(1-dJ) -1);
# Differential System Equation
def nonlinear(t,x):
     return np.array(   [  (1/Ld)* (  10  -    Rs*x[0]*(1+dRs*math.sin(6*t)) + x[3]*Lq*x[1]   )  , \
                           (1/Lq)* ( u(t,x)- Rs*x[1]*(1+dRs*math.sin(6*t)) - x[3]*Ld*x[0]   )  , \
                           x[3], \
                            a0f*x[0]*x[1]  - (TL+ B*x[3])/(Jm*(1+dJ*math.sin(6*t)))    ]) 
# Control functions
def u(t,x):
     return  sat( ( -fh(x) + xd2(t) -  landa*(   ( 1/Jm )*(((3*npp*(Ld-Lq)/2)*x[0]*x[1]) - TL -B*x[3] )   - xd1(t) ) - (   ( F(x)+eta )*sign(s(t,x)))  )    *( Lq/ (x[0]*a0f)   )  )
def s(t,x):
    return  ( (a0f*x[0]*x[1]  - TL/Jm - B*x[3]/Jm ) - xd1(t) )   + landa*(x[3]- xd(t)  )
def F(x): 
    return   a0F*(  (B/(Jm**2)+cF1)*abs(x[0]*x[1])   +  abs(x[3])*abs( Lq*(x[1]**2)/Ld - Ld*(x[0]**2)/Lq )      )  + abs(x[3])*( ((B/Jm)*((1+dB)/(1-dJ)-1))**2)  + TL* (B/(Jm**2))*((1-dB)/((1-dJ)**2)-1)  
def fh(x):
    return   a0f*( (-B/(Jm**2)-cf1)*x[0]*x[1]   +  x[3]*( Lq*(x[1]**2)/Ld - Ld*(x[0]**2)/Lq )      )  + x[3]*(B*B/(Jm**2))  + TL*B/(Jm**2) 
def sign(a):
    if a<0:
        return -1
    elif a == 0:
        return 0
    else:
        return 1
def ustep(t):
    if t>0:
        return 1;
    else:
        return 0
def vd(t):
    return 0 
def sat(v):
    if v>200:
      return 200;
    elif v<-200:
      return -200;
    else:
      return v;
#def vq(t):
#    return 0
def xd1(t):
    return 0 #speed*(t)*(ustep(t) - ustep(t-20)) + speed2*(t)*(ustep(t-20) - ustep(t-40)) + speed3*(t)*(ustep(t-40)) 
def xd(t):
    return speed#*    (ustep(t) - ustep(t-20)) + speed2*    (ustep(t-20) - ustep(t-40)) + speed3*(ustep(t-40) )
def xd2(t):
    return 0

t0, t1 = 0, 20                # start and end
t = np.linspace(0, 10, 6000)  # the points of evaluation of solution
y0 = [4,4, 0 ,0]                   # initial value
y = np.zeros((len(t), len(y0)))   # array for solution
y[0, :] = y0
us = np.zeros((len(t), 1));
us[0, 0] = 0;
r = integrate.ode(nonlinear).set_integrator("dopri5")  # choice of method
r.set_initial_value(y0, t0)   # initial values
for i in range(1, t.size):
   y[i, :] = r.integrate(t[i]) # get one more value, add it to the array
   us[i,:] = u(t[i],y[i,:])
  # if not r.successful():
    #   raise RuntimeError("Could not integrate")
plt.plot(t, y[:,0])
plt.plot(t, y[:,1])
plt.plot(t, y[:,1]+y[:,0])
plt.legend(["i_d", "i_q"], loc ="lower right")

plt.title("Corrientes")
#plt.ylim(-50, 105);
plt.grid(True)
plt.xlabel("time")
plt.show()

plt.plot(t, y[:,3]*60/6.28);
plt.title("Motor Speed (RPM)")
#plt.ylim(240, 250);

plt.grid(True)
plt.xlabel("time")
plt.show()

plt.plot(t, us[:,0]);
plt.title("Voltaje")
#plt.ylim(-1000,10000);
plt.grid(True)
plt.xlabel("time")
plt.show()

