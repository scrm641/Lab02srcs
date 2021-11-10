#@ SynRM - Sliding Mode Control 
# Q3 Advanced Control Systems
# Group A - 2021 
import math
import numpy as np
import random
from scipy import integrate
import matplotlib.pyplot as plt
# Parameters - Review Bibliography 
Ld = 0.230; # d-axis Inductance
Lq = 0.110; 
TL = 14; # Load Torque
B = 0.0045; # Viscous damping coefficient 
D = 0.5; # % Variation of V (%)
Rs = 2.95;
Jm = 0.01838; 
speed = 6.28*10 # Speed desired
landa = 10
eta = 1
npp = 1;
# Differential System Equation
def nonlinear(t,x):
     return np.array(   [  (1/Ld)*( vd(t)- Rs*x[0]+x[3]*Lq*x[1])  , \
                           (1/Lq)*( vq(t)- Rs*x[1]-x[3]*Ld*x[0])  , \
                           x[3], 
                           (1/Jm)*(   (3*npp*(Ld-Lq)/2)*x[1]*x[2] \
                            -B*(1-D+2*D*random.random())*x[3]-TL) + u(t,x)  ])
# Control functions
def u(t,x):
     return ( -fh(x) + xd2(t) - landa*(x[3] - xd1(t)) - (( F(x)+eta )*sign(s(t,x)))  )
def s(t,x):
    return (    x[3]- xd1(t) + landa*(x[2]- xd(t) ) )
def F(x):
    return (D*B*x[3]/Jm)
def fh(x):
    return (   (1/Jm)*(   (3*npp*(Ld-Lq)/2)*x[1]*x[2]- B*x[3]-TL  )   )  
def sign(a):
    if a<0:
        return -1
    elif a == 0:
        return 0
    else:
        return 1
def vd(t):
    return 20
def vq(t):
    return 20
def xd(t):
    return speed*t
def xd1(t):
    return  speed
def xd2(t):
    return  0

t0, t1 = 0, 20                # start and end
t = np.linspace(0, 20, 1000)  # the points of evaluation of solution
y0 = [0, 0, 0 ,0]                   # initial value
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
plt.legend(["i_d", "i_q"], loc ="lower right")

plt.title("Corrientes")
#plt.ylim(-50, 105);
plt.grid(True)
plt.xlabel("time")
plt.show()

plt.plot(t, y[:,3]*60/6.28);
plt.title("Motor Speed (RPM)")
plt.ylim(590, 610);

plt.grid(True)
plt.xlabel("time")
plt.show()
