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
B = 0.0045 # Viscous damping coefficient 
D = 0.5 # % Variation of V (%)
Rs = 2.95
Jm = 0.01838 
speed = 6.28*10 # Speed desired
speed2 = 6.28*7.5
speed3 = 6.28*12
landa = 6
eta = 3
npp = 2;
# Differential System Equation
def nonlinear(t,x):
     return np.array(   [  (1/Ld)*( vd(t)- Rs*x[0]+ x[3]*Lq*x[1]   )  , \
                           (1/Lq)*( vq(t)- Rs*x[1]- x[3]*Ld*x[0]    )  , \
                           x[3], 
                           (1/Jm)*(  (  ((3*npp*(Ld-Lq)/2)*x[0]*x[1] ) + u(t,x) )    -B*(1-D+2*D*random.random())*x[3] )   ])
# Control functions
def u(t,x):
     return ( -fh(x) + xd2(t) - landa*(x[3] - xd1(t)) - (( F(x)+eta )*sign(s(t,x)))  )*(Jm)
def s(t,x):
    return (    x[3]- xd1(t) + landa*(x[2]- xd(t) ) ) *Jm
def F(x):
    return (D*B*abs(x[3])/Jm)
def fh(x):
    return (    (1/Jm)*(  ( ((3*npp*(Ld-Lq)/2)*x[0]*x[1] ) ) - B*x[3] )  )
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
    return 24
def vq(t):
    return 24
def xd(t):
    return speed*(t)*(ustep(t) - ustep(t-20)) + speed2*(t)*(ustep(t-20) - ustep(t-40)) + speed3*(t)*(ustep(t-40)) 
def xd1(t):
    return speed*    (ustep(t) - ustep(t-20)) + speed2*    (ustep(t-20) - ustep(t-40)) + speed3*(ustep(t-40) )
def xd2(t):
    return 0

t0, t1 = 0, 20                # start and end
t = np.linspace(0, 60, 6000)  # the points of evaluation of solution
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

plt.plot(t, -us[:,0]);
plt.title("Control")
#plt.ylim(-1000,10000);
plt.grid(True)
plt.xlabel("time")
plt.show()
