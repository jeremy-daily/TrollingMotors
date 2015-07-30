# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

from matplotlib.pyplot import * # Grab MATLAB plotting functions
from control.matlab import *    # MATLAB-like functions

Tconst = 2775
K = 3

plantNum = [K]
plantDen = [Tconst, 1, 0]

#plantNum = [1]
#plantDen = [1,10,20]

plant=tf(plantNum,plantDen)


# Step response for the system

time=linspace(0,300,num=1000)

figure(1)
yout, T = step(plant,time)
plot(T, yout)

figure(2)
rlocus(plant)

Kp=10
Ki=1
Kd=1

contNum = [Kd,Kp,Ki]
contDen = [1,0]

controller = tf(contNum,contDen)

system = feedback(plant*controller,1)

# Step response for the system
figure(3)
yout, T = step(system,time)
plot(T, yout)
print(tfdata(system))

figure(4)
rlist,klist = rlocus(system)
print(rlist)
print(klist)
