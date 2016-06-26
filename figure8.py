#!/usr/env/python 
#-*- coding: utf-8 -*-
"""
Python Code
Created on Sat Jun 25 17:19:35 2016
by
author: jeremy-daily

Title:

Description:

"""

import matplotlib.pyplot as plt
from math import *
import numpy as np

x0=95
y0=36
heading=345 #degrees

t = np.arange(0,2*np.pi,.1)

a = .0001

x = a * np.sin(t)
dx = a 
y = 2 * a * np.sin(t)* np.cos(t)
dy =  2*a


#phi= (atan(sqrt(7) - 4*cos(0)) + atan( sqrt(7)+ 4 * cos(0))) - (atan(4-sqrt(7)) + atan(4+sqrt(7)))
phi=atan2(dy,dx)

print(phi)
print(phi*180/pi)

rotationAngle = heading*np.pi/180. + phi

east = x0 + x*np.cos(rotationAngle) - y *np.sin(rotationAngle)
north = y0 + x*np.sin(rotationAngle) + y *np.cos(rotationAngle)

plt.plot(east,north,'o',[-2*a+x0,2*a+x0],[y0,y0])
plt.axis('equal') 
plt.show()