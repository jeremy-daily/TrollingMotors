#!/usr/env/python 
#-*- coding: utf-8 -*-
"""
Python Code
Created on Sat Jun 25 18:30:15 2016
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
heading=00 #degrees

t = np.arange(0,np.pi/2,.1)

r = .0001

x = r * (np.cos(t)) - r #left

x = r * (1-np.cos(t)) #right
dx = r 
y = r * np.sin(t) 
dy =  r




rotationAngle = heading*np.pi/180. - pi/2

east = x0 + x*np.cos(rotationAngle) - y *np.sin(rotationAngle)
north = y0 + x*np.sin(rotationAngle) + y *np.cos(rotationAngle)

plt.plot(east,north,'o',[x0,x0],[-r+y0,r+y0])
plt.axis('equal') 
plt.show()
