#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import csv
import matplotlib.patches as mpatches

import matplotlib.cbook as cbook


fname2 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/position2.csv"
#fname3 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/position3.csv"


#fname_goal1 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/goal2.csv"



def getColumn(filename, column):
    results = csv.reader(open(filename), delimiter=",")
    return [result[column] for result in results]


x_0 = getColumn(fname2,0)
y_0 = getColumn(fname2,1)
'''
x_1 = getColumn(fname3,0)
y_1 = getColumn(fname3,1)

xg_2 = getColumn(fname_goal1,0)
yg_2 = getColumn(fname_goal1,1)
'''

#ax = plt.gca()

plt.plot(x_0,y_0,'b--',label="relay1")
#c = mpatches.Circle((x_0[0], y_0[0]), 0.2, facecolor = 'none', edgecolor="red", linewidth=1)

#ax.plot(x_1,y_1,'g--',label="agent1")
#ax.plot(xg_2,yg_2,'o',label="goal agent1")


#ax.add_artist(c)

plt.xlim(-5.0,5.0)
plt.ylim(-5.0,5.0)

plt.show()
