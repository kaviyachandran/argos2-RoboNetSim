#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import csv



fname3 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/timestep3.csv"


#fname_goal1 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/goal2.csv"



def getColumn(filename, column):
    results = csv.reader(open(filename), delimiter=",")
    return [result[column] for result in results]



timeStep3 = getColumn(fname3,0)

timeStep3 = np.asarray(timeStep3, dtype=int)

temp = []
tempIndex = []
#temp = np.asarray(temp)

for idx, val in enumerate(timeStep3):
    
    if(idx > 0):
        temp.insert(idx,timeStep3[idx]-timeStep3[idx-1])
    else:
        temp.insert(idx,timeStep3[idx])
    #tempIndex.insert(idx,timeStep3[idx])
print temp  
plt.scatter(timeStep3,temp)
plt.xlabel("Number Of Times")
plt.ylabel("Time Interval between each meeting of agent and relay")
plt.show()
