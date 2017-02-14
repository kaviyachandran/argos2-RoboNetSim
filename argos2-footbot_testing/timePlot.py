#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import csv
import matplotlib.mlab as mlab

fname3 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/Data/1-1-1 case/Run1/timestep3.csv"


#fname_goal1 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/goal2.csv"



def getColumn(filename, column):
    results = csv.reader(open(filename), delimiter=",")
    return [result[column] for result in results]



timeStep3 = getColumn(fname3,0)

timeStep3 = np.asarray(timeStep3, dtype=int)

temp = []
tempIndex = []
#temp = np.asarray(temp)
sum_val = 0
tempsum = 0

for idx, val in enumerate(timeStep3):
    
    if(idx > 0):
        temp.insert(idx,timeStep3[idx]-timeStep3[idx-1])
        sum_val = sum_val + (timeStep3[idx]-timeStep3[idx-1])
    else:
    	temp.insert(idx,timeStep3[idx])
    	sum_val = timeStep3[idx]

    

print temp  


#plt.figure(1)
#result = plt.hist(temp)
#plt.xlim((min(temp), max(temp)))

mu = np.mean(temp)
variance = np.var(temp)
sigma = np.sqrt(variance)

n, bins, patches = plt.hist(temp, 50, normed=1, facecolor='green', alpha=0.75)
#y = mlab.normpdf( bins, mu, sigma)
#l = plt.plot(bins, y, 'r--', linewidth=1)
#x = np.linspace(min(temp), max(temp), 100)
#dx = result[1][1] - result[1][0]
#scale = len(temp)*dx
#print "scale", scale
#plt.plot(x, mlab.normpdf(x, mean, sigma)*scale)





#markerline, stemlines, baseline = plt.stem(timeStep3, temp, '-.')
#plt.legend(["Interval between data gathering"])
plt.title("Data Gathering Frequency - 1-1-1 case")
#plt.setp(baseline, 'color', 'r', 'linewidth', 2)

#plt.show()
#plt.scatter(timeStep3,temp)
plt.xlabel("Timestep [10 Timesteps = 1 second]")
#plt.ylabel("Time Interval between each meeting of agent and relay")
plt.axis([min(temp), max(temp), 0, 0.07])
plt.grid(True)

plt.show()
