#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import csv

import pickle
from matplotlib.ticker import FuncFormatter
import matplotlib.mlab as mlab



def getColumn(filename, column):
	results = csv.reader(open(filename), delimiter=",")
	return [result[column] for result in results]

temp = []

for i in range(10):

	sizeTemp = np.size(temp)

	fileName = "Run"+ str(i+1)
	fname3 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/Data/1-1-1 case/"+fileName+"/timestep3.csv"
    
	timeStep3 = getColumn(fname3,0)

	timeStep3 = np.asarray(timeStep3, dtype=int)

	
	#temp = np.asarray(temp)
	
	
	for idx, val in enumerate(timeStep3):

		if(idx > 0):
			temp.insert(idx+sizeTemp,timeStep3[idx]-timeStep3[idx-1])
			
		else:
			temp.insert(idx+sizeTemp,timeStep3[idx])
			
print min(temp), max(temp)

mu = np.mean(temp)
variance = np.var(temp)
sigma = np.sqrt(variance)

temp = np.asarray(temp,dtype='f')
temp = [x / 10.0 for x in temp] # converting to seconds 


print temp  


#plt.figure(1)
#result = plt.hist(temp)
#plt.xlim((min(temp), max(temp)))


#n, bins, patches = plt.hist(temp, 50, normed=1, facecolor='green', alpha=0.75)
#y = mlab.normpdf( bins, mu, sigma)
#l = plt.plot(bins, y, 'r--', linewidth=1)
#x = np.linspace(min(temp), max(temp), 100)
#dx = result[1][1] - result[1][0]
#scale = len(temp)*dx
#print "scale", scale
#plt.plot(x, mlab.normpdf(x, mean, sigma)*scale)





#markerline, stemlines, baseline = plt.stem(timeStep3, temp, '-.')
#plt.legend(["Interval between data gathering"])
plt.title("Interval between each data transmission from agent to relay")
#plt.setp(baseline, 'color', 'r', 'linewidth', 2)

#plt.show()
#plt.scatter(timeStep3,temp)
plt.xlabel("Time(seconds)")
plt.ylabel("Frequency")
#plt.ylabel("Time Interval between each meeting of agent and relay")
plt.xlim([0, max(temp)])
plt.grid(True)



plt.hist(temp, bins='auto',facecolor='green', alpha=0.75)  # plt.hist passes it's arguments to np.histogram

plt.grid(True)

plt.show()

'''

def to_percent(y, position):
    # Ignore the passed in position. This has the effect of scaling the default
    # tick locations.
    s = str(100 * y)

    # The percent symbol needs escaping in latex
    if matplotlib.rcParams['text.usetex'] is True:
        return s + r'$\%$'
    else:
        return s + '%'



# Make a normed histogram. It'll be multiplied by 100 later.
plt.hist(temp, bins=50, normed=True)

# Create the formatter using the function to_percent. This multiplies all the
# default labels by 100, making them all percentages
formatter = FuncFormatter(to_percent)

# Set the formatter
plt.gca().yaxis.set_major_formatter(formatter)

plt.show()

'''

