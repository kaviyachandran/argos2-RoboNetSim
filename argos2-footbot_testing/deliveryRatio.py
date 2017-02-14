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

ttl = int(2500)


fname3 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/data2.csv"
fname0 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot_testing/delayTime.csv"

dataSent = getColumn(fname3,0)
dataSent = np.asarray(dataSent, dtype = 'int')

dataDiscarded = getColumn(fname3,1)
dataDiscarded = np.asarray(dataDiscarded, dtype = 'int')

delayTime = getColumn(fname0,0)
delayTime = np.asarray(delayTime, dtype = 'int')

totalData = []
datadiscardedToBase = []

for idx,val in enumerate(delayTime):
	print idx, dataSent[idx],dataDiscarded[idx]
	totalData.append(dataSent[idx] + dataDiscarded[idx])

	print (dataSent[idx]+delayTime[idx])-ttl

	if (dataSent[idx]+delayTime[idx])-ttl > 0:
		datadiscardedToBase.append((dataSent[idx]+delayTime[idx]) - ttl)

print "data produced", sum(totalData)
print "data delivered to base", sum(datadiscardedToBase)
print "delivery ratio", (sum(totalData)-sum(datadiscardedToBase))/(sum(totalData)*1.0)