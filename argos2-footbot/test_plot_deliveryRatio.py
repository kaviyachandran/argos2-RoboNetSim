#!/usr/bin/env python

from collections import defaultdict
import matplotlib.pyplot as plt
import numpy as np
import csv

import matplotlib.cbook as cbook

'''
x = ['2','3','hey','hi']
print x[0]
x_len = len(x)

i = 0
test  = {}
for element in x:
	if(element.isdigit()):
		test[i] = element
	else:
		print element
	i = i + 1
'''
def getColumn(filename, column):
	re = []
	results = csv.reader(open(filename), delimiter=",")
	return [result[column] for result in results]


fname7 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot/generated_data7.csv"
fname8 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot/generated_data8.csv"
fname9 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot/generated_data9.csv"
fname0 = "/home/kavya/RoboNetSim/argos2-RoboNetSim/argos2-footbot/generated_data10.csv"

a2 = getColumn(fname7,0)
a3 = getColumn(fname7,1)
a4 = getColumn(fname7,2)

b2 = getColumn(fname8,0)
b3 = getColumn(fname8,1)
b4 = getColumn(fname8,2)

c2 = getColumn(fname9,0)
c3 = getColumn(fname9,1)
c4 = getColumn(fname9,2)

d2 = getColumn(fname0,0)
d3 = getColumn(fname0,1)
d4 = getColumn(fname0,2)



def calculate_delRatio(x,y,z):
	relay_time = defaultdict(list)
	relay_dataSize = defaultdict(list)
	count = 0
	discarded_packets = 0
	relays_met = []
	for i in z:
		if(i.isdigit()):
			relays_met.append(i)
			relay_time[i].append(x[count])
			
			relay_dataSize[i].append(y[count])
		else:
			discarded_packets = discarded_packets + 1
		count = count + 1

	relays_met = list(set(relays_met))
	data_size = discarded_packets
	
	for i in relays_met:
		for j in relay_dataSize[i]:
			data_size = int(j)+ data_size

	#print data_size, discarded_packets
	return relays_met, relay_time, relay_dataSize, data_size, discarded_packets

relays_met7,relay_time7,relay_dataSize7,data_size7,discarded_packets7 = calculate_delRatio(a2,a3,a4)
relays_met8,relay_time8,relay_dataSize8,data_size8,discarded_packets8 = calculate_delRatio(b2,b3,b4)
relays_met9,relay_time9,relay_dataSize9,data_size9,discarded_packets9 = calculate_delRatio(c2,c3,c4)
relays_met0,relay_time0,relay_dataSize0,data_size0,discarded_packets0 = calculate_delRatio(d2,d3,d4)

print relay_dataSize0['5'],relay_dataSize0['6']
result = [(float(data_size7-discarded_packets7)/data_size7)*100, (float(data_size8-discarded_packets8)/data_size8)*100, (float(data_size9-discarded_packets9)/data_size9)*100, (float(data_size0-discarded_packets0)/data_size0)*100]

N = 4 # No.of mission agents


ind = np.arange(N)  # the x locations for the groups
width = 0.25       # the width of the bars


plt.bar(ind, result, width, color='b')


# add some text for labels, title and axes ticks
plt.ylabel('Percentage of data delivered')
plt.title('Delivery Ratio for mission agents')
plt.xticks(ind + width/2.,('M1', 'M2', 'M3', 'M4') ) 
#plt.xlabels()

plt.show()

