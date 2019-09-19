import numpy as np
import matplotlib.pyplot as plt
import math
import os
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull, convex_hull_plot_2d
from matplotlib import colors
from io import StringIO
import csv

numFiles = [7,7,8,8,9,9,9,9,10,10]
path1 = r'./Webots_'
path2 = r'/Wolf'
path3 = r'.csv'


simData = []

index =0
fig = plt.figure()
for num in numFiles:
	fileNames = [(path1+str(index+1)+path2+str(i)+path3) for i in range(num)]
	robot_data = []
	for file in fileNames:
		data = np.genfromtxt(file, delimiter = ',')
		robot_data.append(data)
	simData.append(robot_data)
	index = index+1

setNo = 0
for simulation in simData:
	parameter = 0
	if (len(simulation[1][0]) == 4):
		fig = plt.figure()
		for robot in simulation:
			plt.plot(robot[:,0],robot[:,-1],'-',label ='Robot No '+str(parameter))
			parameter = parameter+1
		plt.legend(loc='best')
		plt.xlabel('Iteration Number')
		plt.ylabel('Time Elapsed in Seconds')
		plt.title('Parameter set '+str(setNo+1))
	setNo = setNo+1
plt.show()
# 	print(fileNames)

# # 	for simulation in simData[1:]:
# # 		fig.add_subplot(3, 3, index)
# # 		for robotData in simulation:
# # 			plt.plot(robotData[:,1],robotData[:,2],'o')
# # 			plt.title('Scatterplot for Parameter Set '+str(index+1))
# # 	index = index +1
# # plt.suptitle('Scatterplot of robots positions in each iteration superimposed')

# index = 1
# fig = plt.figure()
# for simulation in simData[:-4]:
# 	fig.add_subplot(2, 3, index)
# 	for robotData in simulation:
# 		plt.plot(robotData[:,0],robotData[:,-1],'-')
# 		plt.title('Time per iteration for Parameter set '+str(index+1))
# 		# index = index+1

# plt.suptitle('Time taken per iteration')# 