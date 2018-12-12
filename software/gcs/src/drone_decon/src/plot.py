import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

crashPath = "crashfile.txt"
drone1Path = "drone1file.txt"
drone2Path = "drone2file.txt"

while True:
	print("Update!")
	crashSitesX = []
	crashSitesY = []
	crashSitesZ = []
	drone1Start = []
	drone1X = []
	drone1Y = []
	drone1Z = []

	drone2Start=[]
	drone2X = []
	drone2Y = []
	drone2Z = []

	lines = [line.rstrip() for line in open(crashPath)]
	for line in lines:
		newline = line.split(', ')
		crashSitesX.append(float(newline[0]))
		crashSitesY.append(float(newline[1]))
		crashSitesZ.append(float(newline[2]))

	lines = [line.rstrip() for line in open(drone1Path)]
	for line in lines:
		newline = line.split(', ')
		drone1X.append(float(newline[0]))
		drone1Y.append(float(newline[1]))
		drone1Z.append(float(newline[2]))
	drone1Start = [drone1Y[0], drone1X[0]]  

	lines = [line.rstrip() for line in open(drone2Path)]
	for line in lines:
		newline = line.split(', ')
		drone2X.append(float(newline[0]))
		drone2Y.append(float(newline[1]))
		drone2Z.append(float(newline[2]))
	drone2Start = [drone2Y[0], drone2X[0]]

	map = Basemap(projection='cyl', llcrnrlat=-90,urcrnrlat=90,\
		        llcrnrlon=-180,urcrnrlon=180,resolution='l')
	map.drawcountries(linewidth=0.25)
	map.drawcoastlines(linewidth=0.25)

	map.scatter(drone1Y, drone1X, c="b", marker="x")
	map.scatter(drone2Y, drone2X, c="g", marker="o")
	map.scatter(crashSitesY, crashSitesX, c="r", s=100)
	map.scatter(drone1Start[0], drone1Start[1], c="y", marker="o")
	map.scatter(drone2Start[0], drone2Start[1], c="y", marker="o")
	plt.title("Drone deconfliction map")
	plt.show()



'''fig = plt.figure()
ax = Axes3D(fig)

ax.scatter(crashSitesY, crashSitesX, crashSitesZ, c="r", marker="o", s=100, label="Crash Sites")
ax.scatter(drone1Y, drone1X, drone1Z, c="b", marker="x", label="Drone 1")
ax.scatter(drone2Y, drone1X, drone2Z, c="g", marker="o", label="Drone 2")


ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
ax.set_zlabel("Altitude")

plt.show()'''
