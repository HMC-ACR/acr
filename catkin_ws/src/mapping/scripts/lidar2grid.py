import numpy as np
import matplotlib.pyplot as plt
import math
import csv

scan_file = 'stationary_scan_vlsi_lab.csv'

height = 2                      # height at which all points above (on z-axis) are ignored in count
threshold = 1000                # potential threshold # of pts to distinguish obstacle vs. LIDAR noise
N = 1107422                     # number of data points in scan

# initializing vector of datapoints: each row has form (x, y, z)
dataPoints = np.zeros((N,3))

# open/read data in file
with open(scan_file, mode = 'r') as raw_scan:
    for i in range(N):
        new_line = raw_scan.readline(64)        # read 64 bytes of data (amt. in one line)
        data_inLine = new_line.split(',')       # separate entries based on commas (csv format)
        # enter datapoints converted from str to float format
        dataPoints[i,:] = [ float(data_inLine[0]), float(data_inLine[1]), float(data_inLine[2])]

# establish range of mapping
# note that the min() function was returning exceptionally large xMin and yMin values (~40 m)
xRange = [ -6, max(dataPoints[:,0]) ]
yRange = [ -6, max(dataPoints[:,1]) ]
zRange = [ min(dataPoints[:,2]), max(dataPoints[:,2]) ]

gridResolution = 100                            # minimum resolution of grid
xDiv = (xRange[1]-xRange[0])/gridResolution     # potential x div./spacing
yDiv = (yRange[1]-yRange[0])/gridResolution     # potential y div./spacing
div = min(xDiv,yDiv)

# construct grid as ndarray with dimensions corresponding to ranges incremented by div
# grid has "bottom" corner on minimum x/y positions
grid = np.zeros( (math.floor((xRange[1]-xRange[0])/div), math.floor((yRange[1]-yRange[0])/div)) )
(xMax, yMax) = np.shape(grid)       # dimensions of grid

# go through each 3D point to consider placement in 2D grid
for i in range(N):
    (x,y,z) = dataPoints[i,:]                   # capture a datapoint
    if z<=height:                               # meets height req.
        # grid index based on distance from minimum values
        # might want to check indexing
        gridX = math.floor((x-xRange[0])/div)-1
        gridY = math.floor((y-yRange[0])/div)-1
        grid[gridX, gridY] += 1

# display grid
plt.figure(1)
plt.imshow(grid, aspect = 'auto', origin = 'upper', cmap='jet')
plt.show()
plt.figure(2)
obsGrid = grid >= threshold
plt.imshow(obsGrid, aspect = 'auto', origin = 'upper')
plt.show()