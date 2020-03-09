import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import csv

MAP_XMAX = 2.5 # meters
MAP_YMAX = 2.5 # meters
MAP_XMIN = -2.5
MAP_YMIN = -2.5
MAP_ZMAX = 10
MAP_ZMIN = 0.3
RESOLUTION_CM = 10
THRESHOLD = 100



def convert_to_global(local_measurements, pose, quaternion=False):
    sz = np.shape(local_measurements)
    global_points = np.zeros((sz[0],sz[1]-1))
    # cos/sin of each Euler angle
    ca = np.cos(pose[5])
    sa = np.sin(pose[5])
    cb = np.cos(pose[4])
    sb = np.sin(pose[4])
    cg = np.cos(pose[3])
    sg = np.sin(pose[3])

    R = np.zeros((3,3))
    R[0,:] = [ ca*cb, ca*sb*sg - sa*cg, ca*sb*cg + sa*sg ]
    R[1,:] = [ sa*cb, sa*sb*sg + ca*cg, sa*sb*cg - ca*sg ]
    R[2,:] = [ -sb,   cb*sg,            cb*cg ]

    '''
    # reverse rotation
    Rinv = np.linalg.inv(R)

    # shifted points
    local_T = np.transpose(local_measurements)
    shifted_points = np.transpose( Rinv @ local_T[:3,:] )
    '''
    shifted_points = np.array(local_measurements)[:,:3] @ R

    global_points = np.repeat([pose[:3]],sz[0],axis=0) + shifted_points
    return global_points

def load_data(filename):
    # open/read data in file
    with open('scans/'+filename, mode = 'r') as raw_scan:

        file_reader = csv.reader(raw_scan, delimiter=',')

        data = []

        for row in file_reader:
            row_data = []
            for element in row:
                row_data.append(float(element))

            data.append(row_data)

    return data

def read_csvGrid(filename, asGrid = False):
    with open(filename, mode = 'r') as gridCSV:
        file_reader = csv.reader(gridCSV, delimiter=',')
        grid = []
        if asGrid:
            gridCSV.readline()
            for row in file_reader:
                row_data = []
                for element in row:
                    row_data.append(float(element))
                grid.append(row_data)
            grid = np.asarray(grid)
            grid = grid[:,1:]
        else:
            gridCSV.readline()
            for row in file_reader:
                row_data = []
                for element in row:
                    row_data.append(float(element))
                grid.append(row_data[-1])
            grid = np.asarray(grid)
            grid = grid.reshape((281,299))
        return grid

def tranformationMatrix(theta, xy):
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    t = np.array(xy)
    T = np.append(R, t.reshape((2,1)), axis=1)
    T = np.append(T,np.array([0, 0, 1]), axis=0)
    return T

def computeFitness(T, local_xy, localGrid, globalGrid):
    global_est_xy = T @ local_xy                            # apply transformation
    global_est_xy /= global_est_xy[-1,:].repeat(3,axis=0)   # normalize by last element
    global_est_xy = 10*global_est_xy[:2,:]

    global_est = np.zeros(globalGrid.shape)
    (M, N) = global_est_xy.shape
    for i in range(global_est_xy.shape(1)):
        if x < M and x >= 0 and y < N and y >= 0:
            x = int(global_est_xy[0,i])
            y = int(global_est_xy[1,i])
            global_est[x,y] = localGrid[local_xy[0],local_xy[1]]
    
    fitness = np.sum(globalGrid * global_est)
    return fitness

def main():
    # Converting 3d local coordinates to 2d grid
    globalGrid = read_csvGrid('rawObstacleGrid.csv', asGrid = False)

    # initialize pose of lidar and feed in either local_measurements as a numpy array or give filename of csv
    initial_position = [0, 0, 0, 0, 0, 0]
    fileName = ''
    local_measurements = load_data(fileName)

    localGrid_count = np.zeros((int(100*(MAP_XMAX-MAP_XMIN)//RESOLUTION_CM), int(100*(MAP_YMAX-MAP_YMIN)//RESOLUTION_CM)))
    for point in tqdm(local_measurements):
        x = point[0]
        y = point[1]
        z = point[2]

        if x >= MAP_XMIN and x <= MAP_XMAX and y >= MAP_YMIN and y <= MAP_YMAX and z <= MAP_ZMAX and z >= MAP_ZMIN:

            x_idx = int(100*(x-MAP_XMIN)//RESOLUTION_CM)
            y_idx = int(100*(y-MAP_YMIN)//RESOLUTION_CM)

            localGrid_count[x_idx, y_idx] += 1
    localGrid = localGrid_count >= THRESHOLD

    theta = initial_position[5]
    xy = initial_position[:2]
    T = tranformationMatrix(theta, xy)

    fitness = 0
    x = np.arange(localGrid.shape(0))
    y = np.arange(localGrid.shape(1))
    local_xy = np.meshgrid(x,y)
    local_xy = np.append(local_xy[1],local_xy[0], axis = 0)
    local_xy = .1*local_xy.reshape((2,100))
    
    del_theta_list = np.pi/180*np.linspace(-30, 30, num = 20)
    for iteration in range(30):

        # Computing current fitness ----------------------------------------------------------
        fitness = computeFitness(T, local_xy, localGrid, globalGrid)
        
        # Adjusting ---------------------------------------------------------------------------
        temp_theta = theta
        for del_theta in del_theta_list:
            Tp = tranformationMatrix( del_theta + theta, xy)
            temp_fit = computeFitness(Tp, local_xy, localGrid, globalGrid)
            if temp_fit > fitness:
                fitness = temp_fit
                temp_theta = del_theta + theta

        theta = temp_theta

if __name__ == "__main__":
    main()