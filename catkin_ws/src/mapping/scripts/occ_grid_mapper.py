import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import csv

MAP_XMAX = (92*12+4)/39.37 # meters
MAP_YMAX = (98*12+3)/39.37 # meters
MAP_ZMAX = 10
MAP_ZMIN = 0.3
RESOLUTION_CM = 10
THRESHOLD = 100

def plot_grid(obsGrid, grid = None):
    # display obstacle grid
    eps = 1e-9
    plt.figure(1)
    plt.imshow(obsGrid, aspect = 'auto', origin = 'upper')
    plt.show()

    # if gridcount is presented
    if grid is not None:
        plt.figure(2)
        plt.imshow(np.log10(grid+eps), aspect = 'auto', origin = 'upper', cmap='jet')
        plt.show()

# borrowed conversion
def euler_to_quaternion(yaw, pitch, roll):
    q = np.zeros(4)
    q[1] = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    q[2] = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    q[3] = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    q[0] = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return q

def qInv(q):
    q_inv = np.zeros(np.shape(q))
    q_inv[0] = q[0]
    q_inv[1:] = -q[1:]
    q_inv /= np.sum(np.multiply(q,q))
    return q_inv

def crossProd3D(a,b):
    c = np.zeros(3)
    c[0] = a[1]*b[2] - a[2]*b[1]
    c[1] = a[2]*b[0] - a[0]*b[2]
    c[2] = a[0]*b[1] - a[1]*b[0]
    return c

def qMult(q1, q2):
    q = np.zeros(4)
    q[0] = q1[0]*q2[0] - np.sum(np.multiply(q1[1:],q2[1:]))
    q[1:] = q1[0]*q2[1:] + q2[0]*q1[1:] + crossProd3D(q1[1:],q2[1:])
    return q

def invRotation(point, pose):
    q_point = np.zeros(4)
    q_point[1:] = point
    q_invRot = euler_to_quaternion(-pose[5], -pose[4], -pose[3])
    q_fp = qMult( qMult( q_invRot, q_point ), qInv( q_invRot ) )
    fp = q_fp[1:]
    return fp

def convert_to_global(local_measurements, pose, quaternion=False):
    sz = np.shape(local_measurements)
    global_points = np.zeros((sz[0],sz[1]-1))
    if quaternion:
        for i, local_point in tqdm(enumerate(local_measurements)):
            if len(local_point)<4: break
            local_point = local_point[:3]
            global_point = pose[:3] + invRotation(local_point, pose)
            global_points[i,:] = global_point
    else:
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

def save_grid(filename, grid, asGrid = False):
    # save occupancy grid to a csv file    
    xMax, yMax = np.shape(grid)
    with open(filename, mode = 'w') as grid_file:
        if asGrid:
            grid_writer = csv.writer(grid_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            grid_writer.writerow([' '] + [i for i in range(yMax)])
            for i in range(xMax):
                grid_writer.writerow([i] + list(grid[i,:]))
        else:
            grid_writer = csv.writer(grid_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            grid_writer.writerow(['x (cm)', 'y (cm)', 'occupied'])
            for i in range(xMax):
                for j in range(yMax):
                    grid_writer.writerow([i*RESOLUTION_CM, j*RESOLUTION_CM, int(grid[i,j])])

def main():
    # list of log names for the map
    files = [ '2020_2_17__11_44_18.csv', '2020_2_17__11_50_3.csv', '2020_2_17__11_56_15.csv', '2020_2_17__12_1_10.csv', '2020_2_17__12_6_15.csv', '2020_2_17__12_10_16.csv', '2020_2_17__12_13_55.csv', '2020_2_17__12_18_20.csv', '2020_2_17__12_23_41.csv' ]
    # list of poses (x, y, z, roll, pitch, yaw) of each log in global frame
    poses = [[-0.2032, -0.127, 1.2319, 0, 0, 0], [-3.8105, 9.9314, 1.2319, 0, 0, 0], [-11.4935, -1.7653, 1.2319, 0, 0, 0], [14.097, -1.77165, 1.2319, 0, 0, 0], [-3.8735, -13.2715, 1.2319, 0, 0, 0], [-11.176, -13.5255, 1.2319, 0, 0, 0], [10.7696, -13.5255, 1.2319, 0, 0, 0], [-11.176, 9.9314, 1.2319, 0, 0, 0], [10.7696, 9.9314, 1.2319, 0, 0, 0]]
    for pose in poses:
        pose[0] += (432+114)/39.37
        pose[1] += (527.5+134.25)/39.37
        pose[5] += np.pi/2
    # Preprocessing steps to convert measurments to global frame
    for idx, f in enumerate(files):
        local_measurements = load_data(f)
        print('Starting conversion to global coordinates for file:',f)
        gm = convert_to_global(local_measurements, poses[idx])
        if (idx==0):
            global_measurements = gm
        else:
            global_measurements = np.append(global_measurements, gm, axis=0)

    occ_grid_count = np.zeros((int(100*MAP_XMAX//RESOLUTION_CM), int(100*MAP_YMAX//RESOLUTION_CM)))
    print('Starting point count on grid...')
    for point in tqdm(global_measurements):
        x = point[0]
        y = point[1]
        z = point[2]

        if x >= 0 and x <= MAP_XMAX and y >= 0 and y <= MAP_YMAX and z <= MAP_ZMAX and z >= MAP_ZMIN:

            x_idx = int(min(100*x//RESOLUTION_CM, 100*MAP_XMAX//RESOLUTION_CM - 1))
            y_idx = int(min(100*y//RESOLUTION_CM, 100*MAP_YMAX//RESOLUTION_CM - 1))

            occ_grid_count[x_idx, y_idx] += 1
    obsGrid = occ_grid_count >= THRESHOLD
    plot_grid(obsGrid, occ_grid_count)

    save_grid('rawObstacleGrid.csv', obsGrid)

    return 0

""" Trimming on original THRESHOLD 100 ZMIN 0.3 m
    grid[115:171,104:192] = 0.
    grid[170:181,134:170] = 0.
    grid[18:28,131:159] = 0.
    grid[240:252,245:254] = 0.
    grid[185:207,237:245] = 0.
    grid[20:23,165:234] = 0.

    # cloud
    grid[85:110,65:103] = 0.
    #grid[68:101,106:132] = 0.
    #grid[68:98,160:195] = 0.
    grid[111:240,235:284] = 0.

    grid[23,64:136] = 1.
    grid[23:56,136] = 1.
    grid[23,165:234] = 1.
    grid[113:245,236] = 1.
"""

if __name__ == "__main__":
    #main()
    grid = read_csvGrid('rawObstacleGrid_gridFormat.csv', asGrid = True)
    plot_grid(grid)

    """ Saving grid files in both formats
    save_grid('rawObstacleGrid_gridFormat.csv', grid, asGrid=True)
    save_grid('rawObstacleGrid.csv', grid, asGrid=False)
    """