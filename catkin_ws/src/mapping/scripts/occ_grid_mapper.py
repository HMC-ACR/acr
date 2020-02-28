import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import csv

MAP_XMAX = (92*12+4)/39.37 # meters
MAP_YMAX = (98*12+3)/39.37 # meters
MAP_ZMAX = 10
MAP_ZMIN = 0.1
RESOLUTION_CM = 10
THRESHOLD = 1000

def plot_grid(grid, obsGrid):
    # display grid
    eps = 1e-9
    plt.figure(1)
    plt.imshow(np.log10(grid+eps), aspect = 'auto', origin = 'upper', cmap='jet')
    plt.show()
    plt.figure(2)
    plt.imshow(obsGrid, aspect = 'auto', origin = 'upper')
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


def main():
    # list of log names for the map
    files = [ '2020_2_17__11_44_18.csv', '2020_2_17__11_50_3.csv', '2020_2_17__11_56_15.csv' ]#, '2020_2_17__12_1_10.csv', '2020_2_17__12_6_15.csv', '2020_2_17__12_10_16.csv', '2020_2_17__12_13_55.csv', '2020_2_17__12_18_20.csv', '2020_2_17__12_23_41.csv' ]
    # list of poses (x, y, z, roll, pitch, yaw) of each log in global frame
    poses = [[-0.2032, -0.127, 1.2319, 0, 0, 0], [-3.8105, 9.9314, 1.2319, 0, 0, 0], [-11.4935, -1.7653, 1.2319, 0, 0, 0] ]#, [14.097, -1.77165, 1.2319, 0, 0, 0], [-3.8735, -13.2715, 1.2319, 0, 0, 0], [-11.176, -13.5255, 1.2319, 0, 0, 0], [10.7696, -13.5255, 1.2319, 0, 0, 0], [-11.176, 9.9314, 1.2319, 0, 0, 0], [10.7696, 9.9314, 1.2319, 0, 0, 0]]
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
    plot_grid(occ_grid_count, obsGrid)

    return 0

if __name__ == "__main__":
    main()
