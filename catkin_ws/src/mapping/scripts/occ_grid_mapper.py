import numpy as np
import matplotlib.pyplot as plt
import math
import csv

MAP_WIDTH = 100 # meters
MAP_LENGTH = 100 # meters
RESOLUTION_CM = 10

def plot_grid(grid):
    # display grid
    plt.figure(1)
    plt.imshow(grid, aspect = 'auto', origin = 'upper', cmap='jet')
    plt.show()
    plt.figure(2)
    obsGrid = grid >= threshold
    plt.imshow(obsGrid, aspect = 'auto', origin = 'upper')
    plt.show()


def convert_to_global(local_measurements, poses):
    pass


def load_data(filename):
    # open/read data in file
    with open(filename, mode = 'r') as raw_scan:

        file_reader = csv.reader(raw_scan, delimiter=',')

        data = []

        for row in file_reader:
            row_data = []
            for element in row:
                row_data.append(float(element))

            data.append(row_data)

    return data


def main():
    files = [ 'stationary_scan_vlsi_lab.csv']  # list of log names for the map
    poses = [[0, 0, 0, 0]]  # list of poses (x, y, z, theta) of each log in global frame

    global_measurements = []

    # Preprocessing steps to convert measurments to global frame
    for idx, f in enumerate(files):
        local_measurements = load_data(f)
        gm = convert_to_global(local_measurements, poses[idx])

        global_measurements += gm

    #TODO(aqp/kahiwa): change map_width/length name to something meaninful
    occ_grid_count = np.zeros((MAP_WIDTH//RESOLUTION_CM, MAP_LENGTH//RESOLULTION_CM))

    for point in global_measurements:
        x = point[0]
        y = point[1]
        z = point[2]

        if x >= 0 and x <= MAP_WIDTH and y >= 0 and y <= MAP_LENGTH and z <= MAP_HEIGHT:

            x_idx = x//RESOLUTION_CM
            y_idx = y//RESOLUTION_CM

            occ_grid_count[x_idx, y_idx] += 1


    return 0

if __name__ == "__main__":
    main()
