
import logging
import sys
import time

import numpy as np
import optimization

import log_util

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from lpslib.lopoanchor import LoPoAnchor


import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 

np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})


URI = 'radio://0/80/2M/E7E7E7E703'

N_ANCHORS = 8
N_DATA = 300

distances_avg = np.zeros((N_ANCHORS, N_ANCHORS))
distances_count = np.zeros((N_ANCHORS, N_ANCHORS))
distances_history = np.zeros((N_DATA, N_ANCHORS, N_ANCHORS))

def print_coords(coords):

    s = ""
    for i in range(8):
        s += f"{i}:\n"
        for j, coord in enumerate("xyz"):
            s += f"  {coord}: {coords[i, j]:.2f}\n"

    return s


def split(a, n):
    k, m = divmod(len(a), n)
    return (a[i*k+min(i, m):(i+1)*k+min(i+1, m)] for i in range(n))

SPEED_OF_LIGHT = 299792458
ANTENA_DELAY = 154.6
FREQ = 499.2e6 * 128

def process(x):
    if not x:
        return 0
    return x/(FREQ) * SPEED_OF_LIGHT - ANTENA_DELAY

def log_callback(name, timestamp, data, logconf):
    print("-"*50)
    print(name)
    list(map(lambda x: print(f"{x[0]} :  {x[1]} ... {process(x[1])} "), data.items()))
    print("-"*50)


def get_log_keys():
    logs = []
    for i in range(8):
        for j in range(8):
            if i == j:
                continue
            logs.append((f"tdoa2.dist{i}-{j}", "uint16_t"))

    return list(split(logs, 5))


def log_update_distances(name, timestamp, data, logconf):
   
    global distances_avg
    global distances_count
    global distances_history
    for name, value in data.items():
        i = int(name[10])
        j = int(name[12])
        distances_history[int(distances_count[i][j])][i][j] = process(value)
        distances_avg[i][j] = distances_avg[i][j] * distances_count[i][j] + process(value)
        distances_count[i][j] += 1
        distances_avg[i][j] /= distances_count[i][j]


def write_to_anchors(coords):
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        anchors = LoPoAnchor(scf.cf)
        for i in range(N_ANCHORS):
            anchors.set_position(i, coords[i][:])

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(URI, cf=cf) as scf:
        for i, log in enumerate(get_log_keys()):
            log_util.add_log(scf, f"loco_log{i}", log, cb=log_update_distances)
            

        # log_util.add_log(scf, "loco_log", [("tdoa2.dist0-1", "uint16_t"),
        #                                    ("tdoa2.dist1-0", "uint16_t"),
        #                                   ], cb=log_callback)

        time.sleep(N_DATA/10)

    #print(distances_history)
    with open(f"data_dist/dist_{5}.txt", "w") as f:
        for n in range(distances_history.shape[0]):

            print(distances_history[n, :, :], file=f)

    for i in range(distances_history.shape[0]):
        print(distances_history[i, :, :])

    print("optimization")

    coords = optimization.get_optimized_coords(np.median(distances_history[10:, :, :], 0))
    print(np.median(distances_history[10:, :, :], 0))
    #coords = optimization.get_optimized_coords(np.median(distances_history, 0))
    print(coords)
    #coords = optimization.infer_labels(coords)
   
    print("inferred order")
    print(coords)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2])
    for i in range(N_ANCHORS):
        ax.text(coords[i, 0], coords[i, 1], coords[i, 2], f"{i}", color="red")
    plt.show()
    # write_to_anchors(coords)

    import get_coord

    print(get_coord.real_data-coords)
    #np.savetxt(f"data_cal/trial_{9}", get_coord.real_data-coords)
    print_coords(coords)