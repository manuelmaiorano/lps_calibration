from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 


def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    """

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


N_ANCHORS = 8
N_COORDS = 3

idx_to_msk = {0: np.array([0, 0, 0]), 5: np.array([0, 1, 0]), 7: np.array([1, 1, 0]), 1: np.array([1, 1, 1]),
               2: np.array([1, 1, 1]), 3: np.array([1, 1, 1]), 4: np.array([1, 1, 1]), 6: np.array([1, 1, 1])}

def cost(x, distances, to_reject=[]):
    cost = 0

    for i in range(N_ANCHORS):
        for j in range(N_ANCHORS):
            if i == j:
                continue

            if (i,j) in to_reject or (j,i) in to_reject:
                continue
            
            
            cost += (distances[i][j] - np.sqrt(np.sum((x[i*N_COORDS:i*N_COORDS+3] - x[j*N_COORDS:j*N_COORDS+3] )**2)) )**2 

    print(cost)

    return cost

def cost1(x, distances, to_reject=[]):
    cost = 0

    for i in range(N_ANCHORS):
        for j in range(N_ANCHORS):
            if i >= j:
                continue

            if (i,j) in to_reject or (j,i) in to_reject:
                continue
            
            
            cost += (distances[i][j] +154.6  - np.sqrt(np.sum((x[i*N_COORDS:i*N_COORDS+3] - x[j*N_COORDS:j*N_COORDS+3] )**2)) + x[N_ANCHORS * N_COORDS + i])**2 

    cost +=  np.sum((x[N_ANCHORS*N_COORDS:] + 154.6)**2)
    print(cost)

    return cost


def cost2(x, distances, to_reject=[]):
    cost = 0

    for i in range(N_ANCHORS):
        for j in range(N_ANCHORS):
            if i >= j:
                continue

            if (i,j) in to_reject or (j,i) in to_reject:
                continue
            
            ioffset = np.array([0, 0, 1]) * x[N_ANCHORS * N_COORDS + i]
            joffset = np.array([0, 0, 1]) * x[N_ANCHORS * N_COORDS + j]
            cost += (distances[i][j] - np.sqrt(np.sum((x[i*N_COORDS:i*N_COORDS+3] +ioffset - x[j*N_COORDS:j*N_COORDS+3] -joffset)**2)))**2

    cost +=  np.sum(x[N_ANCHORS*N_COORDS:] **2)/500
    print(cost)

    return cost

def cost3(x, distances, to_reject=[]):
    cost = 0

    for i in range(N_ANCHORS):
        for j in range(N_ANCHORS):
            if i >= j:
                continue

            if (i,j) in to_reject or (j,i) in to_reject:
                continue
            
            cost += (distances[i][j] - np.sqrt(np.sum((x[i*N_COORDS:i*N_COORDS+3] - x[j*N_COORDS:j*N_COORDS+3] )**2)) )**2 

    for i in range(N_ANCHORS):
        if i == 1 or i == 3 or i == 4 or i == 6:
            cost += (x[N_COORDS*i + 2] - 2.2)**2
    print(cost)

    return cost

def cost4(x, distances, to_reject=[]):
    cost = 0
    idx = 0
    for i in range(N_ANCHORS):
        for j in range(N_ANCHORS):
            if i >= j:
                continue

            if (i,j) in to_reject or (j,i) in to_reject:
                continue
            
            cost += (distances[i][j] +x[N_COORDS*N_ANCHORS+idx] - np.sqrt(np.sum((x[i*N_COORDS:i*N_COORDS+3] - x[j*N_COORDS:j*N_COORDS+3] )**2)) )**2 
            idx += 1

    cost +=  np.sum((x[N_ANCHORS*N_COORDS:] )**2)
    for i in range(N_ANCHORS):
        if i == 1 or i == 3 or i == 4 or i == 6:
            cost += (x[N_COORDS*i + 2] - 2.2)**2
    print(cost)

    return cost

def get_optimized_coords(distances, to_reject=[], x0=np.zeros((N_ANCHORS*3 + 8,))):

    cons = ({'type': 'eq', 'fun': lambda x:  x[0]},
            {'type': 'eq', 'fun': lambda x:  x[1]},
            {'type': 'eq', 'fun': lambda x:  x[2]},
            {'type': 'eq', 'fun': lambda x:  x[N_COORDS*5 + 0]},
            {'type': 'eq', 'fun': lambda x:  x[N_COORDS*5 + 2]},
            {'type': 'eq', 'fun': lambda x:  x[N_COORDS*7 + 2]},)
    
    x0[N_ANCHORS*N_COORDS:] = -154.6
    #x0[N_ANCHORS*N_COORDS:] =  np.random.normal(loc=0, scale=2, size=(28, ))
    #x0[:] =  np.random.normal(loc=0, scale=5, size=(24+28, ))
    #x0[N_COORDS*6 + 2] = 2

    res = minimize(lambda x: cost3(x, distances, to_reject=to_reject), x0=x0, constraints=cons)
    print(res.x)
    out = np.reshape(res.x[:N_COORDS *N_ANCHORS], (N_ANCHORS, 3))
    flipz = 1 if out[6, 2]  > 0 else -1
    flipx = 1 if out[2, 0]  > 0 else -1
    flipy  = 1 if out[5, 1]  > 0 else -1
    return out * np.array([1*flipx, 1*flipy, 1*flipz])

def from_coords_to_distances(coords):
    distances = np.zeros((N_ANCHORS, N_ANCHORS))

    for i in range(N_ANCHORS):
        for j in range(N_ANCHORS):
            distances[i][j] = np.sqrt(np.sum((coords[i] -coords[j])**2))

    return distances

def infer_labels(coords):
    reference = np.array([[0,0,0],
                        [0, 1, 1],
                        [1, 1, 0],
                        [1, 0, 1],
                        [0, 0, 1],
                        [0, 1, 0],
                        [1, 1, 1],
                        [1, 0, 0]])
    
    maxz = np.max(coords[:, 2])
    maxy = np.max(coords[:, 1])
    maxx = np.max(coords[:, 0])

    coords_ = coords / np.array([maxx, maxy, maxz])
    
    label2coord = np.zeros((N_ANCHORS, N_COORDS))
    
    for i in range(N_ANCHORS):
        coord = coords_[i][:]

        label = np.argmin(np.sum((reference - coord)**2, 1))
        label2coord[label][:] = coords[i][:]

    return label2coord

def show(coords):

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2])
    for i in range(N_ANCHORS):
        ax.text(coords[i, 0], coords[i, 1], coords[i, 2], f"{i}", color="red")
    #set_axes_equal(ax)
    plt.show()

if __name__ == "__main__":
    
    # distances = from_coords_to_distances(np.array([[0,0,0],
    #                                               [0,1,0],
    #                                               [1,0,0],
    #                                               [0, 0, 1],
    #                                               [0, 0, -1],
    #                                               [0, -1, 0],
    #                                               [-1, 0, 0]]))
    
    distances = from_coords_to_distances(np.array([[0,0,0.18],
                                                  [0.38,3.94,2.25],
                                                  [3.95,3.94,0.18],
                                                  [4.5, -1.1, 2.25],
                                                  [0, 0, 2.25],
                                                  [0.382, 3.94, 0.18],
                                                  [3.952, 3.94, 2.25],
                                                  [4.5, -1.10, 0.18]]))
    
    to_reject = [(0,4), (1,5), (2,6), (3,7)]
    
    coords = get_optimized_coords(distances, to_reject=to_reject)
    
    print("results")
    print(coords)
    coords = infer_labels(coords)
    print("inferred order")
    print(coords)

    show(coords)

