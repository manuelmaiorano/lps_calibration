from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 



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
            
            
            cost += (distances[i][j] - np.sqrt(np.sum((x[i*N_COORDS:i*N_COORDS+3] - x[j*N_COORDS:j*N_COORDS+3] )**2)))**2

    print(cost)

    return cost

def get_optimized_coords(distances, to_reject=[], x0=np.zeros((N_ANCHORS*3,))):

    cons = ({'type': 'eq', 'fun': lambda x:  x[0]},
            {'type': 'eq', 'fun': lambda x:  x[1]},
            {'type': 'eq', 'fun': lambda x:  x[2]},
            {'type': 'eq', 'fun': lambda x:  x[N_COORDS*5 + 0]},
            {'type': 'eq', 'fun': lambda x:  x[N_COORDS*5 + 2]},
            {'type': 'eq', 'fun': lambda x:  x[N_COORDS*7 + 2]},)

    res = minimize(lambda x: cost(x, distances, to_reject=to_reject), x0=x0, constraints=cons)

    return np.reshape(res.x, (N_ANCHORS, 3))

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

