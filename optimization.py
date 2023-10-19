from scipy.optimize import minimize
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 



N_ANCHORS = 7


def cost(x, distances):
    cost = 0

    for i in range(N_ANCHORS):
        for j in range(N_ANCHORS):
            if i == j:
                continue

            cost += (distances[i][j] - np.sqrt(np.sum((x[i*3:i*3+3] - x[j*3:j*3+3])**2)))**2

    print(cost)

    return cost

def get_optimized_coords(distances):

    cons = ({'type': 'eq', 'fun': lambda x:  x[0]},
            {'type': 'eq', 'fun': lambda x:  x[1]},
            {'type': 'eq', 'fun': lambda x:  x[2]},)
    
    xc = (distances[0][1]**2 + distances[2][0]**2 - distances[2][1]**2)/(2*distances[0][1])
    x0=np.zeros((N_ANCHORS*3,))
    #x0[0:9] = np.array([0,0,0,distances[0][1],0,0, xc, np.sqrt((distances[2][0]**2-xc**2)), 0])

    res = minimize(lambda x: cost(x, distances), x0=x0,constraints=cons)

    return np.reshape(res.x, (N_ANCHORS, 3))

def from_coords_to_distances(coords):
    distances = np.zeros((N_ANCHORS, N_ANCHORS))

    for i in range(N_ANCHORS):
        for j in range(N_ANCHORS):
            distances[i][j] = np.sqrt(np.sum((coords[i] -coords[j])**2))

    return distances

if __name__ == "__main__":
    
    distances = from_coords_to_distances(np.array([[0,0,0],
                                                  [0,1,0],
                                                  [1,0,0],
                                                  [0, 0, 1],
                                                  [0, 0, -1],
                                                  [0, -1, 0],
                                                  [-1, 0, 0]]))
    
    coords = get_optimized_coords(distances)
    
    print(coords)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2])
    plt.show()

