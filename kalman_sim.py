import numpy as np
import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # <--- This is important for 3d plotting 

GRAVITY_VEC = - np.array([0,0,1]).reshape((3,1)) *9.81

procnoiseaccxy = 0.5
procnoiseaccz = 1.0

procnoisegyrorp = 0.1
procnoisegyroy = 0.1
procnoiseatt = 0

procnoisevel = 0
procnoisepos = 0

initialstdxy = 100
initialstdz = 1
initialstvel = 0.01
initialstdrp = 0.01
initialstdy = 0.01

tdoastdv = 0.15

def from_quat(q):
    R = np.zeros((3,3))
    R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]
    R[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3]
    R[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2]

    R[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3]
    R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]
    R[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1]

    R[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2]
    R[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1]
    R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
    return R


def cross_product_matrix(a):
    mat = np.zeros((3,3))
    mat[0, 1] = -a[2]
    mat[1, 0] = a[2]
    mat[0, 2] = a[1]
    mat[2, 0] = -a[1]
    mat[1, 2] = -a[0]
    mat[2, 1] = a[0]
    return mat

def enforce_simmetry(mat):
    MAX_COV = 100
    MIN_COV = 1e-6
    for i in range(mat.shape[0]):
        for j in range(mat.shape[0]):
            p = 1/2*(mat[i][j]+mat[j][i])
            if math.isnan(p) or p > MAX_COV:
                mat[i][j] = MAX_COV
                mat[j][i] = MAX_COV
            elif i == j and p < MIN_COV:
                mat[i][j] = MIN_COV
                mat[j][i] = MIN_COV
            else:
                mat[i][j] = p
                mat[j][i] = p


class KalmanFilter:
    def __init__(self) -> None:

        self.S = np.zeros((9, 1))
        self.P = np.diag([initialstdxy**2, initialstdxy**2, initialstdz**2, 
                          initialstvel**2, initialstvel**2, initialstvel**2,
                          initialstdrp**2, initialstdrp**2, initialstdy**2])
        self.R = np.eye(3)
        self.q = np.zeros((4, 1))
        self.q[0] = 1
        self.S[0] = 0.1
        self.S[1] = 0.1
        self.S[2] = 0.5

        self.lastpredt = 0
        self.lastnoiseupdt = 0
        self.isupdated = False

    def predict(self, acc, gyro, now):

        dt = now - self.lastpredt
        self.lastpredt = now
        A = np.eye(9)
        A[:3, 3:6] = self.R * dt
        A[:3, 6:] = self.R @ cross_product_matrix(self.S[3:6]) *dt
        A[3:6, 3:6] = cross_product_matrix(gyro) * dt  + np.eye(3)
        A[3:6, 6:] = cross_product_matrix(self.R @ GRAVITY_VEC) *dt
        d = gyro * dt/2
        A[6, 6] = 1 -d[1]**2/2 - d[2]**2/2
        A[6, 7] = d[2] + d[0]*d[1]/2
        A[6, 8] = -d[1] + d[0]*d[2]/2

        A[7, 6] = -d[2] + d[0]*d[1]/2
        A[7, 7] = 1 -d[0]**2/2 - d[2]**2/2
        A[7, 8] = d[0] + d[1]*d[2]/2

        A[8, 6] = d[1] + d[0]*d[2]/2
        A[8, 7] = -d[0] + d[1]*d[2]/2
        A[8, 8] = 1 -d[0]**2/2 - d[1]**2/2

        self.P = A @ self.P @ A.T

        acc = acc.reshape((3,1))
        self.S[:3] += self.R @ (self.S[3:6]*dt + (acc*dt**2)/2)+ GRAVITY_VEC *dt**2/2
        self.S[3:6] += (acc + cross_product_matrix(gyro) @ self.S[3:6] + self.R @ GRAVITY_VEC)*dt

        dtwx = dt*gyro[0]
        dtwy = dt*gyro[1]
        dtwz = dt*gyro[2]
        angle = math.sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + 1e-6
        ca = math.cos(angle/2)
        sa = math.sin(angle/2)
        dq = np.array([ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle])

        tmpq = np.zeros((4,1))
        tmpq[0] = dq[0]*self.q[0] - dq[1]*self.q[1] - dq[2]*self.q[2] - dq[3]*self.q[3]
        tmpq[1] = dq[1]*self.q[0] + dq[0]*self.q[1] + dq[3]*self.q[2] - dq[2]*self.q[3]
        tmpq[2] = dq[2]*self.q[0] - dq[3]*self.q[1] + dq[0]*self.q[2] + dq[1]*self.q[3]
        tmpq[3] = dq[3]*self.q[0] + dq[2]*self.q[1] - dq[1]*self.q[2] + dq[0]*self.q[3]

        rpzerorev = 0.001
        keep = 1 - rpzerorev

        tmpq = tmpq *keep + rpzerorev * np.array([1, 0, 0, 0]).reshape((4, 1))
        
        self.q = tmpq/np.linalg.norm(tmpq)

        self.isupdated = True

    def add_process_noise(self, now):
        dt = now - self.lastnoiseupdt
        self.lastnoiseupdt = now
        self.P +=  np.diag([procnoiseaccxy * dt**2 + procnoisevel*dt + procnoisepos,
                            procnoiseaccxy * dt**2 + procnoisevel*dt + procnoisepos,
                            procnoiseaccz * dt**2 + procnoisevel*dt + procnoisepos,
                            procnoiseaccxy * dt + procnoisevel,
                            procnoiseaccxy * dt+ procnoisevel,
                            procnoiseaccz* dt+ procnoisevel,
                            procnoisegyrorp*dt + procnoiseatt,
                            procnoisegyrorp*dt + procnoiseatt,
                            procnoisegyroy*dt + procnoiseatt,
                            ])**2
        
        enforce_simmetry(self.P)

    def update_tdoa(self, tdoa, anchor0, anchor1):
        d0 = np.linalg.norm(self.S[0:3] - anchor0)
        d1 = np.linalg.norm(self.S[0:3] - anchor1)
        H = np.zeros((9, 1))
        H[0:3] = (self.S[0:3] - anchor1.reshape((3,1)))/d1 - (self.S[0:3] - anchor0.reshape((3,1)))/d0
        H = H.T
        error = tdoa - (d1-d0)

        S = H @ self.P @ H.T + tdoastdv**2
        K = self.P @ H.T *1/S
        self.S += K * error

        self.P = (K @ H - np.eye(9)) @ self.P @ (K @ H - np.eye(9)).T

        self.P += K @ K.T * tdoastdv**2

        enforce_simmetry(self.P)

        self.isupdated = True

    def finalize(self):
        if not self.isupdated:
            return
        if (np.abs(self.S[6]) > 0.1e-3 or np.abs(self.S[7]) > 0.1e-3 or np.abs(self.S[8]) > 0.1e-3) and \
            (np.abs(self.S[6]) < 10 and np.abs(self.S[7]) < 10 and np.abs(self.S[8]) < 10) :
            angle = math.sqrt(self.S[6]**2 + self.S[7]**2 + self.S[8]**2) + 1e-6
            ca = math.cos(angle/2)
            sa = math.sin(angle/2)
            dq = np.array([ca , sa*self.S[6, 0]/angle , sa*self.S[7, 0]/angle , sa*self.S[8, 0]/angle])

            tmpq = np.zeros((4,1))
            tmpq[0] = dq[0]*self.q[0] - dq[1]*self.q[1] - dq[2]*self.q[2] - dq[3]*self.q[3]
            tmpq[1] = dq[1]*self.q[0] + dq[0]*self.q[1] + dq[3]*self.q[2] - dq[2]*self.q[3]
            tmpq[2] = dq[2]*self.q[0] - dq[3]*self.q[1] + dq[0]*self.q[2] + dq[1]*self.q[3]
            tmpq[3] = dq[3]*self.q[0] + dq[2]*self.q[1] - dq[1]*self.q[2] + dq[0]*self.q[3]

            self.q = tmpq/np.linalg.norm(tmpq)

            A = np.eye(9)
            d = self.S[6:]/2

            A[6, 6] = 1 -d[1]**2/2 - d[2]**2/2
            A[6, 7] = d[2] + d[0]*d[1]/2
            A[6, 8] = -d[1] + d[0]*d[2]/2

            A[7, 6] = -d[2] + d[0]*d[1]/2
            A[7, 7] = 1 -d[0]**2/2 - d[2]**2/2
            A[7, 8] = d[0] + d[1]*d[2]/2

            A[8, 6] = d[1] + d[0]*d[2]/2
            A[8, 7] = -d[0] + d[1]*d[2]/2
            A[8, 8] = 1 -d[0]**2/2 - d[1]**2/2

            self.P = A @ self.P @ A.T


        self.R = from_quat(self.q)
        self.S[6:] = 0
        enforce_simmetry(self.P)
        self.isupdated = False


N = 100

acc = np.random.normal(loc=0, scale=0.01, size=(3, N)) + np.array([0,0,9.81]).reshape((3,1))
gyro = np.random.normal(loc=0, scale=0.01, size=(3, N))


anch_positions = np.zeros((8, 3))
anch_positions = np.array([[0,0,0.18],
                            [0.38,3.94,2.25],
                            [3.95,3.94,0.18],
                            [4.5, -1.1, 2.25],
                            [0, 0, 2.25],
                            [0.382, 3.94, 0.18],
                            [3.952, 3.94, 2.25],
                            [4.5, -1.10, 0.18]])
true_position = np.array([2, 2, 1])

def get_tdoa_measure(i, j):
    return np.linalg.norm((anch_positions[j, :]) - true_position) -  np.linalg.norm((anch_positions[i, :]) - true_position) + np.random.normal(loc=0, scale=0.0, size=(1,1))[0]

S_history = np.zeros((3, N))

def simulate():
    dt = 0.001
    anch0 = 0
    anch1 = 1

    now = 0
    next_pred = now + 0.01
    
    filter = KalmanFilter()
    for i in range(N):

        now = i *dt
        if now >= next_pred:

            filter.predict(acc[:, i], gyro[:, i], now)
            next_pred = now + 0.01

        filter.add_process_noise(dt)

        tdoa = get_tdoa_measure( anch0, anch1)

        filter.update_tdoa(tdoa, anch_positions[anch0, :],  anch_positions[anch1, :])

        filter.finalize()

        anch0 = (anch0+1)%8
        anch1 = (anch1+1)%8

        print(filter.S[:3])
        S_history[:, i] = filter.S[:3].reshape((3,))


def plot_coords(ax, coords):

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.scatter(coords[:, 0], coords[:, 1], coords[:, 2])
    for i in range(8):
        ax.text(coords[i, 0], coords[i, 1], coords[i, 2], f"{i}", color="red")

simulate()
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
plot_coords(ax, anch_positions)
ax.scatter(S_history[0, :], S_history[1,:], S_history[2,:])
plt.show()




