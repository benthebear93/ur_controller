import pickle

import matplotlib.pyplot as plt
import numpy as np


def compute_calibration(points_robot, points_world):
    centroid_robot = np.mean(points_robot, axis=0)
    centroid_world = np.mean(points_world, axis=0)
    q_robot = points_robot - centroid_robot
    q_world = points_world - centroid_world
    H = np.zeros((3, 3))
    for qr, qw in zip(q_robot, q_world):
        H += np.outer(qr, qw)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
    T = centroid_world - R @ centroid_robot
    return R, T

def plot_points(points_robot, points_world, transformed_robot):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(points_robot[:, 0], points_robot[:, 1], points_robot[:, 2], color='red', label='Robot Base Points')
    ax.scatter(points_world[:, 0], points_world[:, 1], points_world[:, 2], color='blue', label='World Points')
    ax.scatter(transformed_robot[:, 0], transformed_robot[:, 1], transformed_robot[:, 2], color='green', marker='^', label='Transformed Robot Points')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.title('Robot-World Calibration Visualization')
    plt.show()


with open('robot_base_p.pkl', 'rb') as f:
    p = pickle.load(f)
    p = np.array(p)

with open('world_base_p_prime.pkl', 'rb') as f:
    p_prime = pickle.load(f)
    p_prime = np.array(p_prime)

R, T = compute_calibration(p, p_prime)
        # Create the homogeneous transformation matrix
H = np.eye(4)
H[:3, :3] = R
H[:3, 3] = T

print(R)
print(T)
# Transform robot points
transformed_robot = (R @ p.T).T + T
plot_points(p, p_prime, transformed_robot)
