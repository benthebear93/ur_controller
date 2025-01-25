import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
from utils import make_tf
from config import T_W_BOARD_CORNER, UR5E_BASE_POS, UR5E_BASE_QUAT
from robot import Robot
from scipy.spatial.transform import Rotation as R
import spatialmath as sm

def transform_w_to_base(target_p_w: np.array):
    R_w_to_base = R.from_quat(UR5E_BASE_QUAT).as_matrix()
    target_p_base = np.linalg.inv(R_w_to_base) @ (target_p_w - np.array(UR5E_BASE_POS))
    return target_p_base

def main() -> None:
    robot = Robot("192.168.0.12", False)
    Tcp_T = make_tf(pos=robot.T_base_tcp.t, ori=robot.T_base_tcp.R)
    print("Current TCP pos :", Tcp_T)

    target_pos_w = np.array([0.539, -0.1417, Tcp_T.t[2]])
    target_p_base = transform_w_to_base(target_pos_w)

    target_T = make_tf(pos=target_p_base, ori=robot.T_base_tcp.R)
    robot.moveL(target_T)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
