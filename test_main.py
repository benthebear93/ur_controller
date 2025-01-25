import numpy as np
from scipy.spatial.transform import Rotation as R

from config import UR5E_BASE_POS, UR5E_BASE_QUAT
from robot import Robot
from utils import make_tf


def transform_w_to_base(target_p_w: np.array):
    R_w_to_base = R.from_quat(UR5E_BASE_QUAT).as_matrix()
    target_p_base = np.linalg.inv(R_w_to_base) @ (target_p_w - np.array(UR5E_BASE_POS))
    return target_p_base


def main() -> None:
    robot = Robot("192.168.0.12", False)
    Tcp_T = make_tf(pos=robot.T_base_tcp.t, ori=robot.T_base_tcp.R)
    print("Current TCP pos :\n", Tcp_T)

    target_pos_w = np.array([0.539, -0.1417, 0.300])
    target_p_base = transform_w_to_base(target_pos_w)

    target_T = make_tf(pos=target_p_base, ori=robot.T_base_tcp.R)
    robot.moveL(target_T)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
