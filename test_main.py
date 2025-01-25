import numpy as np

from config import UR5E_BASE_POS, UR5E_BASE_QUAT
from robot import Robot
from utils import make_tf, transform_w_to_base


def main() -> None:
    robot = Robot("192.168.0.12", False)
    Tcp_T = make_tf(pos=robot.T_base_tcp.t, ori=robot.T_base_tcp.R)
    print("Current TCP pos :\n", Tcp_T)

    target_pos_w = np.array([0.539, -0.1417, 0.5182])
    target_p_base = transform_w_to_base(target_pos_w, UR5E_BASE_POS, UR5E_BASE_QUAT)

    target_T = make_tf(pos=target_p_base, ori=robot.T_base_tcp.R)
    robot.moveL(target_T)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
