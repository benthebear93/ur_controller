import numpy as np

from config import T_W_BASE
from robot import Robot
from utils import make_tf

# def transform_w_to_base(target_p_w: np.array):
#     R_w_to_base = R.from_quat(UR5E_BASE_QUAT).as_matrix()
#     target_p_base = np.linalg.inv(R_w_to_base) @ (target_p_w - np.array(UR5E_BASE_POS))
#     # target_R_base = R.from_quat(UR5E_BASE_QUAT).as_matrix() @ R_w_to_base
#     return target_p_base


def main(robot) -> None:
    Tcp_T = robot.T_base_tcp
    print("Current TCP pos :\n", Tcp_T)
    target_pos_w = np.array([0.700, -0.175, 0.125])
    T_W_Tcp = T_W_BASE @ robot.T_base_tcp
    T_W_Target = make_tf(pos=target_pos_w, ori=T_W_Tcp.R)
    T_BASE_Target = T_W_BASE.inv() @ T_W_Target
    robot.moveL(T_BASE_Target)


if __name__ == "__main__":
    robot = Robot("192.168.0.12", False)
    Tcp_T = robot.T_base_tcp
    print("Current TCP pos :\n", Tcp_T)
    try:
        main(robot)
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
