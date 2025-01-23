import spatialmath as sm

from utils.rtb import make_tf

from .config import T_W_BOARD_CORNER
from .robot import Robot


def main() -> None:
    robot = Robot()

    T_w_target = make_tf(pos=T_W_BOARD_CORNER.t, ori = robot.T_base_tcp.R)

    robot.moveL( sm.SE3.Tz(0.2) @ robot.T_w_base.inv() @ T_w_target)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
    