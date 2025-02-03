import numpy as np
import spatialmath as sm

from config import T_W_BASE, T_W_DEFAULT
from robot import Robot
from utils import make_tf


class AutomateCapex:
    def __init__(self, robot_ip: str, simulation: bool = False):
        self.robot = Robot(robot_ip, simulation)
        self.T_W_BASE = T_W_BASE
        self._init_pose = T_W_DEFAULT.t  # 비공개 속성으로 변경
        print("TCP : \n", self.robot.T_base_tcp)

    @property
    def init_pose(self):
        return self._init_pose

    def get_tcp_pose(self):
        return self.robot.T_base_tcp

    def pick_tube(self, tube_poses):
        """Pick up the tube at the given pose."""
        # TODO 1: CHECK CALIBRATION FIRST!!!

        # Always go to the initial position first.
        # self.robot.hande.move(0, 5, 10)
        self.move_to_target(self.init_pose, T_W_DEFAULT.R)
        # target_pos_w = np.array([0.7561, -0.4463, 0.272])
        # self.move_to_target(target_pos_w, T_W_S1.R)
        # self.robot.moveL(T_BASE_S1)
        # # target_pos_w = np.array([0.700, -0.175, 0.400])
        # self.move_to_target(tube_poses[3])
        # self.robot.hande.move(120, 1, 10)
        # tube_d_up = np.array([0.750, -0.225, 0.250])
        # time.sleep(2)
        # self.move_to_target(tube_d_up)
        pass

    def move_to_target(self, target_pos_w: np.array, rot: sm.SO3 = None):
        """Move the robot to the target position in world coordinates."""
        T_W_Tcp = self.T_W_BASE @ self.get_tcp_pose()

        # TODO 2: CHECK T_W_Tcp and put R in to T_W_DEFAULT.
        print("Tcp T in world :\n", T_W_Tcp)
        if rot is None:
            rot = T_W_Tcp.R

        T_W_Target = make_tf(pos=target_pos_w, ori=rot)
        print("Target T in world :\n", T_W_Target)
        T_BASE_Target = self.T_W_BASE.inv() @ T_W_Target
        print("Target T in base :\n", T_BASE_Target)
        # self.robot.moveL(T_BASE_Target)

    def run(self):
        """Main execution function."""
        tube_a = np.array([0.700, 0.175, 0.220])
        tube_b = np.array([0.700, 0.225, 0.220])
        tube_c = np.array([0.750, 0.175, 0.220])
        tube_d = np.array([0.750, 0.225, 0.135])
        tube_poses = [tube_a, tube_b, tube_c, tube_d]
        self.pick_tube(tube_poses)


if __name__ == "__main__":
    automate = AutomateCapex("192.168.0.12", True)
    try:
        automate.run()
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
