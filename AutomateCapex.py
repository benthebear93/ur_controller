import time

import numpy as np
import spatialmath as sm

from config import T_W_BASE, T_W_DEFAULT, T_W_S1
from robot import Robot
from utils import make_tf


class AutomateCapex:
    def __init__(self, robot_ip: str, simulation: bool = False):
        self.robot = Robot(robot_ip, simulation)
        self.T_W_BASE = T_W_BASE
        self._init_pose = T_W_DEFAULT.t  # 비공개 속성으로 변경
        self.wrench_data = self.robot.recv.getActualTCPForce()
        print("TCP : \n", self.robot.T_base_tcp)
        self.wrench_thrs = 0
        self.contact_pos = 0

    @property
    def init_pose(self):
        return self._init_pose

    def get_tcp_pose(self):
        return self.robot.T_base_tcp

    def pick_tube(self, tube_poses):
        """Pick up the tube at the given pose."""
        self.move_to_target(tube_poses[3])
        self.robot.hande.move(225, 1, 10)
        tube_d_up = np.array([0.750, -0.225, 0.250])
        time.sleep(2)
        self.move_to_target(tube_d_up)
        self.move_to_target(T_W_S1.t, T_W_S1)
        pass

    def push_it_in(self):
        print("push it in")
        # goal = T_W_S1 @ sm.SE3.Tz(0.0175)
        goal_check = 0
        x_check = 1
        y_check = 1
        xy_flag = 0
        spiral = 0
        decay_factor = 0.9  # 점점 작아지는 정도
        spiral_factor = 0.002  # 이동 크기
        while True:
            self.wrench_thrs = np.linalg.norm(self.wrench_data)
            print(self.wrench_thrs)
            self.wrench_data = self.robot.recv.getActualTCPForce()
            print(self.wrench_data)
            print("moved :", 0.0005*goal_check)
            if self.wrench_thrs > 4.0:
                x_check = -1*x_check
                y_check = -1*y_check
                spiral +=1
                r = 0.002 * np.exp(-0.1 * spiral)  # 점점 작아지는 반지름
                theta = spiral * np.pi / 6  # 회전 각도

                x_offset = r * np.cos(theta)
                y_offset = r * np.sin(theta)
                print('x :', x_offset, "y :", y_offset)
                if xy_flag == 0:
                    print("X !")
                    goal =  sm.SE3.Tx(0.002*x_check +x_offset) @ T_W_S1 @ sm.SE3.Tz(0.0005*goal_check-0.0005)
                    self.move_to_target(goal.t, goal, 0.002)
                    goal =  sm.SE3.Tx(0.002*x_check+x_offset) @ T_W_S1 @ sm.SE3.Tz(0.0005*goal_check)
                    self.move_to_target(goal.t, goal, 0.002)
                    xy_flag = 1
                    time.sleep(0.1)
                else:
                    print("Y !")
                    goal = sm.SE3.Ty(0.002*y_check+y_offset) @ T_W_S1 @ sm.SE3.Tz(0.0005*goal_check-0.0005) 
                    self.move_to_target(goal.t, goal, 0.002)
                    goal =  sm.SE3.Ty(0.002*y_check+y_offset) @ T_W_S1 @ sm.SE3.Tz(0.0005*goal_check)
                    self.move_to_target(goal.t, goal, 0.002)
                    xy_flag = 0
                    time.sleep(0.1)
            else:
                goal_check +=1
                goal = T_W_S1 @ sm.SE3.Tz(0.0005*goal_check)
                self.contact_pos = goal
                self.move_to_target(goal.t, goal, 0.0025)
                time.sleep(0.1)
                if goal_check > 175/5:
                    break
        self.robot.hande.move(0, 1, 10)

    def move_to_target(self, target_pos_w: np.array, rot: sm.SO3 = None, vel:int = None):
        """Move the robot to the target position in world coordinates."""
        T_W_Tcp = self.T_W_BASE @ self.get_tcp_pose()

        print("Tcp T in world :\n", T_W_Tcp)
        if rot is None:
            rot = T_W_DEFAULT.R
        if vel is None:
            vel = 0.1

        T_W_Target = make_tf(pos=target_pos_w, ori=rot)
        # print("Target T in world :\n", T_W_Target)
        T_BASE_Target = self.T_W_BASE.inv() @ T_W_Target
        # print("Target T in base :\n", T_BASE_Target)
        self.robot.moveL(T_BASE_Target, vel)

    def run(self):
        # Always go to the initial position first.
        self.robot.hande.move(0, 5, 10)
        self.move_to_target(self.init_pose, T_W_DEFAULT.R)
        """Main execution function."""
        tube_a = np.array([0.700, -0.175, 0.220])
        tube_b = np.array([0.700, -0.225, 0.220])
        tube_c = np.array([0.750, -0.175, 0.220])
        tube_d = np.array([0.750, -0.225, 0.120])
        tube_poses = [tube_a, tube_b, tube_c, tube_d]
        self.pick_tube(tube_poses)
        self.push_it_in()


if __name__ == "__main__":
    automate = AutomateCapex("192.168.0.12", True)
    try:
        automate.run()
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
