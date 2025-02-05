import time

import numpy as np
import spatialmath as sm

from config import T_W_BASE, T_W_DEFAULT, T_W_S1, T_W_S2
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
        self.slot_pos = [T_W_S1, T_W_S2]
        self.occupy_check = [0, 0, 0]
        self.got_in = False

    @property
    def init_pose(self):
        return self._init_pose

    def get_tcp_pose(self):
        return self.robot.T_base_tcp

    def pick_tube(self, tube_poses, slot_num):
        """Pick up the tube at the given pose."""
        self.move_to_target(T_W_DEFAULT)
        self.move_to_target(tube_poses)
        self.robot.hande.move(225, 1, 255)
        tube_d_up = np.array([tube_poses[0], tube_poses[1], tube_poses[2]+0.120])
        time.sleep(2)
        self.move_to_target(tube_d_up)
        self.move_to_target(self.slot_pos[slot_num].t, self.slot_pos[slot_num])
        pass

    def putback_tube(self, tube_poses):
        self.move_to_target(T_W_DEFAULT)
        tube_pose_above = np.array([tube_poses[0], tube_poses[1], tube_poses[2]+0.120])
        self.move_to_target(tube_pose_above)
        self.move_to_target(tube_poses)
        self.robot.hande.move(0, 1, 255)
        time.sleep(3)
        self.move_to_target(tube_pose_above)
        
    def wire_insert(self):
        T_W_Tcp = self.T_W_BASE @ self.get_tcp_pose()
        T_W_Tcp.t[2] = T_W_Tcp.t[2]
        self.move_to_target(T_W_Tcp)

    def push_it_in(self, slot_num, insert_distnace):
        print("push it in")
        depth_step = 0
        dirx = [1, 1, -1, -1]
        diry = [1, -1, -1, 1]
        max_z_dist = insert_distnace
        empty_hole_check = [0, 0]
        while True:
            self.wrench_data = self.robot.recv.getActualTCPForce()
            Fx = self.wrench_data[0]
            Fy = self.wrench_data[1]
            Fz = self.wrench_data[2] 
            print(f"F : {self.wrench_data[0]:.3f}, {self.wrench_data[1]:.3f}, {self.wrench_data[2]:.3f} :")
            print(f"GOAL CHECK ({depth_step}/38) :")
            print(" ")
            if abs(Fz) > 5.5 and depth_step < 35:
                if abs(Fx) > 2.0 or abs(Fy) > 2.0 or abs(Fz) > 15:
                    print("Fixing it")
                    for i in range(0, 4):
                        goal =  sm.SE3.Tx(0.002*dirx[i]) @ self.slot_pos[slot_num] @ sm.SE3.Tz(0.0005*depth_step-0.001)
                        self.move_to_target(goal.t, goal, 0.002)
                        time.sleep(0.1)

                        goal =  sm.SE3.Tx(0.002*dirx[i]) @ self.slot_pos[slot_num] @ sm.SE3.Tz(0.0005*depth_step)
                        self.move_to_target(goal.t, goal, 0.002)
                        time.sleep(0.1)

                        self.wrench_data = self.robot.recv.getActualTCPForce()
                        print(f"Fxing : {self.wrench_data[0]:.3f}, {self.wrench_data[1]:.3f}, {self.wrench_data[2]:.3f} :")
                        # if abs(self.wrench_data[0]) < 2.0 and abs(self.wrench_data[1]) < 2.0  and abs(self.wrench_data[2] < 20):
                        #     goal = sm.SE3.Ty(0.004) @ self.slot_pos[slot_num] @ sm.SE3.Tz(0.0005*(depth_step+1))
                        #     self.move_to_target(goal.t, goal, 0.002)
                        #     time.sleep(0.1)

                        #     self.wrench_data = self.robot.recv.getActualTCPForce()
                        #     print(f"Empty hole check y: {self.wrench_data[0]:.3f}, {self.wrench_data[1]:.3f}, {self.wrench_data[2]:.3f} :")
                        #     if abs(self.wrench_data[0]) > 1.0:
                        #         empty_hole_check[0] = 1
                        #     goal = sm.SE3.Tx(0.004) @ self.slot_pos[slot_num] @ sm.SE3.Tz(0.0005*(depth_step+1))
                        #     self.move_to_target(goal.t, goal, 0.002)
                        #     time.sleep(0.1)

                        #     self.wrench_data = self.robot.recv.getActualTCPForce()
                        #     print(f"Empty hole check x: {self.wrench_data[0]:.3f}, {self.wrench_data[1]:.3f}, {self.wrench_data[2]:.3f} :")
                        #     if abs(self.wrench_data[1]) > 1.0:
                        #         empty_hole_check[1] = 1
                        #     if empty_hole_check[0] + empty_hole_check[1] == 2:
                        #         break
                        #     else:
                        #         if self.wrench_data[2] > 4:
                        #             self.got_in = True
                        #             break
                        #         print("You are falling into empty area, go back to ", empty_hole_check)
                        #         empty_hole_check = [0, 0]
                        goal = sm.SE3.Ty(0.002*diry[i]) @ self.slot_pos[slot_num] @ sm.SE3.Tz(0.0005*depth_step-0.001) 
                        self.move_to_target(goal.t, goal, 0.002)
                        time.sleep(0.1)

                        goal =  sm.SE3.Ty(0.002*diry[i]) @ self.slot_pos[slot_num] @ sm.SE3.Tz(0.0005*depth_step)
                        self.move_to_target(goal.t, goal, 0.002)
                        time.sleep(0.1)

                        self.wrench_data = self.robot.recv.getActualTCPForce()
                        print(f"Fxing 2: {self.wrench_data[0]:.3f}, {self.wrench_data[1]:.3f}, {self.wrench_data[2]:.3f} :")
                        # if abs(self.wrench_data[0]) < 2.0 and abs(self.wrench_data[1]) < 2.0 and abs(self.wrench_data[2] < 20):
                        #     goal = sm.SE3.Ty(0.004) @ self.slot_pos[slot_num] @ sm.SE3.Tz(0.0005*(depth_step+1))
                        #     self.move_to_target(goal.t, goal, 0.002)
                        #     time.sleep(0.1)

                        #     self.wrench_data = self.robot.recv.getActualTCPForce()
                        #     print(f"Empty hole check y: {self.wrench_data[0]:.3f}, {self.wrench_data[1]:.3f}, {self.wrench_data[2]:.3f} :")
                        #     if abs(self.wrench_data[1]) > 1.0:
                        #         empty_hole_check[1] = 1
                        #     goal = sm.SE3.Tx(0.004) @ self.slot_pos[slot_num] @ sm.SE3.Tz(0.0005*(depth_step+1))
                        #     self.move_to_target(goal.t, goal, 0.002)
                        #     time.sleep(0.1)
                            
                        #     self.wrench_data = self.robot.recv.getActualTCPForce()
                        #     print(f"Empty hole check x: {self.wrench_data[0]:.3f}, {self.wrench_data[1]:.3f}, {self.wrench_data[2]:.3f} :")
                        #     if abs(self.wrench_data[0]) > 1.0:
                        #         empty_hole_check[0] = 1
                        #     if empty_hole_check[0] + empty_hole_check[1] == 2:
                        #         break
                        #     else:
                        #         if self.wrench_data[2] > 4:
                        #             self.got_in = True
                        #             break
                        #         print("You are falling into empty area, go back to ", empty_hole_check)
                        #         empty_hole_check = [0, 0]
                else:
                    depth_step +=1
                    goal = self.slot_pos[slot_num] @ sm.SE3.Tz(0.0005*depth_step)
                    self.move_to_target(goal.t, goal, 0.0025)
                    time.sleep(0.1)
            else:
                depth_step +=1
                goal = self.slot_pos[slot_num] @ sm.SE3.Tz(0.0005*depth_step)
                self.move_to_target(goal.t, goal, 0.0025)
                time.sleep(0.1)
            if depth_step >= max_z_dist:
                break
        self.robot.hande.move(0, 1, 10)
        self.occupy_check[slot_num] = 1
        time.sleep(2)

    def move_to_target(self, target_pos_w: np.array, rot: sm.SO3 = None, vel:int = None):
        """Move the robot to the target position in world coordinates."""
        T_W_Tcp = self.T_W_BASE @ self.get_tcp_pose()

        # print("Tcp T in world :\n", T_W_Tcp)
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
        tube_a = np.array([0.700, -0.175, 0.120])
        tube_b = np.array([0.700, -0.225, 0.220])
        tube_c = np.array([0.750, -0.175, 0.220])
        tube_d = np.array([0.750, -0.225, 0.120])
        tube_poses = [tube_a, tube_b, tube_c, tube_d]
        self.pick_tube(tube_poses[3], 0)
        self.push_it_in(0, 35)

        T_W_Tcp = self.T_W_BASE @ self.get_tcp_pose()
        T_W_move_out = T_W_Tcp @ sm.SE3.Tz(-0.08)
        print("MOVE OUT : \n", T_W_move_out)
        self.move_to_target(T_W_move_out, T_W_move_out.R)
        self.pick_tube(tube_poses[0], 1)
        self.push_it_in(1, 32)

        self.robot.hande.move(225, 1, 255)
        time.sleep(3)
        T_W_Tcp = self.T_W_BASE @ self.get_tcp_pose()
        T_W_move_out = T_W_Tcp @ sm.SE3.Tz(-0.08)
        print("MOVE OUT : \n", T_W_move_out)
        self.move_to_target(T_W_move_out, T_W_move_out.R)
        self.putback_tube(tube_poses[0])

        self.move_to_target(self.slot_pos[0].t, self.slot_pos[0])
        get_from_slot1 = self.slot_pos[0] @ sm.SE3.Tz(0.0005*35)
        self.move_to_target(get_from_slot1.t, get_from_slot1, 0.006)
        self.robot.hande.move(225, 1, 255)
        time.sleep(3)
        T_W_Tcp = self.T_W_BASE @ self.get_tcp_pose()
        T_W_move_out = T_W_Tcp @ sm.SE3.Tz(-0.08)
        print("MOVE OUT : \n", T_W_move_out)
        self.move_to_target(T_W_move_out, T_W_move_out.R)
        self.putback_tube(tube_poses[3])
        


if __name__ == "__main__":
    automate = AutomateCapex("192.168.0.12", True)
    try:
        automate.run()
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
