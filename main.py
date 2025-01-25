import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
from utils import make_tf
from config import T_W_BOARD_CORNER, UR5E_BASE_POS, UR5E_BASE_QUAT
from robot import Robot
from scipy.spatial.transform import Rotation as R
import spatialmath as sm

def visualize_tf(transform, label="Transformation"):
    """
    Visualize a homogeneous transformation matrix.
    
    :param transform: A 4x4 homogeneous transformation matrix
    :param label: Label for the frame
    """
    # Extract origin and axes from the transformation matrix
    robot_base_pos = np.array(UR5E_BASE_POS)
    robot_base_ori = np.array(UR5E_BASE_QUAT)
    robot_base_ori = R.from_quat(robot_base_ori).as_matrix()
    print(robot_base_ori)
    robot_base_x_axis = robot_base_pos + robot_base_ori @[0.1, 0, 0] 
    robot_base_y_axis = robot_base_pos + robot_base_ori @[0, 0.1, 0] 
    robot_base_z_axis = robot_base_pos + robot_base_ori @[0, 0, 0.1] 

    table_origin = np.array([0, 0, 0])
    talbe_x_axis = table_origin + [0.2, 0, 0]   # Scale for visualization
    talbe_y_axis = table_origin + [0, 0.2, 0] 
    talbe_z_axis = table_origin + [0, 0, 0.2] 

    origin = transform.t
    print(origin, type(origin))
    rot = transform.R
    x_axis = origin + rot @[0.1, 0, 0]   # Scale for visualization
    y_axis = origin + rot @[0, 0.1, 0] 
    z_axis = origin + rot @[0, 0, 0.1] 

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(*robot_base_pos, color="k", label=label)
    ax.quiver(*robot_base_pos, *(robot_base_x_axis - robot_base_pos), color="r", label="X-axis")
    ax.quiver(*robot_base_pos, *(robot_base_y_axis - robot_base_pos), color="g", label="Y-axis")
    ax.quiver(*robot_base_pos, *(robot_base_z_axis - robot_base_pos), color="b", label="Z-axis")
    # Plot origin
    ax.scatter(*table_origin, color="k", label=label)

    # Plot axes
    ax.quiver(*table_origin, *(talbe_x_axis - table_origin), color="r", label="X-axis")
    ax.quiver(*table_origin, *(talbe_y_axis - table_origin), color="g", label="Y-axis")
    ax.quiver(*table_origin, *(talbe_z_axis - table_origin), color="b", label="Z-axis")

    # Plot origin
    ax.scatter(*origin, color="k", label=label)

    # Plot axes
    ax.quiver(*origin, *(x_axis - origin), color="r", label="X-axis")
    ax.quiver(*origin, *(y_axis - origin), color="g", label="Y-axis")
    ax.quiver(*origin, *(z_axis - origin), color="b", label="Z-axis")

    # Set labels and limits
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    # ax.legend()
    plt.title(f"Visualization of {label}")
    plt.show()


def main() -> None:
    robot = Robot("192.168.0.12", False)

    T_Tcp = make_tf(pos=robot.T_base_tcp.t, ori=robot.T_base_tcp.R)
    print("TW : ", T_Tcp)

    Target_from_w = np.array([0.539, -0.1417, T_Tcp.t[2]])
    Target_from_w_T = make_tf(pos=Target_from_w, ori=robot.T_base_tcp.R)
    print(Target_from_w_T)
    H_r = R.from_quat(UR5E_BASE_QUAT).as_matrix()
    print(H_r)
    Target_from_base = np.linalg.inv(H_r) @ (Target_from_w - np.array(UR5E_BASE_POS))
    print(Target_from_base)

    T_Tcp = make_tf(pos=Target_from_base, ori=robot.T_base_tcp.R)
    print("T goal : ", T_Tcp)
    robot.moveL(T_Tcp)

    # Visualize the transformation
    # visualize_tf(T_w_target, label="Target Transformation")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Program interrupted by user, shutting down.")
