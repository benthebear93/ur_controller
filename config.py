from utils import make_tf

# UR5E_BASE_POS = [
#     0.22331901,
#     0.37537452,
#     0.08791326,
# ]  # from hand to table calibration

# UR5E_BASE_QUAT = [
#     -0.19858483999999996,
#     -0.00311175,
#     0.0012299899999999998,
#     0.98007799,
# ]  # from hand to table calibration

# T_W_BOARD_CORNER = make_tf(
#     pos=[0.70162832, 0.63506023, 0.05866477], ori=[0.7071, 0, 0, -0.7071]
# )
UR5E_BASE_POS = [
    0.1246641456,
    -0.37559191738,
    0.04129487687,
]  # from hand to table calibration

UR5E_BASE_QUAT = [1.13897542e-03, 
                  2.64816757e-04,
                  3.82421349e-01, 
                  9.23987307e-01,
                  ]# from hand to table calibration [ 0.0006181, 0.0009926, 0.9237704, 0.382945 ]

T_W_BOARD_CORNER = make_tf(
    pos=[0.375591917, 0.124541456, 0.0412948769], ori=[ 0.0006181, 0.0009926, 0.9237704, 0.382945 ]
)
ROBOTIQ_PORT = 63352
ROBOT_IP = "192.168.1.125"
_MAX_GRIPPER_FORCE = 255
_MAX_GRIPPER_SPEED = 255
_MAX_GRIPPER_POS = 255

_DEFAULT_GRIPPER_SPEED = int(_MAX_GRIPPER_SPEED * 0.02)
_DEFAULT_GRIPPER_FORCE = int(_MAX_GRIPPER_FORCE * 0.02)
_DEFAULT_GRIPPER_POS   = int(_MAX_GRIPPER_FORCE * 0.02)