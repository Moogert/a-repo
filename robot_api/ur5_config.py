#ur5_config.py

import math

MAX_JOINTS = [2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 2*math.pi, 1]
MIN_JOINTS = [-2*math.pi, -2*math.pi, -2*math.pi, -2*math.pi, -2*math.pi, -2*math.pi, 0]
MAX_VEL = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi, 1]
MIN_VEL = [-math.pi, -math.pi, -math.pi, -math.pi, -math.pi, -math.pi, -1]

#number of elements in a configuration
ROBOT_CONFIG_LEN =7
ROB_CL = ROBOT_CONFIG_LEN
UR5_CONFIG_LEN=6
UR5_CL = UR5_CONFIG_LEN

#time for robot to pause as gripper closes
#not currently used
DEFAULT_GRIPPER_DELAY=0

