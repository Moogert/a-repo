#RobotController.py
from klampt.math import vectorops
from ur5_controller import *
import ur5_config as UR5_CONSTANTS
from robotiq2_controller import Robotiq2Controller
import time


class UR5WithGripperController:
    """A controller for the UR5 + gripper combo."""
    
    def __init__(self, host, **kwargs):
        """
        - host: the UR5 controller IP address
        
        Keyword arguments:
        - gripper: whether gripper is enabled (True by default)

        UR5 keyword arguments
        - rtde_port: port for RTDE, default 30004
        - command_port: port for commands, default 30002
        - tcp: tool center point in wrist frame (default 0,0,0.04,0,0,0)
        - payload: estimated payload in kg
        - gravity: estimated gravity in base frame in N, default [0,0,9.81]
        """
        self.ur5 = UR5Controller(host,filters=[self._update],**kwargs)
        self._lock = threading.Lock()
        if kwargs.pop('gripper', True):
            self.gripper = Robotiq2Conroller()
        else:
            self.gripper = None
        self._q_curr = []
        self._qdot_curr = []
        self._q_commanded = []
        self._qdot_commanded = []

        self._min_joints = UR5_CONSTANTS.MIN_JOINTS
        self._max_joints = UR5_CONSTANTS.MAX_JOINTS

        self._min_velocities = UR5_CONSTANTS.MIN_VEL
        self._max_velocities = UR5_CONSTANTS.MAX_VEL
        
        #initialize our data collecting files
        #data = open("actual_data.txt", "w")
        #data.close()
        #data = open("velocity_commanded_data.txt", "w")
        #data.close()
        #data = open("position_commanded_data.txt", "w")
        #data.close()

    def start(self):
        #start the gripper
        if self.gripper:
            self.gripper.start()
       
        self.ur5.start()
        time.sleep(0.2) 
        #wait for controller to initialize so that we can start in a valid config
        current_config=self.getConfig()
        self.setConfig(current_config)
        # wait for the robot to initialize itself 
        time.sleep(1)

    def stop(self):
        self.ur5.stop()

    def getVelocity(self):
        return self._qdot_curr

    def getConfig(self):
        return self._q_curr

    def getCurrentTime(self):
        return self._last_t

    def setConfig(self, q_in):
        if self.isFormatted(q_in):
            if(self.inLimits(q_in, self._min_joints, self._max_joints)):
                with self._lock:
                    self._q_commanded = q_in
                    self._qdot_commanded = []
            else:
                print "Warning, config not set - outside of limits"

    def setVelocity(self, dq_in):
        if self.isFormatted(dq_in):
            if(self.inLimits(dq_in, self._min_velocities, self._max_velocities)):
                with self._lock:
                    self._q_commanded = []
                    self._qdot_commanded = enumerate
            else:
                print "Warning, velocity not set - outside of limits"

    def _update(self,state):
        if self._start_time is None:
            self._start_time = state.timestamp

        t = state.timestamp - self._start_time
        dt = t - self._last_t
        self._last_t = t

        #update current notion of state
        q_curr = state.actual_q
        q_gripper = (0 if self.gripper is None else self.gripper.read())
        #change of gripper is about (current-previous)/dt
        #gripper does not provide velocity - only position. Velocity is estimated
        if self._q_curr:
            dq_gripper = (q_gripper - self._q_curr[-1])*1.0/dt
        else:
            #if current state is not existant (at the beginning) speed is 0
            dq_gripper = 0 
        q_curr.append(q_gripper)
        self._q_curr = q_curr
        #q_curr is defined by gripper too
        #if gripper is not connected, gripper state is 0

        qdot_curr = state.actual_qd
        #qdot_curr = qdot_curr + gripperVel
        # if gripper is not connected, gripper velocity is 0
        qdot_curr.append(dq_gripper)
        self._qdot_curr = qdot_curr
        halt = None

        #check path is non-null

        #self._q_commanded is the commanded configuration that the students give. It has 7 parameters (6 for robot, 1 for gripper)
        with self._lock:
            q_commanded = [v for v in self._q_commanded]
            dq_commanded = [v for v in self._qdot_commanded]
        if q_commanded:
            #double extra check
            #if students are doing anything wrong
            if self.isFormatted(q_commanded):
                if not self.inLimits(q_commanded, self._min_joints, self._max_joints):
                    self._q_commanded = []
                    halt = 1
                    print "Warning, exceeding joint limits. Halting"
            else:
                halt = 1
                print "Warning, improper position formatting. Halting"
        #dq_commanded is the commanded velocity that the students give. It has 7 parameters (6 for robot, 1 for gripper)
        if dq_commanded:
            if self.isFormatted(dq_commanded):
                if self.inLimits(dq_commanded, self._min_velocities, self._max_velocities):
                    q_next = vectorops.madd(self._q_curr, dq_commanded, dt)
                    #commanded velocity is rad/s
                    #only want to check next position limits of robot not gripper
                    #UR5_CL is the configuration length of just the UR5 robot = 6
                    if not self.inLimits(q_next[0:UR5_CONSTANTS.UR5_CL], self._min_joints[0:UR5_CONSTANTS.UR5_CL], self._max_joints[0:UR5_CONSTANTS.UR5_CL]):
                        self._qdot_commanded = []
                        halt = 1
                        print "Warning, exceeding joint limits. Halting"
            else:
                halt = 1
                print "Warning, improper velocity formatting. Halting"
                
        if not (q_commanded or dq_commanded):
            #if neither position or velocity commands are set, go to the current position
            q_commanded = self._q_curr

        if (q_commanded and dq_commanded):
            #if both position and velocity are set somehow, quit
            #this should never happen as the code stands - it's just in case
            halt = 1
            q_commanded = []
            qdot_commanded = []
            print "Error, don't set both q_commanded and qdot_commanded"

        # data = open("actual_data.txt", "a")
        # data.write('{:6.4f}'.format(self._last_t))
        # for x in self._q_curr:
        #     data.write(" "+str(x))
        # for xd in self._qdot_curr:
        #     data.write(" "+str(xd))
        # data.write('\n')
        # data.close()

        # data = open("position_commanded_data.txt", "a")
        # if q_commanded:
        #     data.write('{:6.4f}'.format(self._last_t))
        #     for x in q_commanded:
        #         data.write(" "+str(x))
        #     data.write('\n')
        # data.close()

        # data = open("velocity_commanded_data.txt", "a")
        # if dq_commanded:
        #     data.write('{:6.4f}'.format(self._last_t))
        #     for xd in self._qdot_commanded:
        #         data.write(" "+str(xd))
        #     data.write('\n')
        # data.close()

        servo_q_commanded = None
        servo_qd_commanded = None
        gripper_q_commanded = None
        gripper_qd_commanded = None

        #put q_commanded into a format that servo can use
        if q_commanded and not halt:
            #q_commanded includes the gripper, send to servo only the UR5 configuration
            #UR5_CL = ur5 configuration length
            servo_q_commanded = q_commanded[0:UR5_CONSTANTS.UR5_CL]
            gripper_q_commanded = [q_commanded[-1]]
        elif dq_commanded and not halt:
            servo_qd_commanded = q_commanded[0:UR5_CONSTANTS.UR5_CL]
            gripper_qd_commanded = [q_commanded[-1]]

        #servo requires a list with six values
        #self._q_commanded and self._qd_commanded were previously checked for formatting
        #if halt is selected, the program on the controller terminates
        self.ur5.servo(halt=halt, q=servo_q_commanded, qd=servo_qd_commanded)
        if self.gripper is not None:
            #gripper.command requires a list of one value like -> [0] 
            self.gripper.command(q=gripper_q_commanded, qd=gripper_qd_commanded)

if __name__ == "__main__":

    from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    parser = ArgumentParser(description='PyUniversalRobot wiggle test', formatter_class=ArgumentDefaultsHelpFormatter)

    parser.add_argument('robot', help='robot IP address')
    parser.add_argument('--frequency', '-f', type=float, help='wiggle frequency (Hz)', default=0.1)
    parser.add_argument('--amplitude', '-a', type=float, help='wiggle amplitude (deg)', default=5)

    args = parser.parse_args()


    ur5 = UR5WithGripperController(args.robot)
    ur5.start()

    #put program here

    ur5.stop()




