from klampt import *
import math
import threading
import random

ur5_indices = [1,2,3,4,5,6]
ee_driver=6


# need to change this
#ee_driver_open_value=0.0
#ee_driver_close_value=0.723

def set_robot_ur5_config(q_robot,q_ur5):
    """Sets the Klamp't robot config q_robot to have the 6-element UR5 configuration q_ur5"""
    assert len(q_robot) > 6
    assert len(q_ur5) == len(ur5_indices)
    for (i,v) in zip(ur5_indices,q_ur5):
        q_robot[i] = v

def get_robot_ur5_config(q_robot):
    """Returns the 6-element UR5 configuration of the Klamp't robot config"""
    assert len(q_robot) > 6
    return [q_robot[i] for i in ur5_indices]

class UR5WithGripperController(object):
    """Emulates the UR5 + gripper control system using a Klamp't physics simulator.
    
    The configuration is a 6+1 DOF configuration containing the UR5 joint angles + the gripper value.
    """
    
    def __init__(self, world, randomize=False, dt=0.008):
        """Arguments:
        - world: must be a WorldModel containing a UR5 + gripper.  This will be updated with the simulation,
        so if you have a planning world model it's best to make this a copy of the planner's world model.
        - randomize (optional): if True, sets the initial configuration of the robot to a somewhat
          random configuration.  This tests whether the application program can accommodate a diverse
          set of initial conditions.
        - dt (optional): overrides the controller time step (default 125 Hz).
        """
        #initialize world
        self.world = world
        self.current_time = 0
        self.dt=0.008
        #used for threading
        self.lock = threading.Lock()
        self.thread = None
        
        # set initial config, UR5 + Gripper
        self.q=[0,-math.pi/2,0,-math.pi/2,0,0,0]
        self.dq=[0]*7
        
        #initialize sensed UR5 + gripper config (from encoders)
        self.last_q_sensed=[0,1.5*math.pi,0,1.5*math.pi,0,0,0]
        #initialize command configuration
        self.qcmd = None
        
        if randomize:
            for i in range(6):
                v = random.uniform(-1,1)
                self.q[i] += v
                self.last_q_sensed[i] += v
            v = random.uniform(0.2,0.8)
            self.q[6] = v
            self.last_q_sensed[6] = v
        
        #This is for handling the issue where simulated encoders in range [0,2pi)
        
        self.robot = self.world.robot(0)                     
        q_start = self.q[0:6] # UR5 start config
        q_init = self.robot.getConfig()
        set_robot_ur5_config(q_init,q_start)
        self.robot.setConfig(q_init)

        #set Gripper start config
        self.robot.driver(ee_driver).setValue(self.q[6])
        
        #create controller
        self._sim = Simulator(self.world)
        self._controller = self._sim.controller(0)
        self._controller.setRate(self.dt)

    def __del__(self):
        if self.thread is not None:
            stop_controller_thread(self)
        
    def getConfig(self):
        """Returns current 6+1 DOF configuration of the robot"""
        return [v for v in self.q]
    
    def getVelocity(self):
        """Returns current 6+1 DOF velocity of the robot"""
        return [v for v in self.dq]
        
    def getCurrentTime(self):
        """Returns the current time of the controller"""
        return self.current_time               
    
    def setConfig(self, q):
        """Sets a 6+1DOF position command"""
        assert len(q)==7,"Commanded configuration must be length 7"
        with self.lock: 
            #TODO: you may want to do something here
            self.qcmd = [v for v in q]
    
    def setVelocity(self, dq):
        """Not implemented."""
        raise NotImplementedError("Although the UR5+gripper interface lets you set the velocity, we aren't using that")
    
    def _sensed2actual(self):
        #Internally used to read the simulated sensors.
        
        # function to convert from sensed positions to actual positions
        #print self.region
        qklampt = self._controller.getSensedConfig()
        sensed_q = get_robot_ur5_config(qklampt)#UR5
        for i in range(6):
            d = sensed_q[i] - self.last_q_sensed[i]
            if d > math.pi:
                d -= math.pi*2
            elif d < -math.pi:
                d += math.pi*2
            self.q[i]+=d
            self.dq[i] = d/self.dt
        self.last_q_sensed=sensed_q
            
        #gripper
        q_g=self.robot.driver(ee_driver).getValue()#set Gripper
        self.dq[6] = (q_g - self.q[6])/self.dt
        self.q[6]=q_g 
        
        #TODO: do we use differencing or the "sensors"
        self.dq = self._controller.getSensedVelocity()[1:7]+[self.robot.driver(ee_driver).getVelocity()]
    
    def _outputPID(self):
        #Internally used to command the simulated robot
        
        #TODO: complete me.
        # You have access to a Klamp't controller, which accepts a PID command including the desired q and dq.
        # Inside the simulator it computes torques to drive the robot.
        # Your job is to figure out how to adjust the commands that 
        # the resulted trajectory matches the recorded trajectory from the actual robot.
        # The current code simply sets q_des=qcmd and dq_des equal to 0. 
        
        if self.qcmd is not None:
            #the user gave some command
            dq_des = [0]*len(self.qcmd)
            q_des=self.qcmd
            self._controller.setPIDCommand(q_des, dq_des)
        else:
            pass
        
        
    def advance(self):
        """Used ONLY in manual simulation. Advances simulation time by dt.
        By default this acts at the actual controller's time step, 125 Hz.
        
        The world model will be updated after this call.
        """
        # Update the robot's configuration from sensors
        self._sensed2actual()
        self._outputPID()
        self._sim.simulate(self.dt)
        self.current_time = self.current_time + self.dt

    def resetFromWorld(self):
        """Used ONLY in manual simulation.  Lets you reset the state of the simulation
        and controller. The robot's current configuration is re-read from the world model.  
        
        Note: this does not read the current velocities, everything will be stopped.
        """
        q = self.robot.getConfig()
        self.q = get_robot_ur5_config(q) + [self.robot.driver(ee_driver).getValue()]
        self.last_q_sensed = [v for v in self.q]
        self.dq = [0]*7
                
        #re-create simulation and controller
        self._sim = Simulator(self.world)
        self._controller = self._sim.controller(0)
        self.qcmd = None

    def start(self,callback=None):
        """Begins a thread for operating in the background."""
        start_controller_thread(self,callback)

    def stop(self):
        """Stops a thread for operating in the background"""
        stop_controller_thread(self)

ACTIVE_EMULATION_THREADS = []
        
def run_ur5gripper_thread(apiref,callback):
    global ACTIVE_EMULATION_THREADS
    ACTIVE_EMULATION_THREADS.append(apiref)
    print "UR5GripperAPI thread #%d starting"%(len(ACTIVE_EMULATION_THREADS),)
    import time
    t0 = time.time()
    while apiref():
        apiobj = apiref()
        if apiobj.threadStop:
            break
        t0 = time.time()
        with apiobj.lock:
            apiobj.advance()
            if callback:
                callback()
        t1 = time.time()
        time.sleep(max(0,apiobj.dt-(t1-t0)))
    print "UR5GripperAPI thread terminating"

def kill_controller_threads():
    global ACTIVE_EMULATION_THREADS
    for i in ACTIVE_EMULATION_THREADS:
        if i() is not None:
            stop_controller_thread(i())
    ACTIVE_EMULATION_THREADS = []
    
def start_controller_thread(emulator,*args):
    import weakref
    if emulator.thread is not None:
        print "UR5WithGripperController emulation thread already started"
        return
    print "Starting UR5WithGripperController emulation thread..."
    emulator.threadStop = False
    thread = threading.Thread(target=run_ur5gripper_thread,args=(weakref.ref(emulator),)+args)
    thread.daemon = True
    thread.start()
    emulator.thread = thread

def stop_controller_thread(emulator):
    if emulator.thread is None:
        print "UR5WithGripperController emulation thread not started"
        return
    print "Stopping UR5WithGripperController emulation thread..."
    emulator.threadStop = True
    emulator.thread.join()
    emulator.thread = None
    print "Stopped."