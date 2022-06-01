# Defining the Controller interface.
import numpy as np
from morse_simulator.algorithms.dataStreamReader import *
import socket
from morse_simulator.algorithms.serialization import get_server_port
from morse_simulator.algorithms.Pose import map_t
from morse_simulator.algorithms.config import config
import time

class controllerInterface:
    def __init__(self):
        self.controller = config.controller
        self.map_transform = map_t
        self.robot_number = 0
        self.port = get_server_port(self.robot_number)
        self.host = '127.0.0.1' # this is the default localhost
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
        except:
            print('Controller could not be connected')
        
        self.initializeController() # this initializes the appropriate controller
    
    def initializeController(self):
        if self.controller == 'engine-speed':
            """ This controller expects angular velocity in rad/s"""
            self.action_fn = self.set_engine_speed
            # assert len(action) == 4
        if self.controller == 'attitude':
            """ This controller expects roll, pitch, yaw, thrust"""
            self.action_fn = self.set_attitude
            # assert len(action) == 4
        if self.controller ==  'velocity':
            """ This controller expects vx, vy, vz, vyaw, tolerance"""
            self.action_fn = self.set_velocity_thrust
            #assert len(action) == 4
        if self.controller == 'waypoint':
            """ This controller expects x, y, z, yaw """
            self.action_fn = self.set_waypoint
            #assert len(action) == 4
        if self.controller == 'teleport':
            """ This controller expects a certain translation and rotation"""
            # It is not recommended to use this controller as it does not
            # implement the quadrotor dynamics.
            
            self.action_fn = self.set_position_orientation
            #assert len(action) == 6
    
    def set_engine_speed(self, action):
        """ Should be used with the engine-speed controller """
        # currently the support for this controller is experimental
        low = np.array([0., 0., 0., 0.]).astype(np.float16)
        high = np.array([100., 100., 100., 100.]).astype(np.float16)
        action = np.asarray(action).astype(np.float32)
        action = (action + 1)/2 # in range (0., 1.)
        
        action = ((high - low) * action) + low
        e1, e2, e3 ,e4 = action # these are the engine speeds in rad/s
        try:
            robot_name = f'robots_{int(self.robot_number)}'
            msg = f'id{self.robot_number} {robot_name}.motion engines [%f, %f, %f, %f]\n' % (e1,e2,e3,e4)
            self.sock.send(msg.encode("utf-8"))
        except Exception as error_msg:
            print(error_msg) # if broken_pipe, increase time.
            
    def set_attitude(self, action):
        """ The thrust is the collective thrust in percentage """ 
        # currently the support for this controller is experimental.
        low = np.array([-np.pi/5, -np.pi/5, 0., 0.]).astype(np.float16)
        high = np.array([np.pi/5, np.pi/5, 2*np.pi, 100.]).astype(np.float16)
        action = np.asarray(action).astype(np.float32)
        action = (action + 1)/2 # in range (0., 1.)
        
        action = ((high - low) * action) + low
        roll, pitch, yaw, thrust = action
        assert thrust >= 0.0 and thrust <= 1.0
        try:
            robot_name = f'robots_{int(self.robot_number)}'
            msg = f'id{self.robot_number} {robot_name}.motion roll %f\n \
                    id{self.robot_number} {robot_name}.motion pitch %f\n \
                    id{self.robot_number} {robot_name}.motion yaw %f\n \
                    id{self.robot_number} {robot_name}.motion thrust %f\n' % (roll, pitch, yaw, thrust)
            self.sock.send(msg.encode("utf-8"))
        except Exception as error_msg:
            print(error_msg)
            
    def set_velocity_thrust(self, action):
        low = np.array([0., 0., 0., 0.]).astype(np.float16)
        high = np.array([100., 100., 100., 100.]).astype(np.float16)
        action = np.asarray(action).astype(np.float32)
        action = (action + 1)/2 # in range (0., 1.)
        
        action = ((high - low) * action) + low
        vx, vy, vz, vyaw = action
        tolerance  = 0.1
        try:
            robot_name = f'robots_{int(self.robot_number)}'
            msg = f'id{self.robot_number} {robot_name}.motion setvel [%f,%f,%f,%f,%f]\n' % (vx, vy, vz, vyaw, tolerance)
            self.sock.send(msg.encode("utf-8"))
        except Exception as error_msg:
            print(error_msg)
    
    def set_waypoint(self, action):
        low = np.array([-100., -100., 0., 0.]).astype(np.float16)
        high = np.array([100., 100., 11, 2*np.pi]).astype(np.float16)
        action = np.asarray(action).astype(np.float32)
        action = (action + 1)/2 # in range (0., 1.)
        
        action = ((high - low) * action) + low
        x, y, z, yaw = action
        tolerance = 0.1
        try:
            robot_name = f'robots_{int(self.robot_number)}'
            msg = f'id{self.robot_number} {robot_name}.motion setdest [%f, %f, %f, %f, %f]\n' % (x, y, z, yaw, tolerance)
            self.sock.send(msg.encode("utf-8"))
        except Exception as error_msg:
            print(error_msg)
    
    def set_position_orientation(self, action):
        
        low = np.array([-100., -100., 0, 0., -np.pi/5, -np.pi/5]).astype(np.float16)
        high = np.array([100., 100., 11, 2*np.pi, np.pi/5, np.pi/5]).astype(np.float16)
        action = np.asarray(action).astype(np.float32)
        action = (action + 1)/2 # in range (0., 1.)
        
        action = ((high - low) * action) + low
        
        x, y, z, roll, pitch, yaw = action
        try:
            num = self.robot_number
            robot_name = f'robots_{int(num)}'
            msg = f"id{num} {robot_name}.motion rotate [%f, %f, %f]\n" % (roll, pitch, yaw)
            self.sock.send(msg.encode("utf-8"))
            time.sleep(0.005) # some time to register and carry out the action.
            msg = f"id{num} {robot_name}.motion translate [%f, %f, %f]\n" % (x,y,z)
            time.sleep(0.005)
            self.sock.send(msg.encode("utf-8"))
        except Exception as error_msg:
            print(error_msg)
    def set_action(self, action):
        self.action_fn(action)
        return
            
    
       
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
            
            
            
            
            