from morse_simulator.algorithms.dataStreamReader import *
import numpy as np
import socket
from morse_simulator.algorithms.serialization import *
from morse_simulator.algorithms.Pose import map_t
import time

class Control:
    def __init__(self, robot_number, motion_port, host = '127.0.0.1'):
        time.sleep(3)
        self.robot_number = robot_number
        self.server_port_index = -1

        
        self.start = False
        
        
        self.map_transform = map_t

        # for communication with motion actuator
        self.port = get_server_port(self.robot_number)
        self.host = host
        
        #try:
        print("Attempting connection to Port : ", self.port, self.robot_number)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.sock.connect((self.host, self.port))
        print('Connected')
        #except:
        #    print("Motion controller connection failed")
    def close_connection(self):
        self.sock.close()
        return
    def take_random_action(self,pose, pose_offset, yaw):
        pose = pose # np.add(np.asarray(pose).reshape(1,2), np.asarray(pose_offset).reshape(1,2)) + np.random.randint(-2,2)*0.05
        waypoint = pose
        # print('Scanning')
        # self.set_waypoint(waypoint, exp_yaw = yaw)    
        try:
            num = self.robot_number
            robot_name = f'robots_{int(num)}'
            #msg = f"id{num} {robot_name}.motion goto [%f, %f, %f, %f, 1e-3]\n" % (x,y,z,yaw) # goto(x, y, z, yaw, tolerance)
            msg = f"id{num} {robot_name}.motion rotate [0, 0, %f]\n" % (yaw)
            self.sock.send(msg.encode("utf-8"))
        except Exception as error_msg:
            print(error_msg)
        return
    def set_waypoint(self, waypoint, exp_yaw = None): # this is blocking.
        #print('\n\n\nWAYPOINT : ', waypoint)
        assert len(waypoint.flatten()) == 2
        #print('Received Wapoint : ',waypoint)
        x, y = waypoint.flatten() # these are the variables that we need.
        print('RELATIVE TRANSLATION : ',(x,y))
        z, yaw = 7.0,np.random.randint(5,25)*(np.pi/180)
        if not exp_yaw is None:
            yaw = exp_yaw
        tolerance = 0.1
        try:
            num = self.robot_number
            robot_name = f'robots_{int(num)}'
            #msg = f"id{num} {robot_name}.motion goto [%f, %f, %f, %f, 1e-3]\n" % (x,y,z,yaw) # goto(x, y, z, yaw, tolerance)
            
            msg = f"id{num} {robot_name}.motion rotate [0, 0, %f]\n" % (yaw)
            self.sock.send(msg.encode("utf-8"))
            time.sleep(0.01) # some time to register and carry out the action.
            msg = f"id{num} {robot_name}.motion translate [%f, %f, %f]\n" % (x,y,z)
            time.sleep(0.01)
            self.sock.send(msg.encode("utf-8"))
        except Exception as error_msg:
            print(error_msg)

    
