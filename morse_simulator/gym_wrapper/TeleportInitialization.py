# Initialize all robots.
import numpy as np
from morse.builder import *
from numpy import genfromtxt
from morse.middleware.socket_request_manager import *
from morse_simulator.algorithms.config import config
"""
translation_coordinates.txt -> This file is generated from blender and tells morse where to place the robots in the sub-environments
currentSuperIndex.log -> Keeps track of the super-environment in which we need to place the robots
port_information.csv -> contains all the socket ports robot-wise for connection to the socket manager
server_ports.txt -> contains all the ports that need to be used for the master server managers
robotServerPortCorrespondence.txt -> Book-keeping to check and save the robot's ports and their respective master server manager 
"""

with open(config.savePath + '/logs/currentSuperIndex.log', 'r') as curr_sIdx:
    content = []
    stringIdx = curr_sIdx.readlines()
    content = list(stringIdx[0].strip(" "))
    sIdx = int(content[-1])



# Adding robot initial coordinates based on environment
with open(config.savePath + '/logs/translation_coordinates.txt','r') as file_trs_coord:
    lines = file_trs_coord.readlines()
x_coord = list(np.zeros(len(lines)))
y_coord = list(np.zeros(len(lines)))
for i,element in enumerate(lines):
    x_coord[i] = int(eval(element[:-1].split(',')[0]))
    y_coord[i] = int(eval(element[:-1].split(',')[-1]))

corner_offsetx, corner_offsety = (24.98999786376953 - 12.7)/2,(19.94999885559082 - 12.7)/2 # these numbers have been calculated based on the length and width of the maps

# Sensor Frequencies.
IMU_FREQUENCY = 30  
LIDAR_FREQUENCY = 30
POSE_FREQUENCY = 30
DCAM_FREQUENCY = 30
VIDEOCAMERA_FREQUENCY = 30
ODO_FREQUENCY = 30

ports = list(genfromtxt(config.savePath + '/logs/port_information.csv', delimiter=',')[1:,:])

with open(config.savePath + '/logs/server_ports.txt', 'r') as server_file:
    jk = server_file.readlines()

server_ports = [int(entry) for entry in jk[0].split(',')[:-1]]

num_robots = len(np.array(ports)[0,:]) - 1

with open(config.savePath + '/logs/currIndex.txt','r') as currIndex:
    startIndex = int(currIndex.readlines()[0])

if startIndex != 0: # Fix for multiple servers having the same robot names.
    startIndex += 1
max_robots_per_senv = config.num_robots_per_senv
server_port_index = (startIndex)//max_robots_per_senv # This is the port of the server that manages this sub-environment instance

set_server_port(server_ports[server_port_index])
with open(config.savePath + '/logs/robotServerPortCorrespondence.txt','a+') as rsc:
    for i in range(startIndex,startIndex + max_robots_per_senv):
        rsc.write(str(server_ports[server_port_index])+','+str(i))
        rsc.write('\n')
k = -1



for i in range(startIndex,startIndex + max_robots_per_senv):
    k += 1
    print('INDEX : ',i)
    local_ports = list(np.array(ports)[:,i+1].astype(np.int32))
    # print("Using Ports : ", local_ports)

    pose_port = local_ports[0] 
    depthcamera_port = local_ports[1] 
    motion_port = local_ports[2]
    display_camera_port = local_ports[3]

    
    # Initial position of sensors
    TRANS_X = 0.0
    TRANS_Y = 0.0
    TRANS_Z = 0.0
    # =====================================================================================
    # Adding robot
    exec(f"robots_{i} = Quadrotor()")
    #robots = ATRV()
    exec(f"robots_{i}.translate(x_coord[k] - corner_offsetx, y_coord[k] - corner_offsety, TRANS_Z+7)")
    # =====================================================================================
    # Adding motion controller
    motion = Teleport() 

    motion.translate(z=0.0)

    motion.add_stream('socket', port = motion_port)
    motion.add_service('socket')

    exec(f"robots_{i}.append(motion)")

    # =====================================================================================
    # Adding Pose sensor
    pose = Pose()

    pose.translate(TRANS_X, TRANS_Y, TRANS_Z)
    pose.frequency(POSE_FREQUENCY)
    pose.add_stream('socket', port=pose_port)
    pose.add_service('socket')

    exec(f"robots_{i}.append(pose)")
    # ======================================================================================
    dcam = DepthCamera()
    dcam.properties(cam_width=917)
    dcam.properties(cam_height=17)
    dcam.properties(cam_far=25.0)
    dcam.properties(cam_near=0.01)
    dcam.frequency(DCAM_FREQUENCY)
    dcam.translate(TRANS_X, TRANS_Y, TRANS_Z + 0.17)      
    dcam.rotate(0.0, -math.pi/12, math.pi)       
    dcam.add_stream('socket', port=depthcamera_port)
    
    
    dcam.add_interface('socket')
    exec(f"robots_{i}.append(dcam)")

with open(config.savePath + '/logs/currIndex.txt','w+') as currIndex:
    currIndex.write(f'{str(i)}')

env = Environment(f'./workingSuperEnvironment/superEnvironment_{sIdx}.blend', fastmode=False)
# env.set_time_scale(5) # DO NOT USE THIS.
env.properties(longitude=54.371209, latitude=18.613334, altitude=30.0)
env._cfg_camera_scene(res_x = 917, res_y = 37) 
env.create(resolution_x = 917, resolution_y = 37) # testing custom render screens to save screen real-estate
