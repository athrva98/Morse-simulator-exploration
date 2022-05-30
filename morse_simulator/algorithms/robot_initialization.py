# Initialize all robots.
import numpy as np
from morse.builder import *
from numpy import genfromtxt
from morse.middleware.socket_request_manager import *

"""
translation_coordinates.txt -> This file is generated from blender and tells morse where to place the robots in the sub-environments
currentSuperIndex.log -> Keeps track of the super-environment in which we need to place the robots
port_information.csv -> contains all the socket ports robot-wise for connection to the socket manager
server_ports.txt -> contains all the ports that need to be used for the master server managers
robotServerPortCorrespondence.txt -> Book-keeping to check and save the robot's ports and their respective master server manager 
"""

with open('./logs/currentSuperIndex.log', 'r') as curr_sIdx:
    content = []
    stringIdx = curr_sIdx.readlines()
    content = list(stringIdx[0].strip(" "))
    sIdx = int(content[-1])



# Adding robot initial coordinates based on environment
with open('./logs/translation_coordinates.txt','r') as file_trs_coord:
    lines = file_trs_coord.readlines()
x_coord = list(np.zeros(len(lines)))
y_coord = list(np.zeros(len(lines)))
for i,element in enumerate(lines):
    x_coord[i] = int(eval(element[:-1].split(',')[0]))
    y_coord[i] = int(eval(element[:-1].split(',')[-1]))

# Sensor Frequencies.
IMU_FREQUENCY = 30  
LIDAR_FREQUENCY = 30
POSE_FREQUENCY = 30
DCAM_FREQUENCY = 30
VIDEOCAMERA_FREQUENCY = 30
ODO_FREQUENCY = 30

ports = list(genfromtxt('./logs/port_information.csv', delimiter=',')[1:,:])

with open('./logs/server_ports.txt', 'r') as server_file:
    jk = server_file.readlines()

server_ports = [int(entry) for entry in jk[0].split(',')[:-1]]

num_robots = len(np.array(ports)[0,:]) - 1

with open('./logs/currIndex.txt','r') as currIndex:
    startIndex = int(currIndex.readlines()[0])

if startIndex != 0: # Fix for multiple servers having the same robot names.
    startIndex += 1


#with open('./logs/morse_offset.log', 'r') as moff:
#    __idx = int(moff.readline()[0])
#__idx = 7
server_port_index = (startIndex)//4 # This is the port of the server that manages this sub-environment instance

set_server_port(server_ports[server_port_index])
with open('./logs/robotServerPortCorrespondence.txt','a+') as rsc:
    for i in range(startIndex,startIndex + 4):
        rsc.write(str(server_ports[server_port_index])+','+str(i))
        rsc.write('\n')
k = -1
for i in range(startIndex,startIndex + 4):
    k += 1
    print('INDEX : ',i)
    local_ports = list(np.array(ports)[:,i+1].astype(np.int32))
    print("Using Ports : ", local_ports)

    videocamera_port = local_ports[0] 
    pose_port = local_ports[1] 
    depthcamera_port = local_ports[2] 
    waypoint_port = local_ports[3] 
    imu_port = local_ports[4] 
    laser_port = local_ports[5] 
    right_laser_port = local_ports[6] 
    left_laser_port = local_ports[7] 
    odometry_port = local_ports[8] 
    motion_port = local_ports[9]
    display_camera_port = local_ports[10]

    
    # Initial position of sensors
    TRANS_X = 0.5
    TRANS_Y = 0.0
    TRANS_Z = 0.3
    # =====================================================================================
    # Adding robot
    exec(f"robots_{i} = ATRV()")
    #robots = ATRV()
    exec(f"robots_{i}.translate(x_coord[k], y_coord[k], TRANS_Z)")
    # =====================================================================================
    # Adding motion controller
    motion = MotionVW() # This is a publisher based on linear and angular velocity.

    motion.translate(z=0.3)

    motion.add_stream('socket', port = motion_port)
    motion.add_service('socket')

    exec(f"robots_{i}.append(motion)")

    waypoint = Waypoint() # This is the waypoint based controller.
    waypoint.add_stream('socket', port = waypoint_port)
    waypoint.add_service('socket')

    exec(f"robots_{i}.append(waypoint)")
    # =====================================================================================
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
    dcam.properties(cam_width=217)
    dcam.properties(cam_height=217)
    dcam.properties(cam_far=15.0)
    dcam.properties(cam_near=0.1)
    dcam.frequency(DCAM_FREQUENCY)
    dcam.translate(TRANS_X, TRANS_Y, TRANS_Z)      
    dcam.rotate(0.0, -math.pi/8, math.pi)       
    dcam.add_stream('socket', port=depthcamera_port)
    dcam.add_interface('socket')
    exec(f"robots_{i}.append(dcam)")

    # ======================================================================================
    # Adding long laser to measure distance in front of the vehicle
    laser = Sick()
    laser.translate(0, 0, TRANS_Z)
    laser.properties(Visible_arc=True) 
    laser.properties(laser_range=5.0) 
    laser.properties(resolution=5)  
    laser.properties(scan_window=40)
    laser.frequency(LIDAR_FREQUENCY)  
    laser.add_stream('socket', port=laser_port)
    laser.add_service('socket')

    exec(f"robots_{i}.append(laser)")

    # ======================================================================================
    # Adding long laser to measure distance on the right side of the vehicle
    right = Sick()
    right.translate(0, 0, TRANS_Z)
    right.rotate(0, 0, -math.pi/2)
    right.properties(Visible_arc=True) 
    right.properties(laser_range=5.0) 
    right.properties(resolution=5)  
    right.properties(scan_window=20)
    right.frequency(LIDAR_FREQUENCY)  
    right.add_stream('socket', port=right_laser_port)
    right.add_service('socket')

    exec(f"robots_{i}.append(right)")

    # ======================================================================================
    # Adding long laser to measure distance on the left side of the vehicle
    left = Sick()
    left.translate(0, 0, TRANS_Z)
    left.rotate(0, 0, math.pi/2)
    left.properties(Visible_arc=True) 
    left.properties(laser_range=5.0)  
    left.properties(resolution=5)  
    left.properties(scan_window=20)
    left.frequency(LIDAR_FREQUENCY) 
    left.add_stream('socket', port=left_laser_port)
    left.add_service('socket')

    exec(f"robots_{i}.append(left)")

    # ======================================================================================
    # Adding encoder
    odometry_differential=Odometry()
    odometry_differential.level('differential')
    odometry_differential.frequency(ODO_FREQUENCY)
    odometry_differential.add_stream('socket', port=odometry_port)
    odometry_differential.add_service('socket')
    exec(f"robots_{i}.append(odometry_differential)")
    # ======================================================================================
with open('./logs/currIndex.txt','w+') as currIndex:
    currIndex.write(f'{str(i)}')

print(sIdx)
env = Environment(f'./workingSuperEnvironment/superEnvironment_{sIdx}.blend', fastmode=False)
env.properties(longitude=54.371209, latitude=18.613334, altitude=30.0)

env.create()
