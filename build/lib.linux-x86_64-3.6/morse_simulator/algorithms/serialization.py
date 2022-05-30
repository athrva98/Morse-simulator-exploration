### All the logging functions are defined here.
import numpy as np
import os
import json
from glob import glob
from morse_simulator.algorithms.config import config


with open('./port_config.json','r') as readJson:
    pathDict = json.load(readJson)
print(pathDict)

def save_server_ports(server_ports): # Generates master servers for sub-simulations
    with open(pathDict['server_port'], 'w+') as server_file:
        for server in server_ports:
            server_file.write(str(server) + ',')
    return

def calculate_index():
    ars = glob(config.savePath + '/results/*')
    idx = -1e3
    if len(ars) > 0:
        for r in ars:
            if 'robot' in r:
                cnum = int(r.split('_')[-1])
                if cnum > idx:
                    idx = cnum
    else:
        idx = 0
    with open(config.savePath + '/logs/morse_offset.log', 'w+') as moff:
        moff.write(str(idx))
    return idx            

def clear_run_logs():
    files = glob(config.savePath + '/logs/*')
    for file in files:
        if not 'allObjs.log' in file and not 'buffer' in file:
            os.remove(file)
    return
def save_port_information(df): # Saves the sensor-ports per robot
    df.to_csv(pathDict['sensor_port'])
    return

def save_current_robot_index(idx):
    with open(config.savePath + '/logs/currIndex.txt','w+') as currIndex:
        currIndex.write(str(idx))
    return

def save_num_robots(num_robots):
    with open(config.savePath + '/logs/numRobots.txt','w+') as currIndex:
        currIndex.write(str(num_robots))
    return

def save_environment_index(idx):
    with open(config.savePath + '/logs/superEnvironmentIdx.log','w+') as sIdx:
        sIdx.write(str(idx))
    return
def save_running_suIdx(idx):
    with open(config.savePath + '/logs/currentSuperIndex.log', 'a+') as curr_sIdx:
        curr_sIdx.write(str(idx))
    return

def save_env_generation_status(status):
    with open(config.savePath + '/logs/superEnvironmentStatus.log', 'w+') as status_code:
        status_code.write(str(status)) # After generating all super environments, the status code is changed to 1.
def read_env_generation_status():
    with open(config.savePath + '/logs/superEnvironmentStatus.log', 'r') as status_code:
        status = int(status_code.readlines()[0])
    return status
def get_port_information():
    ports = list(np.genfromtxt(config.savePath + '/logs/port_information.csv', delimiter=',')[1:,:])
    return ports

def get_superenvironment_limit():
    with open(config.savePath + '/logs/superEnvironmentIdx.log', 'r') as senv:
        sIdx = int(senv.readlines()[0]) - 1
    return sIdx

def get_num_robots():
    with open(config.savePath + '/logs/numRobots.txt','r') as currIndex:
        num_robots = int(currIndex.readlines()[0])
    return num_robots

def reset_robot_server_correspondences():
    with open(pathDict['robot_server_correspondence'],'w+') as rsc: # Clears whatever was written after the previous run
        pass
def reset_current_super_index():
    with open(config.savePath + '/logs/currentSuperIndex.log', 'w+') as curr_sIdx: # Clears whatever was written after the previous run
        pass
def write_occupancy_coordinates(x, y,sIdx, robot_number, image_number):
    iter_ = read_outer_iteration()
    try:
        with open(config.savePath + f'/results/{iter_}/robot_{sIdx}_{robot_number}/occupancy_coordinates_{image_number}.log', 'w+') as coord_file:
                for i in range(len(x)):
                    coord_file.write(str(x[i])+','+str(y[i]))
                    coord_file.write('\n')
    except:
        print('Occupancy Saving Failed')
    return
def write_depth_image(points,sIdx, robot_number, image_number):
    iter_ = read_outer_iteration()
    try:
        with open(config.savePath + f'/results/{iter_}/robot_{sIdx}_{robot_number}/rawDepth_{image_number}.txt','wb') as binaryDepth:
            binaryDepth.write(points)
        #np.save(f'./results/robot_{sIdx}_{robot_number}/depth_{image_number}.npy', points)
    except:
        print('Depth serialization failed')
    return
def write_pose_information(pose_x, pose_y, pose_z, yaw, pitch, roll,sIdx, robot_number, image_number):
    poseInformation = np.r_[pose_x, pose_y, pose_z, yaw, pitch, roll] # row-wise stacking
    iter_ = read_outer_iteration()
    try:
        np.save(config.savePath + f'/results/{iter_}/robot_{sIdx}_{robot_number}/pose_{image_number}.npy', poseInformation)
    except:
        print('Pose serialization failed')
    return
def write_robot_trajectory(pose_x, pose_y,sIdx, robot_number, image_number):
    iter_ = read_outer_iteration()
    try:
        with open(config.savePath + f'/results/{iter_}/robot_{sIdx}_{robot_number}/robot_trajectory_{image_number}.log', 'w+') as coord_file:
                for i in range(len([pose_x])):
                    coord_file.write(str([pose_x][i])+','+str([pose_y][i]))
                    coord_file.write('\n')
    except:
        print('Trajectory svaing failed')
    return
def read_current_super_index():
    with open(config.savePath + '/logs/currentSuperIndex.log', 'r') as curr_sIdx:
            content = []
            stringIdx = curr_sIdx.readlines()
            content = list(stringIdx[0].strip(" "))
            sIdx = int(content[-1])
    return sIdx
def calculate_start_iteration():
    sub_dirs = glob(config.savePath + f'/results/*')
    if not len(sub_dirs) == 0:
        initial_iteration = len(sub_dirs) - 1
        return initial_iteration
    else:
        return 0
def create_robot_folder(sIdx, robot_number):

    iter_ = read_outer_iteration()
    print(f'Creating Folder {sIdx}, {robot_number}, {iter_}')
    if not os.path.exists(config.savePath + f'/results/{iter_}'):
        os.mkdir(config.savePath + f'/results/{iter_}')
    if not os.path.exists(config.savePath + f'/results/{iter_}/robot_{sIdx}_{robot_number}'):
        os.mkdir(config.savePath + f'/results/{iter_}/robot_{sIdx}_{robot_number}')
    return
def save_outer_iteration(iter_):
    with open(config.savePath + '/logs/Iteration.log','w+') as iterations:
        iterations.write(str(iter_))
    return
def read_outer_iteration():
    with open(config.savePath + '/logs/Iteration.log','r') as iterations:
        iter_ = int(iterations.readlines()[0])
    return iter_
def get_server_port(robot_number):
    with open(config.savePath + '/logs/robotServerPortCorrespondence.txt', 'r') as server_file:
        jk = server_file.readlines()
        
    for i in range(len(jk)):
        if ','+str(robot_number) in jk[i]:
            server_port = int(jk[i].split(',')[0])
            break
    return server_port

def get_server_port_index(server_port_number, robot_number):
    num_robots_per_senv = config.num_robots_per_senv
    if (robot_number) % num_robots_per_senv == 0:
        server_port_number += 1
    return server_port_number

