import os
import numpy as np
from morse_simulator.algorithms.serialization import *
from morse_simulator.algorithms.config import config
import subprocess
import time
def generateEnvs():
    index = 0
    """
    superEnvironmentStatus.log -> This indicates the status code that signals the simulations to start. This is initially zero but is changed to 1 after
            generating the super environments.
    numRobots.txt -> This file keeps track of the number of robots in the simulations and the current robot index (This is required for assigning the 
                    robots the the master socket manager)
    currentSuperIndex.log -> This keeps track of the current super-environment being generated
    superEnvironmentIdx.log -> This keeps track of robot initilizations assigned to each of the super-environments
    """
    num_robots_per_senv = config.num_robots_per_senv
    num_robots = get_num_robots()
    reset_robot_server_correspondences()
    initPath = '/'.join(os.path.abspath(__file__).split('/')[:-1])
    for i in range(num_robots//num_robots_per_senv):
        sIdx = get_superenvironment_limit()
        superIdx = sIdx//(num_robots//num_robots_per_senv) + i
        save_running_suIdx(superIdx)
        #subprocess.call(f'gnome-terminal --tab -- morse run robot_initialization.py', shell = True) # This call should necessarily be blocking.
        if config.controller == 'teleport':
            
            subprocess.call(f'morse run {initPath}/TeleportInitialization.py', shell = True)
            print('TELEPORT INITIALIZATION')
        if config.controller == 'waypoint':
            subprocess.call(f'gnome-terminal --tab -- morse run {initPath}/waypointInitialization.py', shell = True)
        if config.controller == 'engine-speed':
            subprocess.call(f'gnome-terminal --tab -- morse run {initPath}/engineInitialization.py', shell = True)
        if config.controller == 'attitude':
            subprocess.call(f'gnome-terminal --tab -- morse run {initPath}/attitudeInitialization.py', shell = True)
        if config.controller == 'velocity':
            subprocess.call(f'gnome-terminal --tab -- morse run {initPath}/velocityInitialization.py', shell = True)
        
        time.sleep(1.5) # enforces synchronization
    save_env_generation_status(1) # save status '1' after generting environments.