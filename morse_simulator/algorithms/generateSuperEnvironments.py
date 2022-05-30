# Considering the Problems that Blenderpy has with Ubuntu, this is a very dirty way of dynamically generating Blender environments.
# This file will be replaced by a better version soon (As soon as blenderpy's issues with compiling on Ubuntu are sorted.)
# Blender Environment Generator File.
import subprocess
from tqdm import tqdm
import numpy as np
from morse_simulator.algorithms.serialization import *
import os
from morse_simulator.algorithms.config import config
# from morse_simulator.algorithms.blenderEnv import createEnvs # illegal import

ABS_PATH = os.path.abspath('./')

num_robots_per_senv = config.num_robots_per_senv

def clear_old_envs():
    os.system('rm '+ABS_PATH+'/workingSuperEnvironment -r')
    os.mkdir(ABS_PATH+'/workingSuperEnvironment')
    return

clear_old_envs() # As the name suggests

def generateSuperEnvs():
    print('GENERATING SUPER-ENVIROMENTS')
    print('^.^.^.^.^.^.^.^.^.^.^.^.^.^.^')

    ports = get_port_information()
    num_robots = len(np.array(ports)[0,:]) - 1
    blenderFilePath = '/'.join(os.path.abspath(__file__).split('/')[:-1])# pathlib.Path().resolve()
     
    for i in tqdm(range(num_robots//num_robots_per_senv)): # The Goal is to run 65 environments in Parallel (meaning that we wil have 65*4 robots (sub-environments being mapped))
        subprocess.call(['blender','--background','--python',f'{blenderFilePath}/blenderEnv.py']) # This call has to necessarily be blocking

