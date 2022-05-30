# Starting simulations
import numpy as np
from numpy import genfromtxt
import subprocess
import time
from tqdm import tqdm
import pathlib
from morse_simulator.algorithms.serialization import *
from morse_simulator.algorithms.config import config
def start_simulations():

    ports = get_port_information()

    num_robots = len(np.array(ports)[0,:]) - 1

    start_time = time.time()

    simulation_time = config.maxTime #360 # read this from configuration file
    abs_path = os.path.abspath(__file__)# pathlib.Path().resolve()
    abs_path = '/'.join(abs_path.split('/')[:-1])
    print('Module Path : ', abs_path)

    child_processes = [] # Container to hold all the generated processes
    print("Starting Robot Simulations...")
    for i in tqdm(range(num_robots)):
        proc = subprocess.Popen(['python3',f'{abs_path}/run_individual_robot.py',f'-n {i+1}'])
        child_processes.append(proc)
    config.child_processes = child_processes
    while time.time() < start_time + simulation_time: # Simply wait till the simulation gets over
        pass
    for process in tqdm(child_processes):
        process.kill()
        process.wait() # This call is required
    return 1

    
