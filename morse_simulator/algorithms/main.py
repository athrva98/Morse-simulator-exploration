import subprocess
import psutil
import os
from morse_simulator.algorithms.serialization import *
from morse_simulator.algorithms.start import start_simulations
from morse_simulator.algorithms.config import config
from morse_simulator.algorithms.env_modified import initialize
from morse_simulator.algorithms.generateSuperEnvironments import generateSuperEnvs
from morse_simulator.algorithms.generate_envs import generateEnvs

def main():
    checkedOffest = False
    for i in range(int(config.maxIterations)): # Run 3 batches of 24 robots.
        if not checkedOffest:
            offset = calculate_start_iteration()
            checkedOffest = True
        print('Current Iteration : ', i + offset)
        clear_run_logs()
        save_outer_iteration(i + offset)
        save_env_generation_status(0) # After generating all super environments, the status code is changed to 1.
        
        
        # subprocess.call(['ulimit','-n','4096'])
        os.system('ulimit -n 4096')
        initialize(config.total_robots)
        generateSuperEnvs()
        generateEnvs()
        
        status = 0

        while status == 0: # wait until the environments are generated before starting the simulations (otherwise the sockets can refuse to connect).
            try: # This is required because we are accessing the file from more than one process simultaneously
                status = read_env_generation_status() # After generating all super environments, the status code is changed to 1.
            except:
                pass
        #process = subprocess.Popen(['gnome-terminal','--tab','--','python3','start.py']) # starts simulations
        _ = start_simulations() # this is a blocking call

        #while psutil.pid_exists(process.pid): # This loop is non blocking so this is needed.
        #    pass
if __name__ == "__main__":
    main()
