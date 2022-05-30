# configuration file.
# from Pose import DepthCameraPose # circular import
import os
import json
class config:
    savePath = '/home/athrva/Documents/morse_package_testing' # for logs and stuff.
    
    port_config = {"robot_server_correspondence": savePath + '/logs/robotServerPortCorrespondence.txt',
                   "server_port": savePath + '/logs/server_ports.txt',
                   "sensor_port": savePath + '/logs/port_information.csv'}
    # if not os.path.exists('./port_config.json'):
    json_info = json.dumps(port_config)
    with open('./port_config.json', 'w+') as port_config_file:
        port_config_file.write(json_info)
    if not os.path.exists(savePath + '/logs'):
        os.mkdir(savePath + '/logs')
    if not os.path.exists(savePath + '/results'):
        os.makedirs(savePath + '/results', exist_ok=True)
    def close(child_processes):
        for process in tqdm(child_processes):
            process.kill()
            process.wait() # This call is required
    def writeNRtotxt(num_robots_per_senv, savePath):
        with open(savePath + '/logs/NRobots.txt', 'w+') as nrfile:
            nrfile.write(str(num_robots_per_senv))
        return
    """ Parameters for the simulation ""
    "" Instead of writing a yaml, use this as the configuration file """
    controller = 'teleport'
    maxTime = 1000
    total_robots = 1
    num_robots_per_senv = 1 # This is deprecated. DO NOT CHANGE THIS. THIS FEATURE IS NOT MEANT FOR THE RL SIMULATOR.
    maxIterations = 1e5 # This is not meant for the RL simulator.
    
    
    
    writeNRtotxt(num_robots_per_senv, savePath)
    

    """ Collected Items : These are the items that are collected from different files of the simulator."""
    robot_number = None
    server_port = None
    motion_port = None
    pose_port = None
    depthcamera_port = None
    child_processes = None # this saves the PIDs of the child processes. These processes are killed when env.hardReset is called
    occupancy_grid = None # this is the running occupancy grid during the simualtions.
    initialPosition = None # this is the spawn position of the robots in the environment. (Used during env.softReset)
    startTime = None # this is used to see if the episode has ended. We use a limited time episode.
    
    """ Transactional items : This file is also used to share variables between different files of this workspace"""
    status_dict = {} # holds the reward and the episode number.
    episode_reward = None # redundant.

if __name__ == '__main__':
    config_obj = config()
    
    
    
    