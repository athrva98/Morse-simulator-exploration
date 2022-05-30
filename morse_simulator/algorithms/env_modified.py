# Environment File for Morse Multi-robot Simulations.
import numpy as np
import pandas as pd
import socket
from morse_simulator.algorithms.serialization import *
from morse_simulator.algorithms.config import config
"""
This file initializes all the socket ports to be used for the robots and the server managers.
server_ports.txt -> contains all the ports for the master server managers
port_information.csv -> contains robot-wise port initialization (some ports are redundant)
"""
def initialize(num_robots):
    num_robots_per_senv = config.num_robots_per_senv

    
    if num_robots % num_robots_per_senv != 0:
        print(f'[WARNING] The number of robots mus be a multiple of {num_robots_per_senv}. Automatically taking the closest multiple.')

    print(f'Number of Robots in Simulations : {num_robots}')
    all_ports = list(np.zeros(4)) # This holds all the ports for all robots.
    """
    We need just three ports per robot.
    1. DepthCamera
    2. MotionController
    3. PoseServer
    4. As a Redundancy
    """

    index = 0

    df = pd.DataFrame({})

    for j in range(num_robots):
        for i in range(4):
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.bind(("localhost",0))
            s.listen(1)
            port = s.getsockname()[1]
            s.close()
            all_ports[i] = port
        df['Robot_'+str(j)] = all_ports

    server_ports = []
    for j in range(num_robots//num_robots_per_senv): # For Socket Servers
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(("localhost",0))
        s.listen(1)
        port = s.getsockname()[1]
        s.close()
        server_ports.append(port)

    save_server_ports(server_ports)
    save_port_information(df)
    save_current_robot_index(index)
    save_num_robots(num_robots)
    save_environment_index(index)


