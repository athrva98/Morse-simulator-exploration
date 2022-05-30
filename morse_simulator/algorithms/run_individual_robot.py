# Calling Script
# Spawned as a sub-process
import numpy as np
from morse_simulator.algorithms.independent_multi_robot_fast import MASTER
import argparse
from morse_simulator.algorithms.serialization import *

parser = argparse.ArgumentParser(description='Independent Multi-Robot Simulations')
parser.add_argument('-n','--robot_instance_number', help='Index of the Robot being simulated', required=True)
args = vars(parser.parse_args())

ports = get_port_information()

local_ports = list(np.array(ports)[:,int(args['robot_instance_number'])].astype(np.int32))


exec(f'MASTER_{int(args["robot_instance_number"])} = MASTER(ports = local_ports, robot_number = int(args["robot_instance_number"]))') # Exec is not required here. Remove it.
#exec(f'megamain_{int(args["robot_instance_number"])}.run_simulation_vw()') # for the older linear-angular velocity based simulator.
exec(f'MASTER_{int(args["robot_instance_number"])}.run_simulation_waypoint()')
