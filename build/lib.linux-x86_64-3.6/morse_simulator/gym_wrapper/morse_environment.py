import numpy as np
import cv2
from morse_simulator.algorithms.config import config
from morse_simulator.gym_wrapper.main import main
from morse_simulator.algorithms.serialization import get_server_port
from pymorse import Morse

class morseConnection:
    
    def __init__(self):
        self.robot_number = 0
        
        self.controller = config.controller
        self._isValidController()
        self.hardResetMain = main
        self.start()
        self.server_port = get_server_port(self.robot_number)
        pass
    def start(self):
        self.hardResetMain()
        
    def _isValidController(self):
        try:
            assert self.controller in ('force-torque',
                                    'attitude',
                                    'linear-angular-velocity',
                                    'teleport',
                                    'waypoint')
        except:
            raise NotImplementedError('Requested controller cannot be found.')
        
    def pauseSim(self):
        # Unfortunately, this cannot be done in MORSE
        # TODO: set some global flag that stops all requests to the
        # controller and keeps the robot in the current state
        # This will be difficult to do for controllers like 
        # force-torque/attitude/velocity-angular controller
        with Morse(port = self.server_port) as morse:
            morse.rpc('deactivate','robots_0') # deactivates all dynamics of the robot
        return
    def checkSabotage(self):
        with Morse(port = self.server_port) as morse:
            pose = morse.rpc('robots_0.pose','get_local_data')
            if pose['z'] < 3.5 or pose['z'] > 9.0:
                return True # probably unstable.
            if abs(pose['pitch']) >= np.pi/4:
                return True # Unstable large angle deviations are difficult to recover from.
            if abs(pose['roll']) >= np.pi/4:
                return True
        return False
    def unpauseSim(self):
        with Morse(port = self.server_port) as morse:
            morse.rpc('activate','robots_0') # reactivates all dynamics
        return
    
    def softResetSim(self):
        """
        This is a soft reset where the sub-environments remain the same
        but the robots are returned to their initial configuration
        
        This does not break the flow of the simulation and hence is a
        fast operation.
        """
        with Morse(port = self.server_port) as morse:
            morse.rpc('simulation', 'reset_objects') # this is a soft reset.
        return
    def hardResetSim(self):
        # TODO : implement this without breaking the serverport.
        config.close(config.child_processes) # kill all the child processes
        self.hardResetMain()
        raise NotImplementedError     
        
