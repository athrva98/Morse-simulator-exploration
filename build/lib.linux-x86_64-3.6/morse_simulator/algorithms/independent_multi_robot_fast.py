# Independent Multi-robot Simulations
from morse_simulator.algorithms.Pose import DepthCameraPose
from morse_simulator.algorithms.wall_control import Control
import time
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from numba import jit
import cv2
from morse_simulator.algorithms.util import *
from morse_simulator.algorithms.costmap import Costmap
from morse_simulator.cpp import c_astar
from morse_simulator.algorithms.footprint import CustomFootprint
from morse_simulator.algorithms.config import config



@jit(nopython = True)
def cluster_based_on_eucl_distance(a, dist_thresh=10):
    """We use the fact that coordinates form a directed graph."""
    idx = [0]
    for i in range(a.shape[0]):
        for j in range(i + 1, a.shape[0]):
            dist = np.linalg.norm(a[i] - a[j], ord = 2)
            if dist <= dist_thresh:
                idx.append(j)
                break
        if i < j and j != a.shape[0]-1:
            continue
        else:
            return a[np.asarray(idx)]

        
@jit (nopython = True)
def projectInDirection(v, y):
    """This function projects a point in the direction of the yaw"""
    h = 0.15; # small perturbation
    if y == 0 or y == 360:
        return np.array([v[0] + h, v[1]])
    elif y == 90:
        return np.array([v[0], v[1]+h])
    elif y == 180:
        return np.array([v[0] - h, v[1]])
    elif y == 270:
        return np.array([v[0], v[1]-h])
    else: # handle all the other "normal" cases
        vx_prime = (v[0] + h)/np.cos(y * np.pi/180)
        vy_prime = (v[1] + h)/np.sin(y * np.pi/180)
        return np.array([vx_prime, vy_prime])


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
def calcYaw(v1, v2, y2):
    """
    v1 -> waypoint
    v2 -> current position
    y2 -> current yaw 
    """
    projectedPose = projectInDirection(v2, y2)
    projectedPoseCentered = np.subtract(projectedPose, v2)
    v1Centered = np.subtract(v1, v2)
    angle = angle_between(v1Centered,projectedPoseCentered) * 180/np.pi
    return angle


class planner:
    def __init__(self, mode = 'closest'): # closest is better in terms of safety
        self.callIteration = 0
        self.footprint_points = np.array(
        [[0.22674791, 0],
         [0.22129365, 0.14978179],
         [0.21026903, 0.17772615],
         [0.13216534, 0.22714073],
         [0.07048001, 0.22728987],
         [-0.06165067, 0.2245],
         [-0.12432992, 0.2102427],
         [-0.23903614, 0.16096445],
         [-0.241, 0.15896477],
         [-0.241, -0.15896477],
         [-0.23903614, -0.16096445],
         [-0.12432992, -0.2102427],
         [-0.06165067, -0.2245],
         [0.07048001, -0.22728987],
         [0.13216534, -0.22714073],
         [0.21026903, -0.17772615],
         [0.22129365, -0.14978179]]) # this is just a foorprint, feel free to plot it and see, it's just a non-centered circle.
        self.angular_resolution = 0.01745
        self.footprint_inflation = 1.1 # how much do we want to inflate the footprint.
        self._mode = mode
        
        pass # this is a default constuctor for the planner.
    
    def astarPlanner(self, partial_occ_grid, start_state, robot_number, debug=False): # uses the A* algorithm for planning a path.
        """ Takes in a partial occupancy grid.
        FREE := 255
        OCCUPIED := 0
        UNCERTAIN := 127
        These values are strict. Using other values my cause this algorithm to fail.
        """
        occupancy_map_data = partial_occ_grid # must be unit8
        
        self.occupancy_map = Costmap(occupancy_map_data, 1.0, origin=[0., 0.]) 
        self.occupancy_map = self.cleanup_map_for_planning(self.occupancy_map) # removes noise from the occupancy grid, allows for faster planning.
        start = np.array([start_state[1], start_state[0], 0]) # must be in grid coordinates
        print('[INFO] Start Intensity : ',self.occupancy_map.data[start[0], start[1]])
        try:
            
            assert self.occupancy_map.data[start[0], start[1]] == 255
            
        except:
            print('INVALID START STATE')
        
        goal = self.sampleGoalFromFrontiers(start) # center of the largest frontier
        if goal is None: # if there are no frontiers left
            return None # planner has failed.
        goal = np.array([goal.flatten()[0], goal.flatten()[1], 0.])
        print('[INFO] Goal Intensity : ',self.occupancy_map.data[int(goal[0]), int(goal[1])])
        if debug == True:
            show_img_ = cv2.cvtColor(self.occupancy_map.data, cv2.COLOR_GRAY2BGR) 
            show_img_[np.max([0, start_state[1]-5]):np.min([show_img_.shape[1], start_state[1]+5]),
                    np.max([0, start_state[0]-5]):np.min([show_img_.shape[0], start_state[0]+5])] = (255,0, 75)
            show_img_[np.max([0, int(goal[0])-5]):np.min([show_img_.shape[1], int(goal[0])+5]),
                    np.max([0, int(goal[1])-5]):np.min([show_img_.shape[0], int(goal[1])+5])] = (0,0,255)
            cv2.imshow(f'OCC_PATH_{robot_number}',show_img_)
            cv2.waitKey(25)
        


        obstacle_values = np.array([0, 127], dtype = np.uint8)
        #self.callIteration += 1
        success, path = self.astar(goal=goal,
                                       start=start,
                                       occupancy_map=self.occupancy_map,
                                       obstacle_values=obstacle_values,
                                       planning_scale=1)
        #print(f'Calling Iteration Number for {robot_number}: ',self.callIteration)
        
        path[:,[0,1]] = path[:,[1,0]] # Dirty fix.
        return path.astype(np.int)
    def _rankFrontiersBasedOnMetric(self, state, frontiers):
        
        if self._mode == 'closest':
            frontier_distances = [np.min(np.sqrt(np.sum((state[:2] - frontier) ** 2, axis=1)))
                                  for frontier in frontiers]
            frontier_ranks = np.argsort(frontier_distances)
        elif self._mode == 'largest':
            frontier_sizes = [frontier.shape[0] for frontier in frontiers]
            frontier_ranks = np.argsort(frontier_sizes)[::-1]
        else:
            frontier_ranks = []
            assert False and "Mode not supported"

        return frontier_ranks
    def cleanup_map_for_planning(self, occupancy_map, filter_obstacles=False, debug=False):
        """
        cleans up the occupancy grid and removes any small unexplored areas. Important because we have a limited FoV depth camera.
        """
        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        occupied_coords = np.argwhere(occupancy_map.data == Costmap.OCCUPIED)
        free_coords = np.argwhere(occupancy_map.data == Costmap.FREE)

        free_mask = np.zeros_like(occupancy_map.data)
        free_mask[free_coords[:, 0], free_coords[:, 1]] = 1
        free_mask = cv2.dilate(free_mask, kernel=kernel, iterations=2)
        free_mask = cv2.erode(free_mask, kernel=kernel, iterations=2)
        new_free_coords = np.argwhere(free_mask == 1)

        if filter_obstacles:
            occupied_mask = np.zeros_like(occupancy_map.data)
            occupied_mask[occupied_coords[:, 0], occupied_coords[:, 1]] = 1
            occupied_mask = cv2.medianBlur(occupied_mask, kernel.shape[0])
            occupied_coords = np.argwhere(occupied_mask == 1)

        cleaned_occupancy_map = occupancy_map.copy()
        cleaned_occupancy_map.data[new_free_coords[:, 0], new_free_coords[:, 1]] = Costmap.FREE
        cleaned_occupancy_map.data[occupied_coords[:, 0], occupied_coords[:, 1]] = Costmap.OCCUPIED

        if debug:
            plt.imshow(cleaned_occupancy_map.data, cmap='gray', interpolation='nearest')
            plt.show()

        return cleaned_occupancy_map
    def findFrontierCenter(self, best_frontier, state): # the state -> the start state
        footprint = CustomFootprint(footprint_points=self.footprint_points,
                                angular_resolution=self.angular_resolution,
                                inflation_scale=self.footprint_inflation)
        # best_frontier = frontiers[frontier_ranks[frontier_idx]]
        assert len(best_frontier.shape) == 2 and best_frontier.shape[0] > 0 # the frontier should be valid.
        # navigate to the middle of the best frontier
        frontier_mean = np.mean(best_frontier, axis=0)
        desired_coord_ranks = np.argsort(np.sqrt(np.sum((frontier_mean - best_frontier) ** 2, axis=1)))
        # todo try more angles
        start_frontier_vector = best_frontier[desired_coord_ranks[0]] - state[:2]
        angle_to_frontier = wrap_angles(np.arctan2(start_frontier_vector[0], start_frontier_vector[1]))
        # find a point near the desired coord where our footprint fits
        goal = None
        for _, ind in enumerate(desired_coord_ranks):
            candidate_state = np.concatenate((np.array(best_frontier[ind]).squeeze(), [angle_to_frontier]))
            if not footprint.check_for_collision(state=candidate_state, occupancy_map=self.occupancy_map):
                goal = candidate_state
                break
        return goal
    def extract_frontiers(self, occupancy_map, approx=True, approx_iters=2,
                      kernel=cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))):
        # todo regional frontiers
        # extract coordinates of occupied, unexplored, and free coordinates
        occupied_coords = np.argwhere(occupancy_map.data.astype(np.uint8) == Costmap.OCCUPIED)
        unexplored_coords = np.argwhere(occupancy_map.data.astype(np.uint8) == Costmap.UNEXPLORED)
        free_coords = np.argwhere(occupancy_map.data.astype(np.uint8) == Costmap.FREE)

        if free_coords.shape[0] == 0 or unexplored_coords.shape[0] == 0:
            print('Either the map is completely explored or invalid')
            return []

        # create a binary mask of unexplored pixels, letting unexplored pixels = 1
        unexplored_mask = np.zeros_like(occupancy_map.data)
        unexplored_mask[unexplored_coords[:, 0], unexplored_coords[:, 1]] = 1

        # dilate using a 3x3 kernel, effectively increasing
        # the size of the unexplored space by one pixel in all directions
        dilated_unexplored_mask = cv2.dilate(unexplored_mask, kernel=kernel)
        dilated_unexplored_mask[occupied_coords[:, 0], occupied_coords[:, 1]] = 1

        # create a binary mask of the free pixels
        free_mask = np.zeros_like(occupancy_map.data)
        free_mask[free_coords[:, 0], free_coords[:, 1]] = 1

        # can isolate the frontiers using the difference between the masks,
        # and looking for contours
        frontier_mask = ((1 - dilated_unexplored_mask) - free_mask)
        if approx:
            frontier_mask = cv2.dilate(frontier_mask, kernel=kernel, iterations=approx_iters)
            frontier_mask = cv2.erode(frontier_mask, kernel=kernel, iterations=approx_iters)

        # this indexing will work with opencv 2.x 3.x and 4.x
        frontiers_xy_px = cv2.findContours(frontier_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[-2:][0]
        frontiers = [np.array(frontier).squeeze(1)[:, ::-1] for frontier in frontiers_xy_px]
        return frontiers # frontiers in the Grid Frame
    
    def sampleGoalFromFrontiers(self,state, debug=False, min_frontier_length = 20, approxSafe = False):
        
        occupancy_map = self.occupancy_map
        # frontierDict = {} # stores the frontier and it's length
        frontiers = self.extract_frontiers(occupancy_map=occupancy_map, approx=False,
                                      kernel=cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3)))
        frontier_sizes = np.array([frontier.shape[0] if len(frontier.shape) > 1 else 1 for frontier in frontiers])
        significant_frontiers = [frontier for i, frontier in enumerate(frontiers) if frontier_sizes[i] > min_frontier_length]
        if len(significant_frontiers) > 0: 
            if self._mode.lower() == 'largest':
                largestFrontier = frontiers[np.argmax(frontier_sizes)]
                if approxSafe == True: # approximate but faster
                    goal = largestFrontier[len(largestFrontier)//3] # closer than the center of the largest frontier for safety (the points are ordered)
                else:
                    goal = self.findFrontierCenter(largestFrontier, state)
                # goal = xy_to_rc(goal, occupancy_map).astype(np.int)
            if self._mode.lower() == 'closest':
                frontier_distances = [np.min(np.sqrt(np.sum((state[:2] - frontier) ** 2, axis=1))) for frontier in frontiers]
                frontier_ranks = np.argsort(frontier_distances)
                closestFrontier = frontiers[frontier_ranks[0]]
                if approxSafe == True: # approximate but faster
                    goal = closestFrontier[len(largestFrontier)//3] # closer than the center of the largest frontier for safety (the points are ordered)
                else:
                    goal = self.findFrontierCenter(closestFrontier, state)
                
        else:
            goal = None # handle this case
        return goal
    def astar(self, goal, start, occupancy_map, obstacle_values, planning_scale=1, delta=0.0, epsilon=1.0, allow_diagonal=False):
        #start_px = xy_to_rc(start, occupancy_map)
        c_start = np.array(start[:2], dtype=np.int32)

        #goal_px = xy_to_rc(goal, occupancy_map)
        c_goal = np.array(goal[:2], dtype=np.int32)

        c_occupancy_map = occupancy_map.data.astype(np.uint8)

        obstacle_values = np.asarray(obstacle_values).astype(np.uint8)
        
        success, path_px = c_astar(c_start,
                                   c_goal,
                                   c_occupancy_map,
                                   obstacle_values,
                                   delta,
                                   epsilon,
                                   planning_scale,
                                   allow_diagonal)

        return success, path_px
    
def inflate(occ, obs):
    for i in range(obs.shape[-1]):
        occ[np.max([0, obs[1,i]-5]):np.min([occ.shape[1], obs[1,i]+5]),
                        np.max([0, obs[0,i]-5]):np.min([occ.shape[0], obs[0,i]+5])] = (255,0,75)
    return occ

class MASTER:
    FREQUENCY = 30 # this is the dcam frequency.
    SIM_MAX_DURATION = 1e5 # some large number that actually doesn't matter
    
    

    def __init__(self,ports,robot_number, sim_max_duration=SIM_MAX_DURATION, frequency=FREQUENCY):


        map2Dpose = plt.figure(linewidth=1, edgecolor='g')
        ax2Dpose = map2Dpose.add_subplot(1, 1, 1)
        map2Dpose.suptitle("Visualization")
        
        
        self.robot_number = robot_number - 1
        
        local_ports = ports # Some ports are obsolete and need removing
        pose_port = local_ports[0] 
        depthcamera_port = local_ports[1] 
        motion_port = local_ports[2]
        display_camera_port = local_ports[3] # Not really used (WE USE THE DEFAULT BLENDER VIEWPORT CAMERA)
        
        self.sim_max_duration = sim_max_duration
        self.frequency = frequency
        self.ax2Dpose = ax2Dpose
        self.control = Control(self.robot_number, motion_port,host='127.0.0.1')
        self.planner = planner()
        self.dcam_pose = DepthCameraPose(self.robot_number, depthcamera_port, pose_port)
        time.sleep(1)

    


    def run_simulation_waypoint(self): # For quadrotors with waypoint based navigation
        prev_time = time.time()
        
        config.startTime = prev_time
        
        time_end = time.time() + self.sim_max_duration # this is the hypothetical end time if all other stopping mechanisms fail.
        img_num = 0 # keeps track of the incoming images
        plot_obj = self.ax2Dpose
        start_simulation_ = True
        initialYaw = 0. # this is the yaw value initially, will be used to correct yaw later.
        odo_pos = np.array([0.,0.,0.])
        
        config.initialPosition = self.dcam_pose.get_current_pose()
        
        while time.time() < time_end:
            if time.time() >= prev_time + 1 / self.frequency: # Sync with sensor frequency (reduces un-necesary compute)
                img_num += 1
                prev_time = time.time()


                if start_simulation_ == True:
                    start_simulation_ = False
                    
                    for i in np.linspace(0, 2*np.pi, 15): # simply scan in position first to generate some frontiers
                        odo_pos[:-1] = self.dcam_pose.get_current_pose()
                        plot_obj, _,_ = self.dcam_pose.points_cloud_update(plot_obj, img_num)
                        occ_grid, pose_offset, pose_grid = self.dcam_pose.occupancy_grid_update()
                        odo_pos[:-1] = odo_pos[:-1] - pose_offset
                        self.control.take_random_action(odo_pos[:-1], pose_offset, i)

                    continue
                plot_obj, _,_ = self.dcam_pose.points_cloud_update(plot_obj, img_num)
                occ_grid, pose_offset, pose_grid = self.dcam_pose.occupancy_grid_update()
                print('OCCUPANCY GRID SHAPE : ',occ_grid.shape)
                pose_grid = np.append(np.asarray(pose_grid), 0.).astype(int) # 0. is the yaw (it doesn't matter here)


                if self.robot_number == 1:
                    print('Inferred Location : ', odo_pos[:-1] + pose_offset)


                occ_grid_planning = occ_grid.copy()
                # occ_grid_planning[np.where(occ_grid_planning < 150)] = 0 # TODO: Wrapper for 2 step robust planning
                
                try:
                    waypointPathGrid = self.planner.astarPlanner(occ_grid_planning, start_state = pose_grid, robot_number = self.robot_number) # Plans based on Frontiers.
                    
                except IndexError: # this is a rather un-explainable error.
                    print('ERROR : SOMETHING WENT WRONG IN THE PLANNER')
                    waypointPathGrid = None
                    
                
                

                if not waypointPathGrid is None: # TODO : Wrap transform into a function
                    
                    waypointPath = self.dcam_pose.occ.xy_from_grid_cell(waypointPathGrid) # converts to egocentric frame
                    waypointPath = np.add(waypointPath, pose_offset.reshape(2,-1//2)).T # converts to world frame
                    assert waypointPath.shape[-1] == 2
                    
                    if waypointPath.shape[0] == 0:
                        waypointPath = None
                    
                    if not waypointPath is None:

                        if waypointPath.shape[0] > 1:
                            # desired_yaw = desired_yaw[np.arange(0, len(desired_yaw)-1, 25).astype(int)]
                            if waypointPath.shape[0] > 160:
                                waypointPath = waypointPath[np.linspace(0, waypointPath.shape[0]-1, 60).astype(int)]
                            else: # if the planner fails, we scan the area by rotating and hoe to find new frontiers.
                                odo_pos[:-1] = self.dcam_pose.get_current_pose() - pose_offset
                                for i in np.linspace(0, 2*np.pi, 7):
                                    self.control.take_random_action(odo_pos, pose_offset, i)
                            for h, waypoint in tqdm(enumerate(waypointPath)):
                                
                                currentPose, yaw = self.dcam_pose.get_current_pose(return_yaw = True)
                                desiredYaw = calcYaw(waypoint, currentPose, yaw)
                                
                                if h % 10 == 0:
                                    #print(f'Setting Waypoint for robot : {self.robot_number}, {waypoint}, yaw : {desired_yaw[h]}')

                                    self.control.set_waypoint(waypoint, exp_yaw = desiredYaw) # this alligns the robot in the motion direction.
                                    write_data = False
                                    if h % 20 == 0:
                                        write_data = False
                                        plot_obj, _,_ = self.dcam_pose.points_cloud_update(plot_obj, img_num, write_data = write_data)
                                        occ_grid, pose_offset, pose_grid = self.dcam_pose.occupancy_grid_update()


                            odo_pos[:-1] = self.dcam_pose.get_current_pose() - pose_offset
                            
                else: # if the planner fails, we scan the area by rotating and hoe to find new frontiers.
                    odo_pos[:-1] = self.dcam_pose.get_current_pose() - pose_offset
                    for i in np.linspace(0, 2*np.pi, 3):
                        self.control.take_random_action(odo_pos, pose_offset, i)
                        
                    
                    



