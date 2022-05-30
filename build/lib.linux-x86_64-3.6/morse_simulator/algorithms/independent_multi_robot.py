# Independent Multi-robot Simulations
"""
This file is deprecated, please run "independent_multi_robot_fast.py"

"""



from Pose import DepthCameraPose
#from wall_control import Control
from wall_control import Control
from odometry import Odometry
import time
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from scipy.interpolate import splprep, splev
# from morse_simulator.algorithms.frontier_based_exploration import run_frontier_exploration, old_run_frontier_exploration
from trajectory_generation import *
from numba import jit

# plt.ion()

@jit(nopython = True)
def cluster_based_on_eucl_distance(a, dist_thresh=10):
    """We use the fact that coordinates form a directed graph."""
    idx = [0]
    for i in range(a.shape[0]):
        for j in range(i + 1, a.shape[0]):
            dist = np.linalg.norm(a[i] - a[j], ord = 2)
            if dist >= dist_thresh:
                idx.append(j)
                break
        if i < j and j != a.shape[0]-1:
            continue
        else:
            return a[np.asarray(idx)]


class MASTER:
    FREQUENCY = 30
    HOW_LONG = 1e5
    SCALE = 1.2
    

    def __init__(self,ports,robot_number, how_long=HOW_LONG, frequency=FREQUENCY, scale=SCALE):


        map2Dpose = plt.figure(linewidth=1, edgecolor='g')
        ax2Dpose = map2Dpose.add_subplot(1, 1, 1)
        map2Dpose.suptitle("Visualization")


        
        self.robot_number = robot_number - 1
        local_ports = ports # Some ports are obsolete and need removing
        videocamera_port = local_ports[0] 
        pose_port = local_ports[1] 
        depthcamera_port = local_ports[2] 
        lidar_port = local_ports[3] 
        imu_port = local_ports[4] 
        laser_port = local_ports[5] 
        right_laser_port = local_ports[6] 
        left_laser_port = local_ports[7] 
        odometry_port = local_ports[8] 
        motion_port = local_ports[9] 
        display_camera_port = local_ports[10]
        self.how_long = how_long
        self.frequency = frequency
        self.scale = scale
        self.ax2Dpose = ax2Dpose
        self.control = Control(self.robot_number, laser_port, right_laser_port, left_laser_port,motion_port,host='127.0.0.1')

        self.odometry_pose = Odometry(odometry_port)
        self.dcam_pose = DepthCameraPose(self.robot_number, depthcamera_port, lidar_port, pose_port)
        time.sleep(1)

    


    def run_simulation_waypoint(self): # For quadrotors with waypoint based navigation
        prev_time = time.time()

        time_end = time.time() + self.how_long
        img_num = 0
        plot_obj = self.ax2Dpose
        start_simulation_ = True
        while time.time() < time_end:
            if time.time() >= prev_time + 1 / self.frequency: # Sync with sensor frequency (Not Really Necessary)
                img_num += 1
                prev_time = time.time()


                if start_simulation_ == True:
                    start_simulation_ = False
                    for i in np.linspace(0, 2*np.pi, 5):
                        full_odo_data = np.array(self.odometry_pose.get_data()) 
                        odo_pos = full_odo_data[:3]
                        plot_obj, _,_ = self.dcam_pose.points_cloud_update(plot_obj, img_num)
                        occ_grid, pose_offset, pose_grid = self.dcam_pose.occupancy_grid_update()
                        self.control.take_random_action(odo_pos[:-1], pose_offset, i)

                    continue

                self.dcam_pose.set_position(odo_pos)
                plot_obj, _,_ = self.dcam_pose.points_cloud_update(plot_obj, img_num)
                occ_grid, pose_offset, pose_grid = self.dcam_pose.occupancy_grid_update()
                print('OCCUPANCY GRID SHAPE : ',occ_grid.shape)
                pose_grid = np.append(np.asarray(pose_grid), 0.).astype(float) # 0. is the yaw (it doesn't matter here)


                if self.robot_number == 1:
                    print('Inferred Location : ', odo_pos[:-1] + pose_offset)


                occ_grid_planning = occ_grid.copy()
                # occ_grid_planning[np.where(occ_grid_planning < 150)] = 0 # TODO: Wrapper for 2 step robust planning
                waypointPathGrid = run_frontier_exploration(map_filename=occ_grid_planning,
                                        params_filename='./params.yaml',
                                        map_resolution=1.0,
                                        start_state=pose_grid,
                                        sensor_range=150.0,
                                        completion_percentage=1.00,
                                        max_exploration_iterations=200,
                                        render=False, save_folder_num = 0) # the sensor range is not in the World Frame.

                if not waypointPathGrid is None: # TODO : Wrap transform into a function
                    #print(waypointPathGrid.shape)
                    desired_yaw = waypointPathGrid[:,-1] #* (np.pi/180)
                    waypointPathGrid = waypointPathGrid[:,:-1]
                    #print('Path : ',waypointPathGrid)
                    waypointPathGrid[:,[0,1]] = waypointPathGrid[:,[1,0]]
                    theta  = (np.pi)
                    r = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
                    waypointPathGrid = np.einsum('ij,kj->ki', r, waypointPathGrid - np.array([occ_grid.shape[1]//2, occ_grid.shape[0]//2])) + np.array([occ_grid.shape[1]//2, occ_grid.shape[0]//2])
                    waypointPathGrid[:,1] = occ_grid.shape[0] - waypointPathGrid[:,1]
                    waypointPathGrid[:,[0,1]] = waypointPathGrid[:,[1,0]]
                    waypointPath = self.dcam_pose.occ.xy_from_grid_cell(waypointPathGrid)
                    # waypointPath = waypointPathGrid * self.dcam_pose.occ.resolution
                    waypointPath = np.add(waypointPath, pose_offset.reshape(2,-1//2)).T
                    print('BEFORE : ',waypointPath.shape)
                    waypointPath = cluster_based_on_eucl_distance(waypointPath, dist_thresh = 5.0)
                    print('AFTER : ',waypointPath.shape)
                    if np.asarray(waypointPath.shape).min() == 0:
                        waypointPath = None
                    # plt.imshow(occ_grid)
                    # plt.cla()
                    # plt.scatter(waypointPath[:,0],  waypointPath[:,1])
                    # plt.draw()
                    # plt.pause(0.05)
                    # waypointPath[:,[0,1]] = waypointPath[:,[1,0]]

                    if not waypointPath is None:

                        if waypointPath.shape[0] > 1:
                            # desired_yaw = desired_yaw[np.arange(0, len(desired_yaw)-1, 25).astype(int)]
                            for h, waypoint in enumerate(waypointPath):
                                
                                #print(f'Setting Waypoint for robot : {self.robot_number}, {waypoint}, yaw : {desired_yaw[h]}')
                                
                                self.control.set_waypoint(waypoint, exp_yaw = 0.)
                                # plot_obj, _,_ = self.dcam_pose.points_cloud_update(plot_obj, img_num) # not required here.
                                # occ_grid, pose_offset, pose_grid = self.dcam_pose.occupancy_grid_update()

                            odo_pos[:-1] = self.dcam_pose.get_current_pose() - pose_offset
                            for i in np.linspace(0, 2*np.pi, 5):
                                self.control.take_random_action(odo_pos[:-1], pose_offset, i)
                            plot_obj, _,_ = self.dcam_pose.points_cloud_update(plot_obj, img_num)
                            occ_grid, pose_offset, pose_grid = self.dcam_pose.occupancy_grid_update()
                        else:
                            waypoint = waypointPath
                            self.control.set_waypoint(waypoint)
                            plot_obj, _,_ = self.dcam_pose.points_cloud_update(plot_obj, img_num)
                            occ_grid, pose_offset, pose_grid = self.dcam_pose.occupancy_grid_update()
                            odo_pos[:-1] = self.dcam_pose.get_current_pose() - pose_offset
                




