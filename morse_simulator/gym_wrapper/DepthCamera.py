# This file contains the DepthCamera class for getting the current pose, and the occupancy grid.
import struct
import scipy
import numpy as np
from morse_simulator.algorithms.depth_camera_server import DepthCameraServer
from morse_simulator.algorithms.sensors_classes import Pose_server
import numba
import os
import matplotlib.pyplot as plt
from morse_simulator.algorithms.serialization import *
import cv2
from scipy.spatial import distance
from morse_simulator.algorithms.config import config
import morse_simulator.gym_wrapper.config as dynamic_cfg

np.seterr('raise') # raise all errors

class map_t:
    """
    This will maintain the occupancy grid and log_odds.
    """
    def __init__(s, resolution=0.11):
        s.resolution = resolution
        s.xmin, s.xmax = 0, 75
        s.ymin, s.ymax = 0, 75
        s.szx = int(np.ceil((s.xmax-s.xmin)/s.resolution+1))
        s.szy = int(np.ceil((s.ymax-s.ymin)/s.resolution+1))

        # binarized map and log-odds
        s.cells = np.zeros((s.szx, s.szy), dtype=np.uint8)
        s.log_odds = np.zeros(s.cells.shape, dtype=np.float64)
        s.log_odds_max = 5e6
        s.num_obs_per_cell = np.zeros(s.cells.shape, dtype=np.uint64)
        s.occupied_prob_thresh = 0.6
        s.log_odds_thresh = np.log(s.occupied_prob_thresh/(1-s.occupied_prob_thresh))
    
    def grid_cell_from_xy(s, x, y):
        p_x = np.zeros(x.shape, dtype = np.int)
        p_y = np.zeros(y.shape, dtype = np.int)
        p_x = (((x + (s.xmax - s.xmin)/2)/(s.resolution)) + 1).astype(int)
        p_y = (((-1*y + (s.ymax - s.ymin)/2)/(s.resolution)) + 1).astype(int)
        c_x, c_y = int(np.ceil((s.xmax/(s.resolution))+1)), int(np.ceil((s.ymax/(s.resolution))+1))
        
        p_idx_x = np.where(p_x - c_x < c_x)[0]
        p_idx_y = np.where(p_y - c_y < c_y)[0]
        p_valid_idx = np.intersect1d(p_idx_x, p_idx_y)
        return np.vstack([p_x[p_valid_idx].reshape(1,-1), p_y[p_valid_idx].reshape(1,-1)]), p_valid_idx

    def xy_from_grid_cell(s, coords):
        coords = coords.reshape(-1//2,2)
        cx = coords[:,0]
        cy = coords[:,1]
        x = s.resolution * (cx - 1) - (s.xmax - s.xmin)/2
        y = -1*(s.resolution * (cy - 1) - (s.ymax - s.ymin)/2)
        return np.vstack([x.reshape(1,-1), y.reshape(1,-1)])

@numba.jit (nopython = True)
def rotation(x, y, z, yaw, pitch, roll, cam_rotate): # fast rotation into world frame using numba
    xyz = np.array([x, y, z])
    pitch -= cam_rotate
    yaw_cos = np.cos(yaw)
    yaw_sin = np.sin(yaw)
    pitch_cos = np.cos(pitch)
    pitch_sin = np.sin(pitch)
    roll_cos = np.cos(roll)
    roll_sin = np.sin(roll)
    r1 = [yaw_cos * pitch_cos, yaw_cos * pitch_sin * roll_sin - yaw_sin * roll_cos,
            yaw_cos * pitch_sin * roll_cos + yaw_sin * roll_sin]
    r2 = [yaw_sin * pitch_cos, yaw_sin * pitch_sin * roll_sin + yaw_cos * roll_cos,
            yaw_sin * pitch_sin * roll_cos - yaw_cos * roll_sin]
    r3 = [-pitch_sin, pitch_cos * roll_sin, pitch_cos * roll_cos]
    r = np.array([r1, r2, r3])
    xyz = np.dot(r, xyz)
    return xyz[0], xyz[1], xyz[2]

class DepthCameraPose:
    def _superIndex(self):
        self.sIdx = int(np.floor(self.robot_number//config.num_robots_per_senv)) 
    def __init__(self,robot_number, depth_camera_port, pose_port):
        print(type(depth_camera_port), type(pose_port))
        # starting position of a robot
        self.robot_number = robot_number

        self.depth_camera_server = DepthCameraServer('localhost', depth_camera_port)
        self.depth_camera_server.run()

        self.pose_server = Pose_server('localhost', pose_port)
        self.pose_server.run()
        self.occ = map_t()
        self.first_update = True
        self.initial_offset = True
        self._superIndex()
        # create_robot_folder(self.sIdx, self.robot_number)
        time.sleep(1) # some time to initialize the servers
        

    def rotation(self, x, y, z, yaw, pitch, roll, cam_rotate):
        
        xyz = scipy.array([x, y, z])

        xyz[0], xyz[1], xyz[2] = rotation(x, y, z, yaw, pitch, roll, cam_rotate)
        
        return xyz[0], xyz[1], xyz[2]

    def get_data(self):
        
        points = self.depth_camera_server.get_points()
        pose_stream = self.pose_server.get_all()
        pose_x = pose_stream['x']
        pose_y = pose_stream["y"]
        pose_z = pose_stream["z"]
        yaw = round(pose_stream['yaw'], 1)
        pitch = round(pose_stream['pitch'], 1)
        roll = round(pose_stream['roll'], 1)
        return points, pose_x, pose_y, pose_z, yaw, pitch, roll

    def get_current_pose(self, return_yaw = False):
        pose_stream = self.pose_server.get_all()
        pose_x = pose_stream['x']
        pose_y = pose_stream["y"]
        if return_yaw == False:
            return np.array([pose_x, pose_y])
        else:
            yaw = round(pose_stream['yaw'], 1)
            return np.array([pose_x, pose_y]), yaw




    def points_cloud_update(self):
        """
        For making an occupancy grid, we nned the pose and the bearing/angle between the rays (for a laser). In this case, since we have a depth camera, we need 
        the output image which contains the depths and the camera FOV and image size.
        """
        
        points, pose_x, pose_y, pose_z, yaw, pitch, roll = self.get_data()
        self.pose_ = [pose_x, pose_y]
        if self.initial_offset == True:
            self.initial_pose = np.asarray([pose_x, pose_y])
            self.initial_offset = False
        x, y, z = [], [], []
        rawPoints = points
        for i in range(0, len(points) - 12, 12):
            xyz = struct.unpack('fff', points[i:i + 12])
            
            x1p, y1p, z1p = self.rotation(xyz[2], xyz[0], xyz[1], yaw, pitch, roll, math.pi/8)
            
            xp = round(x1p + pose_x, 1)
            yp = round(y1p + pose_y, 1)
            zp = round(z1p + pose_z, 1)
            x.append(xp)
            y.append(yp)
            z.append(zp)
        self.depth_data_ = [x,y,z]
        return 1
    
    def ray_tracing(self, ends): # TODO : Speed this function using Numba/Cython/C++ bindings
        """ Uses the Bresenham algorithm """
        def connect2(ends):
            d0, d1 = np.diff(ends, axis=0)[0]
            if not d0 == 0 and not d1 == 0:
                if np.abs(d0) > np.abs(d1): 
                    return np.c_[np.arange(ends[0, 0], ends[1,0] + np.sign(d0), np.sign(d0), dtype=np.int32),
                                 np.arange(ends[0, 1] * np.abs(d0) + np.abs(d0)//2,
                                           ends[0, 1] * np.abs(d0) + np.abs(d0)//2 + (np.abs(d0)+1) * d1, d1, dtype=np.int32) // np.abs(d0)]
                else:
                    return np.c_[np.arange(ends[0, 0] * np.abs(d1) + np.abs(d1)//2,
                                           ends[0, 0] * np.abs(d1) + np.abs(d1)//2 + (np.abs(d1)+1) * d0, d0, dtype=np.int32) // np.abs(d1),
                                 np.arange(ends[0, 1], ends[1,1] + np.sign(d1), np.sign(d1), dtype=np.int32)]
            elif d0 == 0:
                #ret_val = np.arange(ends[0, 1], ends[1,1] + np.sign(d1), np.sign(d1), dtype=np.int32)
                ret_val = np.arange(min([ends[0,1], ends[1,1]]), max([ends[0,1], ends[1,1]]), dtype=np.int32)
                return np.c_[np.ones_like(ret_val)*ends[0,0],
                                 ret_val]
            elif d1 == 0:
                ret_val = np.arange(min([ends[0,0], ends[1,0]]), max([ends[0,0], ends[1,0]]), dtype=np.int32)
                return np.c_[ret_val,
                                 np.ones_like(ret_val)*ends[0,1]]
        return connect2(ends)

    def _inflate_obstacles(self, occupancy_grid, obstacle_coordinates): # inflating the obstacles to avoid collisions
        # @numba.jit (nopython = True)
        def inflate(occ, obs):
            for i in range(obs.shape[-1]):
                occ[np.max([0, obs[1,i]-5]):np.min([occ.shape[1], obs[1,i]+5]),
                                np.max([0, obs[0,i]-5]):np.min([occ.shape[0], obs[0,i]+5])] = 0
            return occ
        return inflate(occupancy_grid, obstacle_coordinates)

    def occupancy_grid_update(self): # this is only used for navigation
        xdepth, ydepth, zdepth = self.depth_data_
        posex, posey = self.pose_
        obstacles = np.stack([xdepth, ydepth], axis = 0).astype(np.int32)
        if self.first_update == True:
            #print('Initializing Occupancy Grid')
            self.occ_map = np.ones_like(self.occ.cells, dtype = np.uint8) * 127 # initially everything is uncertain
            self.first_update = False
            self.pose_offset = np.array([posex, posey])
        
        xdepth.append(posex) # Adding the pose in the hull points to constraint the occupied region (like bresenham algorithm but easier)
        ydepth.append(posey)
        hull_points = np.stack([np.asarray(xdepth), np.asarray(ydepth)], axis = 0)
        hull_points = np.subtract(hull_points, self.pose_offset.reshape(2,1))

        #print('hull_points shape : ', hull_points.shape)
        grid_p, _ = self.occ.grid_cell_from_xy(hull_points[0,:], hull_points[1,:])
        start = None
        start = grid_p.T[-1].reshape(1,2)
        for i in range(grid_p.T.shape[0] - 1):
            start = grid_p.T[-1].reshape(1,2)
            end = grid_p.T[i].reshape(1,2)

            ends = np.concatenate([start, end], axis = 0)
            pts = self.ray_tracing(ends).astype(np.int32)
            mask = np.logical_and([pts[:,1] < self.occ_map.shape[0]],[pts[:,0] < self.occ_map.shape[1]])[0]
            self.occ_map[pts[:,1][mask], pts[:,0][mask]] = 255
        if not start is None:
            self.occ_map[max([0, start[0,1]-5]):min([self.occ_map.shape[1], start[0,1]+5]),
                            max([0, start[0,0]-5]):min([self.occ_map.shape[0], start[0,0]+5])] = 255 # account for robot footprint.
        #plt.show()
        #cv2.drawContours(image=self.occ_map, contours = [grid_p.T], contourIdx = -1, color = 255, thickness=-1)
        mask = np.logical_and([grid_p[1,:-1] < self.occ_map.shape[0]],[grid_p[0,:-1] < self.occ_map.shape[1]])[0]
        self.occ_map[grid_p[1,:-1][mask], grid_p[0,:-1][mask]] = 0 # obstacles are occupied.
        self.occ_map = self._inflate_obstacles(self.occ_map, np.stack([grid_p[0,:-1][mask],grid_p[1,:-1][mask]], axis = 0))
        
        
        dynamic_cfg.occupancy_grid = self.occ_map # this makes the occupancy grid available to the RL algorithms.
        # plt.figure()
        # plt.imshow(self.occ_map)
        # plt.scatter([start[0,0], start[0,1]], [start[0,1], start[0,0]])
        # plt.show()
        # cv2.imshow(f'Map_{self.robot_number}', self.occ_map)
        # cv2.waitKey(15)
        # cv2.imwrite('./test.png', self.occ_map)
        #print('Pose is : ', start)
        #cv2.destroyAllWindows()
        return self.occ_map, self.pose_offset, start.reshape(-1,1) # returns the current occupancy map, pose_offset, and the pose in grid frame.
    
    
    
    
    
    
    
    